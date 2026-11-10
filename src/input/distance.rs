use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::SPEED_OF_SOUND;
use super::smoothed::SmoothShared;
use crate::clock::{Event, Task, lock};
use crate::devices::Values;
use crate::error::{Result, invalid};
use crate::pins::{Edges, Pin, PinFactory, PinFunction, PinHandle, Pull, resolve};

/// Options for [`DistanceSensor`].
#[derive(Clone)]
pub struct DistanceSensorOptions {
    /// Readings averaged (median, default 9).
    pub queue_len: usize,
    /// Maximum distance in meters; `value()` is `distance / max_distance` (default 1 m).
    pub max_distance: f64,
    /// Objects closer than this are "in range" (default 0.3 m).
    pub threshold_distance: f64,
    /// Report readings before the queue is full (default `true`).
    pub partial: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for DistanceSensorOptions {
    fn default() -> Self {
        DistanceSensorOptions {
            queue_len: 9,
            max_distance: 1.0,
            threshold_distance: 0.3,
            partial: true,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for DistanceSensorOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DistanceSensorOptions")
            .field("queue_len", &self.queue_len)
            .field("max_distance", &self.max_distance)
            .field("threshold_distance", &self.threshold_distance)
            .field("partial", &self.partial)
            .finish_non_exhaustive()
    }
}

#[derive(Default)]
struct Echo {
    rise: Option<Duration>,
    fall: Option<Duration>,
}

struct DistanceInner {
    task: Option<Task>,
    wake: [Event; 2],
    shared: Arc<SmoothShared>,
    echo: PinHandle,
    trigger: PinHandle,
    max_distance: f64,
}

impl Drop for DistanceInner {
    fn drop(&mut self) {
        // Stop the sampler, waking it if it is waiting for an echo.
        let mut task = self.task.take();
        if let Some(t) = &mut task {
            t.signal();
        }
        for e in &self.wake {
            e.set();
        }
        drop(task);
    }
}

/// An HC-SR04 ultrasonic distance sensor.
///
/// Wire the echo pin through a voltage divider: the sensor outputs 5 V.
#[derive(Clone)]
pub struct DistanceSensor {
    inner: Arc<DistanceInner>,
}

impl fmt::Debug for DistanceSensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DistanceSensor")
            .field("distance", &self.distance())
            .finish()
    }
}

/// Time between measurements (the HC-SR04 datasheet asks for >= 60 ms).
const CYCLE: Duration = Duration::from_millis(60);

impl DistanceSensor {
    /// Creates a sensor with the given echo and trigger pins.
    pub fn new(echo: u32, trigger: u32) -> Result<Self> {
        Self::with_options(echo, trigger, DistanceSensorOptions::default())
    }

    /// Creates a sensor with explicit options.
    pub fn with_options(echo: u32, trigger: u32, o: DistanceSensorOptions) -> Result<Self> {
        if o.max_distance <= 0.0 {
            return invalid("max_distance must be positive");
        }
        if !(0.0..=o.max_distance).contains(&o.threshold_distance) {
            return invalid("threshold_distance must be between 0 and max_distance");
        }
        let factory = resolve(&o.pin_factory)?;
        let trig = PinHandle::new(&factory, trigger)?;
        trig.set_function(PinFunction::Output)?;
        trig.set_state(0.0)?;
        let echo_pin = PinHandle::new(&factory, echo)?;
        echo_pin.set_function(PinFunction::Input)?;
        echo_pin.set_pull(Pull::Floating)?;
        echo_pin.set_edges(Edges::Both)?;
        let state: Arc<Mutex<Echo>> = Arc::default();
        let (rose, fell) = (Event::new(), Event::new());
        let (rose2, fell2) = (rose.clone(), fell.clone());
        {
            let state = state.clone();
            let (rose, fell) = (rose.clone(), fell.clone());
            echo_pin.set_when_changed(Some(Arc::new(move |ticks, level| {
                let mut st = lock(&state);
                if level {
                    st.rise = Some(ticks);
                    rose.set();
                } else if st.rise.is_some() {
                    st.fall = Some(ticks);
                    fell.set();
                }
            })))?;
        }
        let shared = SmoothShared::new(
            factory.clock(),
            o.queue_len,
            o.partial,
            o.threshold_distance / o.max_distance,
        )?;
        let clock = factory.clock();
        let tp: Arc<dyn Pin> = trig.pin.clone();
        let max = o.max_distance;
        let task = shared.start(CYCLE, move |stop| {
            *lock(&state) = Echo::default();
            rose.clear();
            fell.clear();
            tp.set_state(1.0).ok()?;
            clock.wait(stop, Some(Duration::from_micros(10)));
            tp.set_state(0.0).ok()?;
            if !clock.wait(&rose, Some(Duration::from_millis(50))) {
                return Some(1.0);
            }
            if !clock.wait(&fell, Some(Duration::from_millis(50))) {
                return Some(1.0);
            }
            let st = lock(&state);
            let width = st.fall?.saturating_sub(st.rise?);
            let d = width.as_secs_f64() * SPEED_OF_SOUND / 2.0;
            Some((d / max).min(1.0))
        });
        Ok(DistanceSensor {
            inner: Arc::new(DistanceInner {
                task: Some(task),
                wake: [rose2, fell2],
                shared,
                echo: echo_pin,
                trigger: trig,
                max_distance: o.max_distance,
            }),
        })
    }

    /// Smoothed distance in meters (capped at `max_distance`).
    pub fn distance(&self) -> f64 {
        self.value() * self.inner.max_distance
    }

    /// `distance / max_distance`, `0.0..=1.0`.
    pub fn value(&self) -> f64 {
        self.inner.shared.value()
    }

    /// Maximum distance in meters.
    pub fn max_distance(&self) -> f64 {
        self.inner.max_distance
    }

    /// The in-range threshold in meters.
    pub fn threshold_distance(&self) -> f64 {
        self.inner.shared.threshold() * self.inner.max_distance
    }

    /// Changes the in-range threshold.
    pub fn set_threshold_distance(&self, meters: f64) {
        self.inner
            .shared
            .set_threshold(meters / self.inner.max_distance);
    }

    /// `true` when something is closer than `threshold_distance`.
    pub fn in_range(&self) -> bool {
        !self.inner.shared.is_active()
    }

    /// Runs `f` when an object comes within range.
    pub fn when_in_range(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_deactivated(Some(Arc::new(f)));
    }

    /// Runs `f` when the object leaves the range.
    pub fn when_out_of_range(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_activated(Some(Arc::new(f)));
    }

    /// Blocks until something is in range (or `timeout`).
    pub fn wait_for_in_range(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(false, timeout)
    }

    /// Blocks until nothing is in range (or `timeout`).
    pub fn wait_for_out_of_range(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(true, timeout)
    }

    /// An endless iterator over `value()`.
    pub fn values(&self) -> Values<f64> {
        self.inner.shared.values()
    }

    /// Echo and trigger GPIO numbers.
    pub fn pins(&self) -> (u32, u32) {
        (self.inner.echo.number(), self.inner.trigger.number())
    }
}
