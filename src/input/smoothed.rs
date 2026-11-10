use std::collections::VecDeque;
use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::resolve_pull;
use crate::clock::{Clock, Event, Task, lock};
use crate::devices::{Callback, Values};
use crate::error::{Result, invalid};
use crate::pins::{Edges, Pin, PinFactory, PinFunction, PinHandle, resolve};

fn median(v: &mut [f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    }
}

/// The sample queue of a smoothed input plus its event machinery.
pub(crate) struct SmoothShared {
    pub(crate) clock: Arc<dyn Clock>,
    queue: Mutex<VecDeque<f64>>,
    queue_len: usize,
    partial: bool,
    threshold: Mutex<f64>,
    active: Mutex<Option<bool>>,
    activated: Event,
    deactivated: Event,
    when_activated: Mutex<Option<Callback>>,
    when_deactivated: Mutex<Option<Callback>>,
}

impl SmoothShared {
    pub(crate) fn new(
        clock: Arc<dyn Clock>,
        queue_len: usize,
        partial: bool,
        threshold: f64,
    ) -> Result<Arc<Self>> {
        if queue_len == 0 {
            return invalid("queue_len must be at least 1");
        }
        Ok(Arc::new(SmoothShared {
            clock,
            queue: Mutex::new(VecDeque::with_capacity(queue_len)),
            queue_len,
            partial,
            threshold: Mutex::new(threshold),
            active: Mutex::new(None),
            activated: Event::new(),
            deactivated: Event::new(),
            when_activated: Mutex::new(None),
            when_deactivated: Mutex::new(None),
        }))
    }

    fn ready_value(&self) -> Option<f64> {
        let q = lock(&self.queue);
        if q.is_empty() || (!self.partial && q.len() < self.queue_len) {
            return None;
        }
        let mut v: Vec<f64> = q.iter().copied().collect();
        Some(median(&mut v))
    }

    pub(crate) fn value(&self) -> f64 {
        self.ready_value().unwrap_or(0.0)
    }

    pub(crate) fn is_active(&self) -> bool {
        lock(&self.active).unwrap_or(false)
    }

    pub(crate) fn push(&self, v: f64) {
        {
            let mut q = lock(&self.queue);
            q.push_back(v);
            while q.len() > self.queue_len {
                q.pop_front();
            }
        }
        let Some(value) = self.ready_value() else {
            return;
        };
        let active = value > *lock(&self.threshold);
        let prev = lock(&self.active).replace(active);
        if active {
            self.deactivated.clear();
            self.activated.set();
        } else {
            self.activated.clear();
            self.deactivated.set();
        }
        if prev.is_some_and(|p| p != active) {
            let cb = if active {
                lock(&self.when_activated).clone()
            } else {
                lock(&self.when_deactivated).clone()
            };
            if let Some(cb) = cb {
                cb();
            }
        }
    }

    pub(crate) fn set_threshold(&self, t: f64) {
        *lock(&self.threshold) = t;
    }

    pub(crate) fn threshold(&self) -> f64 {
        *lock(&self.threshold)
    }

    pub(crate) fn wait(&self, active: bool, timeout: Option<Duration>) -> bool {
        let ev = if active {
            &self.activated
        } else {
            &self.deactivated
        };
        self.clock.wait(ev, timeout)
    }

    pub(crate) fn on_activated(&self, f: Option<Callback>) {
        *lock(&self.when_activated) = f;
    }

    pub(crate) fn on_deactivated(&self, f: Option<Callback>) {
        *lock(&self.when_deactivated) = f;
    }

    /// Starts the sampling task.
    pub(crate) fn start(
        self: &Arc<Self>,
        sample_wait: Duration,
        mut sample: impl FnMut(&Event) -> Option<f64> + Send + 'static,
    ) -> Task {
        let sh = self.clone();
        let clock = self.clock.clone();
        Task::spawn(&self.clock, "gpio-zero-queue", move |stop| {
            while !stop.is_set() {
                if let Some(v) = sample(&stop) {
                    if stop.is_set() {
                        break;
                    }
                    sh.push(v);
                }
                if clock.wait(&stop, Some(sample_wait)) {
                    break;
                }
            }
        })
    }

    pub(crate) fn values(self: &Arc<Self>) -> Values<f64> {
        let weak = Arc::downgrade(self);
        Values::from_fn(move || weak.upgrade().map(|s| s.value()))
    }
}

/// Options for [`MotionSensor`] and [`LineSensor`].
#[derive(Clone)]
pub struct SmoothedInputOptions {
    /// Pull-up (`Some(true)`), pull-down (`Some(false)`, default) or floating (`None`).
    pub pull_up: Option<bool>,
    /// Required when `pull_up` is `None`.
    pub active_state: Option<bool>,
    /// Number of samples averaged (median).
    pub queue_len: usize,
    /// Samples per second.
    pub sample_rate: f64,
    /// The averaged value must exceed this to be active.
    pub threshold: f64,
    /// Report values before the queue is full.
    pub partial: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl fmt::Debug for SmoothedInputOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SmoothedInputOptions")
            .field("pull_up", &self.pull_up)
            .field("active_state", &self.active_state)
            .field("queue_len", &self.queue_len)
            .field("sample_rate", &self.sample_rate)
            .field("threshold", &self.threshold)
            .field("partial", &self.partial)
            .finish_non_exhaustive()
    }
}

impl SmoothedInputOptions {
    /// Defaults for [`MotionSensor`]: 1 sample at 10 Hz.
    pub fn motion() -> Self {
        SmoothedInputOptions {
            pull_up: Some(false),
            active_state: None,
            queue_len: 1,
            sample_rate: 10.0,
            threshold: 0.5,
            partial: false,
            pin_factory: None,
        }
    }

    /// Defaults for [`LineSensor`]: 5 samples at 100 Hz.
    pub fn line() -> Self {
        SmoothedInputOptions {
            queue_len: 5,
            sample_rate: 100.0,
            ..Self::motion()
        }
    }
}

struct SmoothedDigital {
    _task: Task,
    shared: Arc<SmoothShared>,
    pin: PinHandle,
}

impl SmoothedDigital {
    fn new(pin: u32, o: SmoothedInputOptions) -> Result<Arc<Self>> {
        if o.sample_rate <= 0.0 {
            return invalid("sample_rate must be positive");
        }
        let (pull, active_state) = resolve_pull(o.pull_up, o.active_state)?;
        let factory = resolve(&o.pin_factory)?;
        let handle = PinHandle::new(&factory, pin)?;
        handle.set_function(PinFunction::Input)?;
        handle.set_pull(pull)?;
        let shared = SmoothShared::new(factory.clock(), o.queue_len, o.partial, o.threshold)?;
        let p: Arc<dyn Pin> = handle.pin.clone();
        let task = shared.start(Duration::from_secs_f64(1.0 / o.sample_rate), move |_| {
            p.state()
                .ok()
                .map(|s| f64::from(u8::from((s >= 0.5) == active_state)))
        });
        Ok(Arc::new(SmoothedDigital {
            _task: task,
            shared,
            pin: handle,
        }))
    }
}

/// A PIR motion sensor (e.g. HC-SR501).
#[derive(Clone)]
pub struct MotionSensor {
    inner: Arc<SmoothedDigital>,
}

impl fmt::Debug for MotionSensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MotionSensor")
            .field("motion_detected", &self.motion_detected())
            .finish()
    }
}

/// A line-following sensor (e.g. TCRT5000 module): a dark line reads low.
#[derive(Clone)]
pub struct LineSensor {
    inner: Arc<SmoothedDigital>,
}

impl fmt::Debug for LineSensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LineSensor")
            .field("line_detected", &self.line_detected())
            .finish()
    }
}

macro_rules! smoothed_common {
    () => {
        /// Smoothed value, `0.0..=1.0` (median of the sample queue).
        pub fn value(&self) -> f64 {
            self.inner.shared.value()
        }

        /// `true` when the smoothed value exceeds the threshold.
        pub fn is_active(&self) -> bool {
            self.inner.shared.is_active()
        }

        /// Activation threshold.
        pub fn threshold(&self) -> f64 {
            self.inner.shared.threshold()
        }

        /// Changes the activation threshold.
        pub fn set_threshold(&self, t: f64) {
            self.inner.shared.set_threshold(t);
        }

        /// An endless iterator over the smoothed value.
        pub fn values(&self) -> Values<f64> {
            self.inner.shared.values()
        }

        /// The GPIO number.
        pub fn pin(&self) -> u32 {
            self.inner.pin.number()
        }

        /// Removes the event callbacks.
        pub fn clear_callbacks(&self) {
            self.inner.shared.on_activated(None);
            self.inner.shared.on_deactivated(None);
        }
    };
}

impl MotionSensor {
    /// Creates a motion sensor with gpiozero's defaults.
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, SmoothedInputOptions::motion())
    }

    /// Creates a motion sensor with explicit options.
    pub fn with_options(pin: u32, options: SmoothedInputOptions) -> Result<Self> {
        Ok(MotionSensor {
            inner: SmoothedDigital::new(pin, options)?,
        })
    }

    smoothed_common!();

    /// `true` while motion is detected.
    pub fn motion_detected(&self) -> bool {
        self.is_active()
    }

    /// Runs `f` when motion starts.
    pub fn when_motion(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_activated(Some(Arc::new(f)));
    }

    /// Runs `f` when motion stops.
    pub fn when_no_motion(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_deactivated(Some(Arc::new(f)));
    }

    /// Blocks until motion is detected (or `timeout`).
    pub fn wait_for_motion(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(true, timeout)
    }

    /// Blocks until motion stops (or `timeout`).
    pub fn wait_for_no_motion(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(false, timeout)
    }
}

impl LineSensor {
    /// Creates a line sensor with gpiozero's defaults.
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, SmoothedInputOptions::line())
    }

    /// Creates a line sensor with explicit options.
    pub fn with_options(pin: u32, options: SmoothedInputOptions) -> Result<Self> {
        Ok(LineSensor {
            inner: SmoothedDigital::new(pin, options)?,
        })
    }

    smoothed_common!();

    /// `true` when a (dark) line is under the sensor, i.e. the input is inactive.
    pub fn line_detected(&self) -> bool {
        !self.is_active()
    }

    /// Runs `f` when a line is detected.
    pub fn when_line(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_deactivated(Some(Arc::new(f)));
    }

    /// Runs `f` when the line is lost.
    pub fn when_no_line(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_activated(Some(Arc::new(f)));
    }

    /// Blocks until a line is detected (or `timeout`).
    pub fn wait_for_line(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(false, timeout)
    }

    /// Blocks until no line is detected (or `timeout`).
    pub fn wait_for_no_line(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(true, timeout)
    }
}

/// Options for [`LightSensor`].
#[derive(Clone)]
pub struct LightSensorOptions {
    /// Samples averaged (default 5).
    pub queue_len: usize,
    /// Charge time treated as complete darkness (default 10 ms).
    pub charge_time_limit: Duration,
    /// Light level above which the sensor is active (default 0.1).
    pub threshold: f64,
    /// Report values before the queue is full (default `false`).
    pub partial: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for LightSensorOptions {
    fn default() -> Self {
        LightSensorOptions {
            queue_len: 5,
            charge_time_limit: Duration::from_millis(10),
            threshold: 0.1,
            partial: false,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for LightSensorOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LightSensorOptions")
            .field("queue_len", &self.queue_len)
            .field("charge_time_limit", &self.charge_time_limit)
            .field("threshold", &self.threshold)
            .field("partial", &self.partial)
            .finish_non_exhaustive()
    }
}

struct LightInner {
    task: Option<Task>,
    charged: Event,
    shared: Arc<SmoothShared>,
    pin: PinHandle,
}

impl Drop for LightInner {
    fn drop(&mut self) {
        let mut task = self.task.take();
        if let Some(t) = &mut task {
            t.signal();
        }
        self.charged.set();
        drop(task);
    }
}

/// A light-dependent resistor in series with a capacitor, measured by
/// timing how long the capacitor takes to charge.
#[derive(Clone)]
pub struct LightSensor {
    inner: Arc<LightInner>,
}

impl fmt::Debug for LightSensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LightSensor")
            .field("value", &self.value())
            .finish()
    }
}

impl LightSensor {
    /// Creates a light sensor with gpiozero's defaults.
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, LightSensorOptions::default())
    }

    /// Creates a light sensor with explicit options.
    pub fn with_options(pin: u32, o: LightSensorOptions) -> Result<Self> {
        if o.charge_time_limit.is_zero() {
            return invalid("charge_time_limit must be positive");
        }
        let factory = resolve(&o.pin_factory)?;
        let handle = PinHandle::new(&factory, pin)?;
        handle.set_function(PinFunction::Output)?;
        handle.set_state(0.0)?;
        handle.set_edges(Edges::Rising)?;
        let charged = Event::new();
        let charged2 = charged.clone();
        let rise: Arc<Mutex<Option<Duration>>> = Arc::default();
        {
            let charged = charged.clone();
            let rise = rise.clone();
            handle.set_when_changed(Some(Arc::new(move |ticks, level| {
                if level {
                    *lock(&rise) = Some(ticks);
                    charged.set();
                }
            })))?;
        }
        let shared = SmoothShared::new(factory.clock(), o.queue_len, o.partial, o.threshold)?;
        let p: Arc<dyn Pin> = handle.pin.clone();
        let clock = factory.clock();
        let limit = o.charge_time_limit;
        let task = shared.start(Duration::ZERO, move |stop| {
            charged.clear();
            *lock(&rise) = None;
            p.set_function(PinFunction::Output).ok()?;
            p.set_state(0.0).ok()?;
            if clock.wait(stop, Some(Duration::from_millis(100))) {
                return None;
            }
            let start = clock.now();
            p.set_function(PinFunction::Input).ok()?;
            let charge = if clock.wait(&charged, Some(limit)) {
                lock(&rise).unwrap_or(start).saturating_sub(start)
            } else {
                limit
            };
            Some(1.0 - charge.min(limit).as_secs_f64() / limit.as_secs_f64())
        });
        Ok(LightSensor {
            inner: Arc::new(LightInner {
                task: Some(task),
                charged: charged2,
                shared,
                pin: handle,
            }),
        })
    }

    smoothed_common!();

    /// `true` when the light level is above the threshold.
    pub fn light_detected(&self) -> bool {
        self.is_active()
    }

    /// Runs `f` when light is detected.
    pub fn when_light(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_activated(Some(Arc::new(f)));
    }

    /// Runs `f` when it gets dark.
    pub fn when_dark(&self, f: impl Fn() + Send + Sync + 'static) {
        self.inner.shared.on_deactivated(Some(Arc::new(f)));
    }

    /// Blocks until light is detected (or `timeout`).
    pub fn wait_for_light(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(true, timeout)
    }

    /// Blocks until it is dark (or `timeout`).
    pub fn wait_for_dark(&self, timeout: Option<Duration>) -> bool {
        self.inner.shared.wait(false, timeout)
    }
}
