//! Building blocks shared by all devices: the [`Values`] iterator, `source`
//! loops and blink/pulse animations.

use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::clock::{Clock, Event, Task, lock};
use crate::pins::Pin;

/// Callback type for device events (`when_pressed`, `when_motion`, ...).
pub type Callback = Arc<dyn Fn() + Send + Sync>;

/// Default delay between iterations of a `source` loop (gpiozero's
/// `source_delay`).
pub const DEFAULT_SOURCE_DELAY: Duration = Duration::from_millis(10);

/// An endless iterator over a device's current value (gpiozero's `values`).
///
/// Each call to `next` reads the device again; the iterator ends only when
/// the device is dropped (or a read fails). Feed it to another device's
/// `set_source` to link the two, or adapt it with [`tools`](crate::tools)
/// and ordinary iterator adapters.
pub struct Values<T> {
    read: Box<dyn FnMut() -> Option<T> + Send>,
}

impl<T> Values<T> {
    /// Wraps a reader closure. Returning `None` ends the iterator.
    pub fn from_fn(read: impl FnMut() -> Option<T> + Send + 'static) -> Self {
        Values {
            read: Box::new(read),
        }
    }
}

impl<T> Iterator for Values<T> {
    type Item = T;
    fn next(&mut self) -> Option<T> {
        (self.read)()
    }
}

impl<T> fmt::Debug for Values<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Values")
    }
}

/// The background loop that copies values from a `source` iterator into a
/// device.
#[derive(Default)]
pub(crate) struct SourceSlot {
    task: Mutex<Option<Task>>,
    delay: Arc<Mutex<Option<Duration>>>,
}

impl SourceSlot {
    pub(crate) fn delay(&self) -> Duration {
        lock(&self.delay).unwrap_or(DEFAULT_SOURCE_DELAY)
    }

    pub(crate) fn set_delay(&self, d: Duration) {
        *lock(&self.delay) = Some(d);
    }

    /// Starts copying `values` into the device with `apply`. `apply`
    /// returns `false` when the device is gone.
    pub(crate) fn start<I, T, F>(&self, clock: &Arc<dyn Clock>, values: I, apply: F)
    where
        I: IntoIterator<Item = T>,
        I::IntoIter: Send + 'static,
        F: Fn(T) -> bool + Send + 'static,
    {
        self.stop();
        let delay = self.delay.clone();
        let c = clock.clone();
        let mut iter = values.into_iter();
        let task = Task::spawn(clock, "gpio-zero-source", move |stop| {
            loop {
                if stop.is_set() {
                    break;
                }
                let Some(v) = iter.next() else { break };
                if stop.is_set() || !apply(v) {
                    break;
                }
                let d = lock(&delay).unwrap_or(DEFAULT_SOURCE_DELAY);
                if c.wait(&stop, Some(d)) {
                    break;
                }
            }
        });
        *lock(&self.task) = Some(task);
    }

    pub(crate) fn stop(&self) {
        let t = lock(&self.task).take();
        drop(t);
    }

    pub(crate) fn is_running(&self) -> bool {
        lock(&self.task).as_ref().is_some_and(|t| !t.is_finished())
    }
}

/// Timing of a blink or pulse animation.
///
/// ```
/// use gpio_zero::Blink;
/// use std::time::Duration;
/// let b = Blink::new(Duration::from_millis(500), Duration::from_millis(500))
///     .fade(Duration::from_millis(200), Duration::from_millis(200))
///     .times(3);
/// assert_eq!(b.n, Some(3));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blink {
    /// Time spent fully on.
    pub on_time: Duration,
    /// Time spent fully off.
    pub off_time: Duration,
    /// Fade-in time (PWM devices only).
    pub fade_in: Duration,
    /// Fade-out time (PWM devices only).
    pub fade_out: Duration,
    /// Number of cycles (`None` = forever).
    pub n: Option<u32>,
    /// Run in a background thread (`true`) or block until done (`false`).
    pub background: bool,
}

impl Default for Blink {
    fn default() -> Self {
        Blink {
            on_time: Duration::from_secs(1),
            off_time: Duration::from_secs(1),
            fade_in: Duration::ZERO,
            fade_out: Duration::ZERO,
            n: None,
            background: true,
        }
    }
}

impl Blink {
    /// A background blink, forever, without fades.
    pub fn new(on_time: Duration, off_time: Duration) -> Self {
        Blink {
            on_time,
            off_time,
            ..Default::default()
        }
    }

    /// A pulse: fade in then out, no steady on/off time.
    pub fn pulse(fade_in: Duration, fade_out: Duration) -> Self {
        Blink {
            on_time: Duration::ZERO,
            off_time: Duration::ZERO,
            fade_in,
            fade_out,
            ..Default::default()
        }
    }

    /// Sets the fade times.
    pub fn fade(mut self, fade_in: Duration, fade_out: Duration) -> Self {
        self.fade_in = fade_in;
        self.fade_out = fade_out;
        self
    }

    /// Repeats `n` times then stops (off).
    pub fn times(mut self, n: u32) -> Self {
        self.n = Some(n);
        self
    }

    /// Blocks the caller until finished instead of running in the background.
    pub fn foreground(mut self) -> Self {
        self.background = false;
        self
    }

    pub(crate) fn has_fade(&self) -> bool {
        !self.fade_in.is_zero() || !self.fade_out.is_zero()
    }
}

/// Animation frame rate for fades (gpiozero uses 25).
pub(crate) const FPS: u32 = 25;

pub(crate) type Frame = (Vec<f64>, Duration);

fn lerp(a: &[f64], b: &[f64], t: f64) -> Vec<f64> {
    a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect()
}

/// The frame sequence gpiozero uses for `blink`/`pulse`.
pub(crate) fn blink_frames(spec: &Blink, on: &[f64], off: &[f64]) -> Vec<Frame> {
    let step = Duration::from_secs_f64(1.0 / f64::from(FPS));
    let mut frames = Vec::new();
    let n_in = (spec.fade_in.as_secs_f64() * f64::from(FPS)) as usize;
    for i in 0..n_in {
        frames.push((lerp(off, on, i as f64 / n_in as f64), step));
    }
    frames.push((on.to_vec(), spec.on_time));
    let n_out = (spec.fade_out.as_secs_f64() * f64::from(FPS)) as usize;
    for i in 0..n_out {
        frames.push((lerp(on, off, i as f64 / n_out as f64), step));
    }
    frames.push((off.to_vec(), spec.off_time));
    frames
}

/// Writes logical values to a pin, honouring `active_high`.
#[derive(Clone)]
pub(crate) struct PinWriter {
    pub(crate) pin: Arc<dyn Pin>,
    pub(crate) active_high: bool,
}

impl PinWriter {
    pub(crate) fn write(&self, v: f64) -> crate::Result<()> {
        self.pin
            .set_state(if self.active_high { v } else { 1.0 - v })
    }
}

fn run_frames(
    clock: &dyn Clock,
    stop: &Event,
    frames: &[Frame],
    n: Option<u32>,
    writers: &[PinWriter],
) {
    let mut count = 0;
    while n.is_none_or(|n| count < n) {
        for (vals, delay) in frames {
            for (w, v) in writers.iter().zip(vals) {
                let _ = w.write(*v);
            }
            if clock.wait(stop, Some(*delay)) {
                return;
            }
        }
        count += 1;
    }
}

/// Starts (or runs in the foreground) an animation, replacing whatever was
/// in `slot`.
pub(crate) fn animate(
    slot: &Mutex<Option<Task>>,
    clock: &Arc<dyn Clock>,
    writers: Vec<PinWriter>,
    frames: Vec<Frame>,
    n: Option<u32>,
    background: bool,
) {
    let old = lock(slot).take();
    drop(old);
    if background {
        let c = clock.clone();
        let task = Task::spawn(clock, "gpio-zero-blink", move |stop| {
            run_frames(&*c, &stop, &frames, n, &writers);
        });
        *lock(slot) = Some(task);
    } else {
        run_frames(&**clock, &Event::new(), &frames, n, &writers);
    }
}

pub(crate) fn stop_task(slot: &Mutex<Option<Task>>) {
    let old = lock(slot).take();
    drop(old);
}

pub(crate) fn check_unit(v: f64, what: &str) -> crate::Result<f64> {
    if (0.0..=1.0).contains(&v) {
        Ok(v)
    } else {
        crate::error::invalid(format!("{what} must be between 0 and 1, got {v}"))
    }
}

pub(crate) fn check_signed(v: f64, what: &str) -> crate::Result<f64> {
    if (-1.0..=1.0).contains(&v) {
        Ok(v)
    } else {
        crate::error::invalid(format!("{what} must be between -1 and 1, got {v}"))
    }
}
