use std::fmt;
use std::sync::Arc;
use std::time::Duration;

use super::OutputInner;
use crate::clock::Clock;
use crate::devices::{SourceSlot, Values, check_signed};
use crate::error::{Result, invalid};
use crate::pins::{PinFactory, resolve};

/// Options for [`Servo`].
#[derive(Clone)]
pub struct ServoOptions {
    /// Initial position, `-1.0..=1.0`, or `None` for detached (default `Some(0.0)`).
    pub initial_value: Option<f64>,
    /// Pulse width at the minimum position (default 1 ms).
    pub min_pulse_width: Duration,
    /// Pulse width at the maximum position (default 2 ms).
    pub max_pulse_width: Duration,
    /// Time between pulses (default 20 ms = 50 Hz).
    pub frame_width: Duration,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for ServoOptions {
    fn default() -> Self {
        ServoOptions {
            initial_value: Some(0.0),
            min_pulse_width: Duration::from_micros(1000),
            max_pulse_width: Duration::from_micros(2000),
            frame_width: Duration::from_millis(20),
            pin_factory: None,
        }
    }
}

impl fmt::Debug for ServoOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ServoOptions")
            .field("initial_value", &self.initial_value)
            .field("min_pulse_width", &self.min_pulse_width)
            .field("max_pulse_width", &self.max_pulse_width)
            .field("frame_width", &self.frame_width)
            .finish_non_exhaustive()
    }
}

struct ServoInner {
    out: Arc<OutputInner>,
    min_pw: f64,
    max_pw: f64,
    frame: f64,
    clock: Arc<dyn Clock>,
    source: SourceSlot,
}

impl ServoInner {
    fn set(&self, value: Option<f64>) -> Result<()> {
        match value {
            None => self.out.set(0.0),
            Some(v) => {
                let v = check_signed(v, "servo value")?;
                let pw = self.min_pw + (v + 1.0) / 2.0 * (self.max_pw - self.min_pw);
                self.out.set(pw / self.frame)
            }
        }
    }

    fn read(&self) -> Option<f64> {
        let duty = self.out.read();
        if duty == 0.0 {
            return None;
        }
        let pw = duty * self.frame;
        let v = (pw - self.min_pw) / (self.max_pw - self.min_pw) * 2.0 - 1.0;
        Some((v * 1e9).round() / 1e9)
    }
}

impl Drop for ServoInner {
    fn drop(&mut self) {
        self.source.stop();
    }
}

/// A PWM-controlled hobby servo.
///
/// The value runs from `-1.0` (minimum position) to `1.0` (maximum);
/// `None` stops sending pulses ("detached").
#[derive(Clone)]
pub struct Servo {
    inner: Arc<ServoInner>,
}

impl fmt::Debug for Servo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Servo")
            .field("value", &self.value())
            .finish()
    }
}

impl Servo {
    /// Creates a servo on GPIO `pin` with default options.
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, ServoOptions::default())
    }

    /// Creates a servo with explicit options.
    pub fn with_options(pin: u32, options: ServoOptions) -> Result<Self> {
        if options.min_pulse_width >= options.max_pulse_width {
            return invalid("min_pulse_width must be less than max_pulse_width");
        }
        if options.max_pulse_width >= options.frame_width {
            return invalid("max_pulse_width must be less than frame_width");
        }
        let factory = resolve(&options.pin_factory)?;
        let frame = options.frame_width.as_secs_f64();
        let out = OutputInner::new(&factory, pin, true, Some(1.0 / frame), Some(0.0))?;
        let inner = ServoInner {
            out,
            min_pw: options.min_pulse_width.as_secs_f64(),
            max_pw: options.max_pulse_width.as_secs_f64(),
            frame,
            clock: factory.clock(),
            source: SourceSlot::default(),
        };
        inner.set(options.initial_value)?;
        Ok(Servo {
            inner: Arc::new(inner),
        })
    }

    /// Moves to the minimum position.
    pub fn min(&self) -> Result<()> {
        self.inner.set(Some(-1.0))
    }

    /// Moves to the middle position.
    pub fn mid(&self) -> Result<()> {
        self.inner.set(Some(0.0))
    }

    /// Moves to the maximum position.
    pub fn max(&self) -> Result<()> {
        self.inner.set(Some(1.0))
    }

    /// Stops sending pulses.
    pub fn detach(&self) -> Result<()> {
        self.inner.set(None)
    }

    /// Current position, or `None` when detached.
    pub fn value(&self) -> Option<f64> {
        self.inner.read()
    }

    /// Sets the position (`-1.0..=1.0`) or detaches with `None`.
    pub fn set_value(&self, value: impl Into<Option<f64>>) -> Result<()> {
        self.inner.set(value.into())
    }

    /// `true` unless detached.
    pub fn is_active(&self) -> bool {
        self.value().is_some()
    }

    /// The current pulse width, or `None` when detached.
    pub fn pulse_width(&self) -> Option<Duration> {
        let d = self.inner.out.read();
        (d > 0.0).then(|| Duration::from_secs_f64(d * self.inner.frame))
    }

    /// Frame width (time between pulses).
    pub fn frame_width(&self) -> Duration {
        Duration::from_secs_f64(self.inner.frame)
    }

    /// Minimum pulse width.
    pub fn min_pulse_width(&self) -> Duration {
        Duration::from_secs_f64(self.inner.min_pw)
    }

    /// Maximum pulse width.
    pub fn max_pulse_width(&self) -> Duration {
        Duration::from_secs_f64(self.inner.max_pw)
    }

    /// An endless iterator over the position.
    pub fn values(&self) -> Values<Option<f64>> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || weak.upgrade().map(|i| i.read()))
    }

    /// Continuously sets the position from `source` (items convertible to
    /// `Option<f64>`, so plain `f64` works).
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator,
        I::Item: Into<Option<f64>> + 'static,
        I::IntoIter: Send + 'static,
    {
        let weak = Arc::downgrade(&self.inner);
        self.inner
            .source
            .start(&self.inner.clock, source, move |v| match weak.upgrade() {
                Some(i) => {
                    let _ = i.set(v.into());
                    true
                }
                None => false,
            });
    }

    /// Stops the `source` loop.
    pub fn clear_source(&self) {
        self.inner.source.stop();
    }

    /// The GPIO number.
    pub fn pin(&self) -> u32 {
        self.inner.out.pin.number()
    }
}

/// Options for [`AngularServo`].
#[derive(Clone, Debug)]
pub struct AngularServoOptions {
    /// Initial angle, or `None` for detached (default `Some(0.0)`).
    pub initial_angle: Option<f64>,
    /// Angle at the minimum position (default `-90.0`).
    pub min_angle: f64,
    /// Angle at the maximum position (default `90.0`).
    pub max_angle: f64,
    /// Pulse widths, frame width and factory (its `initial_value` is ignored).
    pub servo: ServoOptions,
}

impl Default for AngularServoOptions {
    fn default() -> Self {
        AngularServoOptions {
            initial_angle: Some(0.0),
            min_angle: -90.0,
            max_angle: 90.0,
            servo: ServoOptions::default(),
        }
    }
}

/// A servo positioned by angle.
#[derive(Clone, Debug)]
pub struct AngularServo {
    servo: Servo,
    min_angle: f64,
    max_angle: f64,
}

impl AngularServo {
    /// Creates an angular servo with default options (-90°..90°).
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, AngularServoOptions::default())
    }

    /// Creates an angular servo with explicit options.
    pub fn with_options(pin: u32, options: AngularServoOptions) -> Result<Self> {
        if options.min_angle == options.max_angle {
            return invalid("min_angle and max_angle must differ");
        }
        let mut so = options.servo.clone();
        so.initial_value = None;
        let s = AngularServo {
            servo: Servo::with_options(pin, so)?,
            min_angle: options.min_angle,
            max_angle: options.max_angle,
        };
        s.set_angle(options.initial_angle)?;
        Ok(s)
    }

    fn to_value(&self, angle: f64) -> Result<f64> {
        let (lo, hi) = if self.min_angle < self.max_angle {
            (self.min_angle, self.max_angle)
        } else {
            (self.max_angle, self.min_angle)
        };
        if !(lo..=hi).contains(&angle) {
            return invalid(format!("angle {angle} is outside {lo}..={hi}"));
        }
        Ok((angle - self.min_angle) / (self.max_angle - self.min_angle) * 2.0 - 1.0)
    }

    /// Current angle, or `None` when detached.
    pub fn angle(&self) -> Option<f64> {
        self.servo.value().map(|v| {
            let a = self.min_angle + (v + 1.0) / 2.0 * (self.max_angle - self.min_angle);
            (a * 1e6).round() / 1e6
        })
    }

    /// Moves to `angle` (or detaches with `None`).
    pub fn set_angle(&self, angle: impl Into<Option<f64>>) -> Result<()> {
        match angle.into() {
            None => self.servo.detach(),
            Some(a) => self.servo.set_value(self.to_value(a)?),
        }
    }

    /// Minimum angle.
    pub fn min_angle(&self) -> f64 {
        self.min_angle
    }

    /// Maximum angle.
    pub fn max_angle(&self) -> f64 {
        self.max_angle
    }

    /// The underlying [`Servo`] (value-based API, `min`/`mid`/`max`, `source`).
    pub fn servo(&self) -> &Servo {
        &self.servo
    }

    /// An endless iterator over the angle.
    pub fn angles(&self) -> Values<Option<f64>> {
        let weak = Arc::downgrade(&self.servo.inner);
        let (lo, hi) = (self.min_angle, self.max_angle);
        Values::from_fn(move || {
            weak.upgrade()
                .map(|i| i.read().map(|v| lo + (v + 1.0) / 2.0 * (hi - lo)))
        })
    }
}

impl std::ops::Deref for AngularServo {
    type Target = Servo;
    fn deref(&self) -> &Servo {
        &self.servo
    }
}
