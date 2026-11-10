use std::fmt;
use std::sync::Arc;
use std::time::Duration;

use super::OutputInner;
use crate::clock::Clock;
use crate::devices::{SourceSlot, Values, check_signed, check_unit};
use crate::error::{Result, invalid};
use crate::pins::{PinFactory, resolve};

/// Options for [`Motor`].
#[derive(Clone)]
pub struct MotorOptions {
    /// Optional enable pin, driven high while the motor exists.
    pub enable: Option<u32>,
    /// Use PWM for speed control (default `true`). Without PWM only full
    /// speed (`1.0`) or stop is possible.
    pub pwm: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for MotorOptions {
    fn default() -> Self {
        MotorOptions {
            enable: None,
            pwm: true,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for MotorOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MotorOptions")
            .field("enable", &self.enable)
            .field("pwm", &self.pwm)
            .finish_non_exhaustive()
    }
}

pub(crate) struct MotorInner {
    forward: Arc<OutputInner>,
    backward: Arc<OutputInner>,
    _enable: Option<Arc<OutputInner>>,
    pwm: bool,
    clock: Arc<dyn Clock>,
    source: SourceSlot,
}

impl MotorInner {
    fn check_speed(&self, speed: f64) -> Result<f64> {
        let s = check_unit(speed, "speed")?;
        if !self.pwm && s != 0.0 && s != 1.0 {
            return invalid("a non-PWM motor only accepts speed 0 or 1");
        }
        Ok(s)
    }

    pub(crate) fn set(&self, value: f64) -> Result<()> {
        let v = check_signed(value, "motor value")?;
        let speed = self.check_speed(v.abs())?;
        if v >= 0.0 {
            self.backward.set(0.0)?;
            self.forward.set(speed)
        } else {
            self.forward.set(0.0)?;
            self.backward.set(speed)
        }
    }

    pub(crate) fn read(&self) -> f64 {
        self.forward.read() - self.backward.read()
    }
}

impl Drop for MotorInner {
    fn drop(&mut self) {
        self.source.stop();
    }
}

/// A DC motor driven through an H-bridge with forward and backward pins
/// (e.g. L298N, DRV8833).
#[derive(Clone)]
pub struct Motor {
    pub(crate) inner: Arc<MotorInner>,
}

impl fmt::Debug for Motor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Motor")
            .field("value", &self.value())
            .finish()
    }
}

impl Motor {
    /// Creates a motor on the given forward/backward pins.
    pub fn new(forward: u32, backward: u32) -> Result<Self> {
        Self::with_options(forward, backward, MotorOptions::default())
    }

    /// Creates a motor with explicit options.
    pub fn with_options(forward: u32, backward: u32, options: MotorOptions) -> Result<Self> {
        let factory = resolve(&options.pin_factory)?;
        let freq = options.pwm.then_some(100.0);
        let f = OutputInner::new(&factory, forward, true, freq, Some(0.0))?;
        let b = OutputInner::new(&factory, backward, true, freq, Some(0.0))?;
        let enable = options
            .enable
            .map(|n| OutputInner::new(&factory, n, true, None, Some(1.0)))
            .transpose()?;
        Ok(Motor {
            inner: Arc::new(MotorInner {
                forward: f,
                backward: b,
                _enable: enable,
                pwm: options.pwm,
                clock: factory.clock(),
                source: SourceSlot::default(),
            }),
        })
    }

    /// Drives forward at `speed` (`0.0..=1.0`).
    pub fn forward(&self, speed: f64) -> Result<()> {
        self.inner.set(check_unit(speed, "speed")?)
    }

    /// Drives backward at `speed` (`0.0..=1.0`).
    pub fn backward(&self, speed: f64) -> Result<()> {
        self.inner.set(-check_unit(speed, "speed")?)
    }

    /// Reverses direction, keeping the speed.
    pub fn reverse(&self) -> Result<()> {
        self.inner.set(-self.value())
    }

    /// Stops the motor.
    pub fn stop(&self) -> Result<()> {
        self.inner.set(0.0)
    }

    /// Current velocity: `-1.0` (full backward) to `1.0` (full forward).
    pub fn value(&self) -> f64 {
        self.inner.read()
    }

    /// Sets the velocity, `-1.0..=1.0`.
    pub fn set_value(&self, value: f64) -> Result<()> {
        self.inner.set(value)
    }

    /// `true` if the motor is moving.
    pub fn is_active(&self) -> bool {
        self.value() != 0.0
    }

    /// An endless iterator over the velocity.
    pub fn values(&self) -> Values<f64> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || weak.upgrade().map(|i| i.read()))
    }

    /// Continuously sets the velocity from `source`.
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator,
        I::Item: Into<f64> + 'static,
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

    /// Sets the delay between `source` iterations.
    pub fn set_source_delay(&self, delay: Duration) {
        self.inner.source.set_delay(delay);
    }
}

/// Options for [`PhaseEnableMotor`].
#[derive(Clone)]
pub struct PhaseEnableMotorOptions {
    /// Use PWM on the enable pin (default `true`).
    pub pwm: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for PhaseEnableMotorOptions {
    fn default() -> Self {
        PhaseEnableMotorOptions {
            pwm: true,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for PhaseEnableMotorOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhaseEnableMotorOptions")
            .field("pwm", &self.pwm)
            .finish_non_exhaustive()
    }
}

struct PeInner {
    phase: Arc<OutputInner>,
    enable: Arc<OutputInner>,
    pwm: bool,
    clock: Arc<dyn Clock>,
    source: SourceSlot,
}

impl PeInner {
    fn set(&self, value: f64) -> Result<()> {
        let v = check_signed(value, "motor value")?;
        let speed = v.abs();
        if !self.pwm && speed != 0.0 && speed != 1.0 {
            return invalid("a non-PWM motor only accepts speed 0 or 1");
        }
        self.phase.set(if v < 0.0 { 1.0 } else { 0.0 })?;
        self.enable.set(speed)
    }

    fn read(&self) -> f64 {
        let speed = self.enable.read();
        if self.phase.read() >= 0.5 {
            -speed
        } else {
            speed
        }
    }
}

impl Drop for PeInner {
    fn drop(&mut self) {
        self.source.stop();
    }
}

/// A motor driven by a phase/enable controller (e.g. DRV8835 in PH/EN mode,
/// the Pololu boards): one pin sets direction, the other speed.
#[derive(Clone)]
pub struct PhaseEnableMotor {
    inner: Arc<PeInner>,
}

impl fmt::Debug for PhaseEnableMotor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PhaseEnableMotor")
            .field("value", &self.value())
            .finish()
    }
}

impl PhaseEnableMotor {
    /// Creates the motor on the given phase and enable pins.
    pub fn new(phase: u32, enable: u32) -> Result<Self> {
        Self::with_options(phase, enable, PhaseEnableMotorOptions::default())
    }

    /// Creates the motor with explicit options.
    pub fn with_options(phase: u32, enable: u32, options: PhaseEnableMotorOptions) -> Result<Self> {
        let factory = resolve(&options.pin_factory)?;
        let p = OutputInner::new(&factory, phase, true, None, Some(0.0))?;
        let e = OutputInner::new(
            &factory,
            enable,
            true,
            options.pwm.then_some(100.0),
            Some(0.0),
        )?;
        Ok(PhaseEnableMotor {
            inner: Arc::new(PeInner {
                phase: p,
                enable: e,
                pwm: options.pwm,
                clock: factory.clock(),
                source: SourceSlot::default(),
            }),
        })
    }

    /// Drives forward at `speed`.
    pub fn forward(&self, speed: f64) -> Result<()> {
        self.inner.set(check_unit(speed, "speed")?)
    }

    /// Drives backward at `speed`.
    pub fn backward(&self, speed: f64) -> Result<()> {
        self.inner.set(-check_unit(speed, "speed")?)
    }

    /// Reverses direction.
    pub fn reverse(&self) -> Result<()> {
        self.inner.set(-self.value())
    }

    /// Stops.
    pub fn stop(&self) -> Result<()> {
        self.inner.set(0.0)
    }

    /// Velocity, `-1.0..=1.0`.
    pub fn value(&self) -> f64 {
        self.inner.read()
    }

    /// Sets the velocity.
    pub fn set_value(&self, value: f64) -> Result<()> {
        self.inner.set(value)
    }

    /// `true` if moving.
    pub fn is_active(&self) -> bool {
        self.value() != 0.0
    }

    /// An endless iterator over the velocity.
    pub fn values(&self) -> Values<f64> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || weak.upgrade().map(|i| i.read()))
    }

    /// Continuously sets the velocity from `source`.
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator,
        I::Item: Into<f64> + 'static,
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
}
