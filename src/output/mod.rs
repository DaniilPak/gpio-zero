//! Output devices: LEDs, buzzers, motors, servos and generic outputs.

use std::fmt;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use crate::clock::{Clock, Task};
use crate::devices::{
    Blink, PinWriter, SourceSlot, Values, animate, blink_frames, check_unit, stop_task,
};
use crate::error::{Result, invalid};
use crate::pins::{PinFactory, PinFunction, PinHandle, resolve};

mod motor;
mod rgb;
mod servo;
mod tonal;

pub use motor::{Motor, MotorOptions, PhaseEnableMotor, PhaseEnableMotorOptions};
pub use rgb::{Color, RgbLed, RgbLedOptions};
pub use servo::{AngularServo, AngularServoOptions, Servo, ServoOptions};
pub use tonal::{TonalBuzzer, TonalBuzzerOptions, Tone};

/// Shared state of a single-pin output.
pub(crate) struct OutputInner {
    pub(crate) pin: PinHandle,
    pub(crate) active_high: bool,
    pub(crate) pwm: bool,
    pub(crate) clock: Arc<dyn Clock>,
    pub(crate) blink: Mutex<Option<Task>>,
    pub(crate) source: SourceSlot,
}

impl OutputInner {
    pub(crate) fn new(
        factory: &Arc<dyn PinFactory>,
        pin: u32,
        active_high: bool,
        pwm_frequency: Option<f64>,
        initial: Option<f64>,
    ) -> Result<Arc<Self>> {
        let handle = PinHandle::new(factory, pin)?;
        handle.set_function(PinFunction::Output)?;
        if let Some(f) = pwm_frequency {
            handle.set_frequency(Some(f))?;
        }
        let inner = OutputInner {
            pin: handle,
            active_high,
            pwm: pwm_frequency.is_some(),
            clock: factory.clock(),
            blink: Mutex::new(None),
            source: SourceSlot::default(),
        };
        if let Some(v) = initial {
            inner.write(v)?;
        }
        Ok(Arc::new(inner))
    }

    pub(crate) fn writer(&self) -> PinWriter {
        PinWriter {
            pin: self.pin.pin.clone(),
            active_high: self.active_high,
        }
    }

    pub(crate) fn write(&self, v: f64) -> Result<()> {
        self.writer().write(v)
    }

    /// Logical value (after `active_high` inversion).
    pub(crate) fn read(&self) -> f64 {
        let s = self.pin.state().unwrap_or(0.0);
        if self.active_high { s } else { 1.0 - s }
    }

    /// Stops any blink and writes `v`.
    pub(crate) fn set(&self, v: f64) -> Result<()> {
        stop_task(&self.blink);
        self.write(v)
    }

    pub(crate) fn blink(&self, spec: Blink) -> Result<()> {
        if spec.has_fade() && !self.pwm {
            return invalid("fade_in/fade_out require a PWM device");
        }
        let frames = blink_frames(&spec, &[1.0], &[0.0]);
        animate(
            &self.blink,
            &self.clock,
            vec![self.writer()],
            frames,
            spec.n,
            spec.background,
        );
        Ok(())
    }

    pub(crate) fn start_source<I, T>(self: &Arc<Self>, values: I, conv: fn(T) -> Result<f64>)
    where
        I: IntoIterator<Item = T>,
        I::IntoIter: Send + 'static,
        T: 'static,
    {
        let weak: Weak<Self> = Arc::downgrade(self);
        self.source
            .start(&self.clock, values, move |v| match weak.upgrade() {
                Some(inner) => {
                    if let Ok(v) = conv(v) {
                        let _ = inner.set(v);
                    }
                    true
                }
                None => false,
            });
    }

    pub(crate) fn values<T: 'static>(self: &Arc<Self>, conv: fn(f64) -> T) -> Values<T> {
        let weak = Arc::downgrade(self);
        Values::from_fn(move || weak.upgrade().map(|i| conv(i.read())))
    }
}

impl Drop for OutputInner {
    fn drop(&mut self) {
        self.source.stop();
        stop_task(&self.blink);
    }
}

/// Options for digital outputs ([`OutputDevice`], [`DigitalOutputDevice`],
/// [`Led`], [`Buzzer`]).
#[derive(Clone)]
pub struct OutputOptions {
    /// `true` (default): high = on. `false`: low = on.
    pub active_high: bool,
    /// Initial state; `None` leaves the pin as it is. Default `Some(false)`.
    pub initial_value: Option<bool>,
    /// Pin factory to use (default: [`default_factory`](crate::pins::default_factory)).
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for OutputOptions {
    fn default() -> Self {
        OutputOptions {
            active_high: true,
            initial_value: Some(false),
            pin_factory: None,
        }
    }
}

impl fmt::Debug for OutputOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OutputOptions")
            .field("active_high", &self.active_high)
            .field("initial_value", &self.initial_value)
            .finish_non_exhaustive()
    }
}

fn bool_to_f64(v: bool) -> Result<f64> {
    Ok(f64::from(u8::from(v)))
}

macro_rules! common_output_methods {
    () => {
        /// The GPIO number this device is connected to.
        pub fn pin(&self) -> u32 {
            self.inner.pin.number()
        }

        /// Whether the device is active-high.
        pub fn active_high(&self) -> bool {
            self.inner.active_high
        }

        /// Stops the `source` loop, if any.
        pub fn clear_source(&self) {
            self.inner.source.stop();
        }

        /// Whether a `source` loop is running.
        pub fn has_source(&self) -> bool {
            self.inner.source.is_running()
        }

        /// Delay between `source` iterations (default 10 ms).
        pub fn source_delay(&self) -> Duration {
            self.inner.source.delay()
        }

        /// Sets the delay between `source` iterations.
        pub fn set_source_delay(&self, delay: Duration) {
            self.inner.source.set_delay(delay);
        }

        /// Closes the device, releasing its pin (same as dropping the last handle).
        pub fn close(self) {}
    };
}

macro_rules! digital_output {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone)]
        pub struct $name {
            pub(crate) inner: Arc<OutputInner>,
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.debug_struct(stringify!($name))
                    .field("pin", &self.inner.pin.number())
                    .field("value", &self.value())
                    .finish()
            }
        }

        impl $name {
            /// Creates the device on GPIO `pin` with default options.
            pub fn new(pin: u32) -> Result<Self> {
                Self::with_options(pin, OutputOptions::default())
            }

            /// Creates the device with explicit options.
            pub fn with_options(pin: u32, options: OutputOptions) -> Result<Self> {
                let factory = resolve(&options.pin_factory)?;
                let initial = options.initial_value.map(|v| f64::from(u8::from(v)));
                Ok(Self {
                    inner: OutputInner::new(&factory, pin, options.active_high, None, initial)?,
                })
            }

            /// Turns the device on.
            pub fn on(&self) -> Result<()> {
                self.inner.set(1.0)
            }

            /// Turns the device off.
            pub fn off(&self) -> Result<()> {
                self.inner.set(0.0)
            }

            /// Reverses the state.
            pub fn toggle(&self) -> Result<()> {
                self.set_value(!self.value())
            }

            /// `true` if the device is on.
            pub fn value(&self) -> bool {
                self.inner.read() >= 0.5
            }

            /// Switches the device on or off.
            pub fn set_value(&self, value: bool) -> Result<()> {
                self.inner.set(f64::from(u8::from(value)))
            }

            /// `true` if the device is on.
            pub fn is_active(&self) -> bool {
                self.value()
            }

            /// An endless iterator over the device's state.
            pub fn values(&self) -> Values<bool> {
                self.inner.values(|v| v >= 0.5)
            }

            /// Continuously sets the device from `source` (every
            /// [`source_delay`](Self::source_delay)) in a background thread,
            /// replacing any previous source.
            pub fn set_source<I>(&self, source: I)
            where
                I: IntoIterator,
                I::Item: Into<bool> + 'static,
                I::IntoIter: Send + 'static,
            {
                self.inner
                    .start_source(source.into_iter().map(Into::into), bool_to_f64);
            }

            common_output_methods!();
        }
    };
}

macro_rules! digital_blink {
    ($name:ident) => {
        impl $name {
            /// Blinks: `on_time` on, `off_time` off, `n` times (`None` =
            /// forever). With `background = false` this blocks until done.
            pub fn blink(
                &self,
                on_time: Duration,
                off_time: Duration,
                n: Option<u32>,
                background: bool,
            ) -> Result<()> {
                self.inner.blink(Blink {
                    on_time,
                    off_time,
                    n,
                    background,
                    ..Blink::default()
                })
            }

            /// Blinks with a [`Blink`] spec (fades are not allowed).
            pub fn blink_with(&self, spec: Blink) -> Result<()> {
                self.inner.blink(spec)
            }
        }
    };
}

digital_output!(
    /// A generic on/off output (gpiozero's `OutputDevice`).
    OutputDevice
);
digital_output!(
    /// An on/off output that can also blink (gpiozero's `DigitalOutputDevice`).
    DigitalOutputDevice
);
digital_output!(
    /// A light-emitting diode.
    ///
    /// ```no_run
    /// use gpio_zero::Led;
    /// use std::time::Duration;
    /// let led = Led::new(17)?;
    /// led.on()?;
    /// led.blink(Duration::from_millis(500), Duration::from_millis(500), None, true)?;
    /// # Ok::<(), gpio_zero::Error>(())
    /// ```
    Led
);
digital_output!(
    /// An active (self-oscillating) buzzer.
    Buzzer
);
digital_blink!(DigitalOutputDevice);
digital_blink!(Led);
digital_blink!(Buzzer);

impl Led {
    /// `true` if the LED is lit.
    pub fn is_lit(&self) -> bool {
        self.value()
    }
}

impl Buzzer {
    /// Beeps: like [`Led::blink`].
    pub fn beep(
        &self,
        on_time: Duration,
        off_time: Duration,
        n: Option<u32>,
        background: bool,
    ) -> Result<()> {
        self.blink(on_time, off_time, n, background)
    }
}

/// Options for PWM outputs ([`PwmOutputDevice`], [`PwmLed`]).
#[derive(Clone)]
pub struct PwmOptions {
    /// `true` (default): a higher duty cycle = brighter.
    pub active_high: bool,
    /// Initial value, `0.0..=1.0` (default `0.0`).
    pub initial_value: f64,
    /// PWM frequency in Hz (default 100).
    pub frequency: f64,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for PwmOptions {
    fn default() -> Self {
        PwmOptions {
            active_high: true,
            initial_value: 0.0,
            frequency: 100.0,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for PwmOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PwmOptions")
            .field("active_high", &self.active_high)
            .field("initial_value", &self.initial_value)
            .field("frequency", &self.frequency)
            .finish_non_exhaustive()
    }
}

fn unit(v: f64) -> Result<f64> {
    check_unit(v, "value")
}

macro_rules! pwm_output {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone)]
        pub struct $name {
            pub(crate) inner: Arc<OutputInner>,
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.debug_struct(stringify!($name))
                    .field("pin", &self.inner.pin.number())
                    .field("value", &self.value())
                    .finish()
            }
        }

        impl $name {
            /// Creates the device on GPIO `pin` with default options.
            pub fn new(pin: u32) -> Result<Self> {
                Self::with_options(pin, PwmOptions::default())
            }

            /// Creates the device with explicit options.
            pub fn with_options(pin: u32, options: PwmOptions) -> Result<Self> {
                check_unit(options.initial_value, "initial_value")?;
                let factory = resolve(&options.pin_factory)?;
                Ok(Self {
                    inner: OutputInner::new(
                        &factory,
                        pin,
                        options.active_high,
                        Some(options.frequency),
                        Some(options.initial_value),
                    )?,
                })
            }

            /// Full brightness / duty cycle.
            pub fn on(&self) -> Result<()> {
                self.inner.set(1.0)
            }

            /// Off.
            pub fn off(&self) -> Result<()> {
                self.inner.set(0.0)
            }

            /// Sets the value to `1 - value`.
            pub fn toggle(&self) -> Result<()> {
                self.inner.set(1.0 - self.value())
            }

            /// Current value, `0.0..=1.0`.
            pub fn value(&self) -> f64 {
                self.inner.read()
            }

            /// Sets the value, `0.0..=1.0`.
            pub fn set_value(&self, value: f64) -> Result<()> {
                self.inner.set(check_unit(value, "value")?)
            }

            /// `true` if the value is non-zero.
            pub fn is_active(&self) -> bool {
                self.value() > 0.0
            }

            /// PWM frequency in Hz.
            pub fn frequency(&self) -> f64 {
                self.inner.pin.frequency().unwrap_or(0.0)
            }

            /// Changes the PWM frequency.
            pub fn set_frequency(&self, frequency: f64) -> Result<()> {
                self.inner.pin.set_frequency(Some(frequency))
            }

            /// Blinks between fully on and off.
            pub fn blink(
                &self,
                on_time: Duration,
                off_time: Duration,
                n: Option<u32>,
                background: bool,
            ) -> Result<()> {
                self.inner.blink(Blink {
                    on_time,
                    off_time,
                    n,
                    background,
                    ..Blink::default()
                })
            }

            /// Blinks with a full [`Blink`] spec (including fades).
            pub fn blink_with(&self, spec: Blink) -> Result<()> {
                self.inner.blink(spec)
            }

            /// Fades in and out repeatedly.
            pub fn pulse(
                &self,
                fade_in: Duration,
                fade_out: Duration,
                n: Option<u32>,
                background: bool,
            ) -> Result<()> {
                self.inner.blink(Blink {
                    n,
                    background,
                    ..Blink::pulse(fade_in, fade_out)
                })
            }

            /// An endless iterator over the value.
            pub fn values(&self) -> Values<f64> {
                self.inner.values(|v| v)
            }

            /// Continuously sets the value from `source` in a background
            /// thread. Accepts anything convertible to `f64` (including `bool`).
            pub fn set_source<I>(&self, source: I)
            where
                I: IntoIterator,
                I::Item: Into<f64> + 'static,
                I::IntoIter: Send + 'static,
            {
                self.inner.start_source(source.into_iter().map(Into::into), unit);
            }

            common_output_methods!();
        }
    };
}

pwm_output!(
    /// A generic PWM output (gpiozero's `PWMOutputDevice`).
    PwmOutputDevice
);
pwm_output!(
    /// An LED with variable brightness (gpiozero's `PWMLED`).
    ///
    /// ```no_run
    /// use gpio_zero::PwmLed;
    /// use std::time::Duration;
    /// let led = PwmLed::new(17)?;
    /// led.set_value(0.5)?;
    /// led.pulse(Duration::from_secs(1), Duration::from_secs(1), None, true)?;
    /// # Ok::<(), gpio_zero::Error>(())
    /// ```
    PwmLed
);

impl PwmLed {
    /// `true` if the LED is lit at all.
    pub fn is_lit(&self) -> bool {
        self.is_active()
    }
}
