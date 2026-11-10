use std::fmt;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::OutputInner;
use crate::clock::{Clock, Task};
use crate::devices::{Blink, SourceSlot, Values, animate, blink_frames, check_unit, stop_task};
use crate::error::{Error, Result, invalid};
use crate::pins::{PinFactory, resolve};

/// An RGB colour with components in `0.0..=1.0`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Color {
    /// Red component.
    pub red: f64,
    /// Green component.
    pub green: f64,
    /// Blue component.
    pub blue: f64,
}

impl Color {
    /// Black (off).
    pub const BLACK: Color = Color::new(0.0, 0.0, 0.0);
    /// White (full on).
    pub const WHITE: Color = Color::new(1.0, 1.0, 1.0);
    /// Red.
    pub const RED: Color = Color::new(1.0, 0.0, 0.0);
    /// Green.
    pub const GREEN: Color = Color::new(0.0, 1.0, 0.0);
    /// Blue.
    pub const BLUE: Color = Color::new(0.0, 0.0, 1.0);
    /// Yellow.
    pub const YELLOW: Color = Color::new(1.0, 1.0, 0.0);
    /// Cyan.
    pub const CYAN: Color = Color::new(0.0, 1.0, 1.0);
    /// Magenta.
    pub const MAGENTA: Color = Color::new(1.0, 0.0, 1.0);

    /// Creates a colour from components in `0.0..=1.0`.
    pub const fn new(red: f64, green: f64, blue: f64) -> Self {
        Color { red, green, blue }
    }

    /// Creates a colour from 8-bit components.
    pub fn from_rgb8(red: u8, green: u8, blue: u8) -> Self {
        Color::new(
            f64::from(red) / 255.0,
            f64::from(green) / 255.0,
            f64::from(blue) / 255.0,
        )
    }

    fn to_array(self) -> [f64; 3] {
        [self.red, self.green, self.blue]
    }

    fn validate(self) -> Result<Self> {
        for c in self.to_array() {
            check_unit(c, "colour component")?;
        }
        Ok(self)
    }
}

impl From<(f64, f64, f64)> for Color {
    fn from((r, g, b): (f64, f64, f64)) -> Self {
        Color::new(r, g, b)
    }
}

impl From<Color> for (f64, f64, f64) {
    fn from(c: Color) -> Self {
        (c.red, c.green, c.blue)
    }
}

impl FromStr for Color {
    type Err = Error;

    /// Parses `#rrggbb` / `rrggbb` hex or one of the basic colour names.
    fn from_str(s: &str) -> Result<Self> {
        let named = match s.to_ascii_lowercase().as_str() {
            "black" => Some(Color::BLACK),
            "white" => Some(Color::WHITE),
            "red" => Some(Color::RED),
            "green" | "lime" => Some(Color::GREEN),
            "blue" => Some(Color::BLUE),
            "yellow" => Some(Color::YELLOW),
            "cyan" => Some(Color::CYAN),
            "magenta" => Some(Color::MAGENTA),
            _ => None,
        };
        if let Some(c) = named {
            return Ok(c);
        }
        let hex = s.strip_prefix('#').unwrap_or(s);
        if hex.len() != 6 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
            return invalid(format!("cannot parse colour {s:?}"));
        }
        let byte = |i: usize| u8::from_str_radix(&hex[i..i + 2], 16).unwrap_or(0);
        Ok(Color::from_rgb8(byte(0), byte(2), byte(4)))
    }
}

/// Options for [`RgbLed`].
#[derive(Clone)]
pub struct RgbLedOptions {
    /// `true` (default) for common-cathode LEDs; `false` for common-anode.
    pub active_high: bool,
    /// Initial colour (default black).
    pub initial_value: Color,
    /// Use PWM for each channel (default `true`); otherwise only 0/1 levels.
    pub pwm: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for RgbLedOptions {
    fn default() -> Self {
        RgbLedOptions {
            active_high: true,
            initial_value: Color::BLACK,
            pwm: true,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for RgbLedOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RgbLedOptions")
            .field("active_high", &self.active_high)
            .field("initial_value", &self.initial_value)
            .field("pwm", &self.pwm)
            .finish_non_exhaustive()
    }
}

struct RgbInner {
    leds: [Arc<OutputInner>; 3],
    pwm: bool,
    clock: Arc<dyn Clock>,
    blink: Mutex<Option<Task>>,
    source: SourceSlot,
}

impl RgbInner {
    fn write(&self, c: Color) -> Result<()> {
        let c = c.validate()?;
        if !self.pwm && c.to_array().iter().any(|v| *v != 0.0 && *v != 1.0) {
            return invalid("non-PWM RgbLed only accepts 0 or 1 components");
        }
        for (led, v) in self.leds.iter().zip(c.to_array()) {
            led.write(v)?;
        }
        Ok(())
    }

    fn set(&self, c: Color) -> Result<()> {
        stop_task(&self.blink);
        self.write(c)
    }

    fn read(&self) -> Color {
        Color::new(
            self.leds[0].read(),
            self.leds[1].read(),
            self.leds[2].read(),
        )
    }
}

impl Drop for RgbInner {
    fn drop(&mut self) {
        self.source.stop();
        stop_task(&self.blink);
    }
}

/// A full-colour LED made of three LEDs (gpiozero's `RGBLED`).
#[derive(Clone)]
pub struct RgbLed {
    inner: Arc<RgbInner>,
}

impl fmt::Debug for RgbLed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RgbLed")
            .field("value", &self.value())
            .finish()
    }
}

impl RgbLed {
    /// Creates an RGB LED on three GPIO pins with default options.
    pub fn new(red: u32, green: u32, blue: u32) -> Result<Self> {
        Self::with_options(red, green, blue, RgbLedOptions::default())
    }

    /// Creates an RGB LED with explicit options.
    pub fn with_options(red: u32, green: u32, blue: u32, options: RgbLedOptions) -> Result<Self> {
        let factory = resolve(&options.pin_factory)?;
        let freq = options.pwm.then_some(100.0);
        let init = options.initial_value.validate()?.to_array();
        let r = OutputInner::new(&factory, red, options.active_high, freq, Some(init[0]))?;
        let g = OutputInner::new(&factory, green, options.active_high, freq, Some(init[1]))?;
        let b = OutputInner::new(&factory, blue, options.active_high, freq, Some(init[2]))?;
        let inner = RgbInner {
            leds: [r, g, b],
            pwm: options.pwm,
            clock: factory.clock(),
            blink: Mutex::new(None),
            source: SourceSlot::default(),
        };
        inner.write(options.initial_value)?;
        Ok(RgbLed {
            inner: Arc::new(inner),
        })
    }

    /// White.
    pub fn on(&self) -> Result<()> {
        self.inner.set(Color::WHITE)
    }

    /// Black.
    pub fn off(&self) -> Result<()> {
        self.inner.set(Color::BLACK)
    }

    /// Inverts each component.
    pub fn toggle(&self) -> Result<()> {
        let c = self.value();
        self.inner
            .set(Color::new(1.0 - c.red, 1.0 - c.green, 1.0 - c.blue))
    }

    /// Current colour.
    pub fn value(&self) -> Color {
        self.inner.read()
    }

    /// Sets the colour.
    pub fn set_value(&self, color: impl Into<Color>) -> Result<()> {
        self.inner.set(color.into())
    }

    /// Alias of [`value`](Self::value).
    pub fn color(&self) -> Color {
        self.value()
    }

    /// Alias of [`set_value`](Self::set_value).
    pub fn set_color(&self, color: impl Into<Color>) -> Result<()> {
        self.set_value(color)
    }

    /// Red component.
    pub fn red(&self) -> f64 {
        self.value().red
    }

    /// Sets the red component.
    pub fn set_red(&self, v: f64) -> Result<()> {
        self.set_value(Color {
            red: v,
            ..self.value()
        })
    }

    /// Green component.
    pub fn green(&self) -> f64 {
        self.value().green
    }

    /// Sets the green component.
    pub fn set_green(&self, v: f64) -> Result<()> {
        self.set_value(Color {
            green: v,
            ..self.value()
        })
    }

    /// Blue component.
    pub fn blue(&self) -> f64 {
        self.value().blue
    }

    /// Sets the blue component.
    pub fn set_blue(&self, v: f64) -> Result<()> {
        self.set_value(Color {
            blue: v,
            ..self.value()
        })
    }

    /// `true` if any component is non-zero.
    pub fn is_active(&self) -> bool {
        self.value() != Color::BLACK
    }

    /// Alias of [`is_active`](Self::is_active).
    pub fn is_lit(&self) -> bool {
        self.is_active()
    }

    /// Blinks between `on_color` and `off_color`.
    pub fn blink(
        &self,
        on_time: Duration,
        off_time: Duration,
        on_color: Color,
        off_color: Color,
        n: Option<u32>,
        background: bool,
    ) -> Result<()> {
        self.blink_with(
            Blink {
                on_time,
                off_time,
                n,
                background,
                ..Blink::default()
            },
            on_color,
            off_color,
        )
    }

    /// Blinks with a full [`Blink`] spec.
    pub fn blink_with(&self, spec: Blink, on_color: Color, off_color: Color) -> Result<()> {
        on_color.validate()?;
        off_color.validate()?;
        if spec.has_fade() && !self.inner.pwm {
            return invalid("fades require pwm = true");
        }
        let frames = blink_frames(&spec, &on_color.to_array(), &off_color.to_array());
        let writers = self.inner.leds.iter().map(|l| l.writer()).collect();
        animate(
            &self.inner.blink,
            &self.inner.clock,
            writers,
            frames,
            spec.n,
            spec.background,
        );
        Ok(())
    }

    /// Fades between `off_color` and `on_color`.
    pub fn pulse(
        &self,
        fade_in: Duration,
        fade_out: Duration,
        on_color: Color,
        off_color: Color,
        n: Option<u32>,
        background: bool,
    ) -> Result<()> {
        self.blink_with(
            Blink {
                n,
                background,
                ..Blink::pulse(fade_in, fade_out)
            },
            on_color,
            off_color,
        )
    }

    /// An endless iterator over the colour.
    pub fn values(&self) -> Values<Color> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || weak.upgrade().map(|i| i.read()))
    }

    /// Continuously sets the colour from `source`.
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator,
        I::Item: Into<Color> + 'static,
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

    /// The GPIO numbers (red, green, blue).
    pub fn pins(&self) -> [u32; 3] {
        [
            self.inner.leds[0].pin.number(),
            self.inner.leds[1].pin.number(),
            self.inner.leds[2].pin.number(),
        ]
    }
}
