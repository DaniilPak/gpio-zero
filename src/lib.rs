//! # gpio-zero
//!
//! A simple, high-level interface to GPIO devices on the Raspberry Pi and
//! other Linux boards, modelled on Python's
//! [gpiozero](https://gpiozero.readthedocs.io/). This is an independent
//! project, not affiliated with the gpiozero authors.
//!
//! ```no_run
//! use gpio_zero::{Button, Led};
//!
//! let led = Led::new(17)?;
//! let button = Button::new(2)?;
//!
//! // gpiozero: led.source = button.values
//! led.set_source(button.values());
//! # Ok::<(), gpio_zero::Error>(())
//! ```
//!
//! ## Backends
//!
//! Devices get their pins from a [`PinFactory`](pins::PinFactory):
//!
//! - **`cdev`** (default feature): the Linux GPIO character device
//!   (`/dev/gpiochipN`) via `gpiocdev`, plus `spidev` for hardware SPI.
//!   Works on every Raspberry Pi including the Pi 5 (RP1), and on most other
//!   Linux boards. PWM is done in software.
//! - **`rppal`** (optional feature): the `rppal` crate (Pi 0–5), with its
//!   software PWM and hardware SPI.
//! - **mock** (always available): [`MockFactory`](pins::mock::MockFactory)
//!   simulates pins, SPI chips, HC-SR04 echoes and RC charging, with a
//!   virtual clock so timing-based behaviour is testable without hardware.
//!
//! Pick a backend with `GPIOZERO_PIN_FACTORY=cdev|rppal|mock`, or call
//! [`set_default_factory`](pins::set_default_factory).
//!
//! ## Handles, callbacks and threads
//!
//! Every device is a cheap-to-clone handle; clone it into callbacks
//! (`when_pressed`, ...). The hardware is released when the last handle is
//! dropped. Blinking, pulsing, hold detection, sensor sampling and `source`
//! loops run in background threads that are stopped and joined
//! automatically.

#![warn(missing_docs)]

pub mod boards;
pub mod clock;
pub mod devices;
pub mod error;
pub mod input;
pub mod output;
pub mod pins;
pub mod spi_devices;
pub mod tools;

pub use boards::{
    ButtonBoard, LedBarGraph, LedBoard, LedBoardOptions, Robot, RobotOptions, TrafficLights,
};
pub use devices::{Blink, Values};
pub use error::{Error, Result};
pub use input::{
    Button, ButtonOptions, DigitalInputDevice, DistanceSensor, DistanceSensorOptions, InputDevice,
    InputOptions, LightSensor, LightSensorOptions, LineSensor, MotionSensor, RotaryEncoder,
    RotaryEncoderOptions, SmoothedInputOptions,
};
pub use output::{
    AngularServo, AngularServoOptions, Buzzer, Color, DigitalOutputDevice, Led, Motor,
    MotorOptions, OutputDevice, OutputOptions, PhaseEnableMotor, PhaseEnableMotorOptions, PwmLed,
    PwmOptions, PwmOutputDevice, RgbLed, RgbLedOptions, Servo, ServoOptions, TonalBuzzer,
    TonalBuzzerOptions, Tone,
};
pub use spi_devices::{
    Mcp3001, Mcp3002, Mcp3004, Mcp3008, Mcp3201, Mcp3202, Mcp3204, Mcp3208, McpAdc, McpChip,
    McpOptions, MockMcp,
};

/// Everything needed for typical programs: `use gpio_zero::prelude::*;`.
pub mod prelude {
    pub use crate::boards::*;
    pub use crate::devices::{Blink, Values};
    pub use crate::error::{Error, Result};
    pub use crate::input::*;
    pub use crate::output::*;
    pub use crate::pins::mock::MockFactory;
    pub use crate::pins::{Pin, PinFactory, SpiConfig};
    pub use crate::spi_devices::*;
    pub use crate::tools;
    pub use std::time::Duration;
}

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;
