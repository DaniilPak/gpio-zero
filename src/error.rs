//! Error type used throughout the crate.

use std::fmt;

/// Errors returned by pins, pin factories and devices.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The pin is already reserved by another device.
    PinInUse(u32),
    /// The pin number or specification is not valid for this factory.
    InvalidPin(String),
    /// A value outside the accepted range was supplied.
    InvalidValue(String),
    /// The pin does not support PWM.
    PwmUnsupported(u32),
    /// Attempted to write to a pin configured as an input.
    PinSetInput(u32),
    /// The requested pull configuration is not supported.
    PullUnsupported(u32),
    /// The factory has no SPI support for the requested configuration.
    SpiUnsupported(String),
    /// No default pin factory could be created.
    NoPinFactory(String),
    /// The device has been closed.
    DeviceClosed,
    /// An I/O error from the operating system.
    Io(std::io::Error),
    /// An error reported by a backend library.
    Backend(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::PinInUse(n) => write!(f, "pin GPIO{n} is already in use"),
            Error::InvalidPin(s) => write!(f, "invalid pin: {s}"),
            Error::InvalidValue(s) => write!(f, "invalid value: {s}"),
            Error::PwmUnsupported(n) => write!(f, "PWM is not supported on GPIO{n}"),
            Error::PinSetInput(n) => write!(f, "cannot set the state of input pin GPIO{n}"),
            Error::PullUnsupported(n) => write!(f, "pull configuration not supported on GPIO{n}"),
            Error::SpiUnsupported(s) => write!(f, "SPI not supported: {s}"),
            Error::NoPinFactory(s) => write!(f, "no pin factory available: {s}"),
            Error::DeviceClosed => write!(f, "device is closed"),
            Error::Io(e) => write!(f, "I/O error: {e}"),
            Error::Backend(s) => write!(f, "backend error: {s}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

/// Result alias using [`Error`].
pub type Result<T> = std::result::Result<T, Error>;

pub(crate) fn invalid<T>(msg: impl Into<String>) -> Result<T> {
    Err(Error::InvalidValue(msg.into()))
}
