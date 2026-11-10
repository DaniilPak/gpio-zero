//! The pin layer: [`Pin`], [`PinFactory`] and the available backends.
//!
//! Devices never talk to hardware directly. They ask a [`PinFactory`] for
//! [`Pin`]s (and SPI interfaces), exactly like gpiozero's pin factories. The
//! factory used is, in order of priority:
//!
//! 1. the factory passed in a device's options (`pin_factory`),
//! 2. the factory set for the current thread with [`set_thread_factory`],
//! 3. the global factory set with [`set_default_factory`],
//! 4. one chosen by the `GPIOZERO_PIN_FACTORY` environment variable
//!    (`cdev`, `rppal` or `mock`; `mock` runs in real time),
//! 5. the first backend compiled in that works: `cdev`, then `rppal`.

use std::cell::RefCell;
use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::clock::{Clock, lock};
use crate::error::{Error, Result};

pub mod mock;
pub mod spi;

#[cfg(all(feature = "cdev", target_os = "linux"))]
pub mod cdev;

#[cfg(all(feature = "rppal", target_os = "linux"))]
pub mod rppal;

pub use spi::{SoftwareSpi, SpiConfig, SpiInterface};

/// What a pin is configured as.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PinFunction {
    /// Input (the default for unconfigured pins).
    #[default]
    Input,
    /// Output.
    Output,
}

/// Internal pull resistor configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Pull {
    /// No pull resistor (floating).
    #[default]
    Floating,
    /// Pull up to 3V3.
    Up,
    /// Pull down to ground.
    Down,
}

/// Which edges trigger the pin's change callback.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Edges {
    /// No edge detection.
    None,
    /// Low-to-high transitions.
    Rising,
    /// High-to-low transitions.
    Falling,
    /// Both transitions (the default).
    #[default]
    Both,
}

impl Edges {
    /// Whether a transition to `state` should be reported.
    pub fn matches(self, state: bool) -> bool {
        match self {
            Edges::None => false,
            Edges::Rising => state,
            Edges::Falling => !state,
            Edges::Both => true,
        }
    }
}

/// Callback invoked on a pin edge with the edge timestamp (on the factory
/// clock's time base) and the new physical state.
pub type EdgeCallback = Arc<dyn Fn(Duration, bool) + Send + Sync>;

/// A single GPIO pin. All methods take `&self`; implementations use interior
/// mutability so pins can be shared between a device and its background tasks.
///
/// `state` is a physical level in `0.0..=1.0`: `0.0`/`1.0` for digital pins,
/// or the duty cycle when PWM is active (a frequency is set).
pub trait Pin: Send + Sync + fmt::Debug {
    /// The pin's (BCM / line offset) number.
    fn number(&self) -> u32;
    /// Current function.
    fn function(&self) -> PinFunction;
    /// Sets the function (input/output).
    fn set_function(&self, function: PinFunction) -> Result<()>;
    /// Reads the state (input level, or last output level / duty cycle).
    fn state(&self) -> Result<f64>;
    /// Sets the output state (or the duty cycle when PWM is active).
    fn set_state(&self, value: f64) -> Result<()>;
    /// Current pull.
    fn pull(&self) -> Pull;
    /// Sets the pull resistor.
    fn set_pull(&self, pull: Pull) -> Result<()>;
    /// PWM frequency in Hz, or `None` when PWM is off.
    fn frequency(&self) -> Option<f64>;
    /// Enables PWM at the given frequency (or disables it with `None`).
    fn set_frequency(&self, frequency: Option<f64>) -> Result<()>;
    /// Debounce period applied to edge detection.
    fn bounce(&self) -> Option<Duration>;
    /// Sets the debounce period.
    fn set_bounce(&self, bounce: Option<Duration>) -> Result<()>;
    /// Edges reported to the change callback.
    fn edges(&self) -> Edges;
    /// Sets which edges are reported.
    fn set_edges(&self, edges: Edges) -> Result<()>;
    /// Installs (or removes) the edge callback.
    fn set_when_changed(&self, callback: Option<EdgeCallback>) -> Result<()>;
    /// Releases hardware resources. The pin returns to an input.
    fn close(&self);
}

/// Creates pins and SPI interfaces for a particular backend.
pub trait PinFactory: Send + Sync + fmt::Debug {
    /// Backend name (`"mock"`, `"cdev"`, `"rppal"`).
    fn name(&self) -> &str;
    /// Reserves pin `number` for exclusive use and returns it.
    fn reserve_pin(&self, number: u32) -> Result<Arc<dyn Pin>>;
    /// Releases a reservation made by [`reserve_pin`](PinFactory::reserve_pin).
    fn release_pin(&self, number: u32);
    /// Opens an SPI interface.
    fn spi(&self, config: &SpiConfig) -> Result<Box<dyn SpiInterface>>;
    /// The clock used for all timing on this factory.
    fn clock(&self) -> Arc<dyn Clock>;
}

/// Reservation bookkeeping shared by the factories.
#[derive(Debug, Default)]
pub(crate) struct Reservations(Mutex<std::collections::HashSet<u32>>);

impl Reservations {
    pub(crate) fn reserve(&self, n: u32) -> Result<()> {
        if lock(&self.0).insert(n) {
            Ok(())
        } else {
            Err(Error::PinInUse(n))
        }
    }

    pub(crate) fn release(&self, n: u32) {
        lock(&self.0).remove(&n);
    }
}

/// A reserved pin that is closed and released when dropped.
pub(crate) struct PinHandle {
    pub(crate) pin: Arc<dyn Pin>,
    pub(crate) factory: Arc<dyn PinFactory>,
}

impl fmt::Debug for PinHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "GPIO{}", self.pin.number())
    }
}

impl PinHandle {
    pub(crate) fn new(factory: &Arc<dyn PinFactory>, number: u32) -> Result<Self> {
        let pin = factory.reserve_pin(number)?;
        Ok(PinHandle {
            pin,
            factory: factory.clone(),
        })
    }
}

impl std::ops::Deref for PinHandle {
    type Target = dyn Pin;
    fn deref(&self) -> &Self::Target {
        &*self.pin
    }
}

impl Drop for PinHandle {
    fn drop(&mut self) {
        let _ = self.pin.set_when_changed(None);
        self.pin.close();
        self.factory.release_pin(self.pin.number());
    }
}

static DEFAULT: Mutex<Option<Arc<dyn PinFactory>>> = Mutex::new(None);

thread_local! {
    static THREAD_DEFAULT: RefCell<Option<Arc<dyn PinFactory>>> = const { RefCell::new(None) };
}

/// Sets the process-wide default pin factory.
pub fn set_default_factory(factory: Arc<dyn PinFactory>) {
    *lock(&DEFAULT) = Some(factory);
}

/// Sets (or clears) a default pin factory for the current thread only. It
/// takes precedence over the global default; handy for tests running in
/// parallel, each with its own [`MockFactory`](mock::MockFactory).
pub fn set_thread_factory(factory: Option<Arc<dyn PinFactory>>) {
    THREAD_DEFAULT.with(|t| *t.borrow_mut() = factory);
}

/// Returns the pin factory devices use when none is given explicitly,
/// creating the global default on first use.
pub fn default_factory() -> Result<Arc<dyn PinFactory>> {
    if let Some(f) = THREAD_DEFAULT.with(|t| t.borrow().clone()) {
        return Ok(f);
    }
    let mut g = lock(&DEFAULT);
    if let Some(f) = g.as_ref() {
        return Ok(f.clone());
    }
    let f = create_factory()?;
    *g = Some(f.clone());
    Ok(f)
}

pub(crate) fn resolve(factory: &Option<Arc<dyn PinFactory>>) -> Result<Arc<dyn PinFactory>> {
    match factory {
        Some(f) => Ok(f.clone()),
        None => default_factory(),
    }
}

fn create_factory() -> Result<Arc<dyn PinFactory>> {
    let requested = std::env::var("GPIOZERO_PIN_FACTORY").ok();
    match requested.as_deref().map(str::to_ascii_lowercase).as_deref() {
        Some("mock") => return Ok(mock::MockFactory::with_real_time()),
        #[cfg(all(feature = "cdev", target_os = "linux"))]
        Some("cdev") | Some("native") => return Ok(cdev::CdevFactory::new()?),
        #[cfg(all(feature = "rppal", target_os = "linux"))]
        Some("rppal") => return Ok(rppal::RppalFactory::new()?),
        Some(other) => {
            return Err(Error::NoPinFactory(format!(
                "GPIOZERO_PIN_FACTORY={other} is not available in this build"
            )));
        }
        None => {}
    }
    #[allow(unused_mut)]
    let mut errors: Vec<String> = Vec::new();
    #[cfg(all(feature = "cdev", target_os = "linux"))]
    match cdev::CdevFactory::new() {
        Ok(f) => return Ok(f),
        Err(e) => errors.push(format!("cdev: {e}")),
    }
    #[cfg(all(feature = "rppal", target_os = "linux"))]
    match rppal::RppalFactory::new() {
        Ok(f) => return Ok(f),
        Err(e) => errors.push(format!("rppal: {e}")),
    }
    if errors.is_empty() {
        errors.push(
            "no hardware backend compiled for this target; use MockFactory or \
             GPIOZERO_PIN_FACTORY=mock"
                .into(),
        );
    }
    Err(Error::NoPinFactory(errors.join("; ")))
}
