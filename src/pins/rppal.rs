//! [`rppal`](https://docs.rs/rppal) backend for Raspberry Pi boards.
//!
//! GPIO through rppal (pins numbered by BCM GPIO), rppal's software PWM on
//! any pin, rppal interrupts for edges, and rppal's hardware SPI.

use std::fmt;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use rppal::gpio::{Gpio, InputPin, Level, OutputPin, Trigger};

use super::{
    EdgeCallback, Edges, Pin, PinFactory, PinFunction, Pull, Reservations, SoftwareSpi, SpiConfig,
    SpiInterface,
};
use crate::clock::{Clock, RealClock, lock};
use crate::error::{Error, Result, invalid};

fn backend(e: impl fmt::Display) -> Error {
    Error::Backend(e.to_string())
}

/// The rppal pin factory.
pub struct RppalFactory {
    me: Weak<RppalFactory>,
    gpio: Gpio,
    reservations: Reservations,
    clock: Arc<dyn Clock>,
}

impl fmt::Debug for RppalFactory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RppalFactory").finish_non_exhaustive()
    }
}

impl RppalFactory {
    /// Opens the GPIO peripheral.
    pub fn new() -> Result<Arc<Self>> {
        let gpio = Gpio::new().map_err(|e| Error::NoPinFactory(format!("rppal: {e}")))?;
        Ok(Arc::new_cyclic(|me| RppalFactory {
            me: me.clone(),
            gpio,
            reservations: Reservations::default(),
            clock: Arc::new(RealClock),
        }))
    }
}

impl PinFactory for RppalFactory {
    fn name(&self) -> &str {
        "rppal"
    }

    fn reserve_pin(&self, number: u32) -> Result<Arc<dyn Pin>> {
        let n = u8::try_from(number).map_err(|_| Error::InvalidPin(format!("GPIO{number}")))?;
        self.reservations.reserve(number)?;
        match RppalPin::open(self.gpio.clone(), n) {
            Ok(p) => Ok(p),
            Err(e) => {
                self.reservations.release(number);
                Err(e)
            }
        }
    }

    fn release_pin(&self, number: u32) {
        self.reservations.release(number);
    }

    fn spi(&self, config: &SpiConfig) -> Result<Box<dyn SpiInterface>> {
        use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
        match *config {
            SpiConfig::Hardware {
                port,
                device,
                clock_hz,
            } => {
                let bus = match port {
                    0 => Bus::Spi0,
                    1 => Bus::Spi1,
                    2 => Bus::Spi2,
                    3 => Bus::Spi3,
                    4 => Bus::Spi4,
                    5 => Bus::Spi5,
                    6 => Bus::Spi6,
                    _ => return Err(Error::SpiUnsupported(format!("no SPI bus {port}"))),
                };
                let ss = match device {
                    0 => SlaveSelect::Ss0,
                    1 => SlaveSelect::Ss1,
                    2 => SlaveSelect::Ss2,
                    _ => {
                        return Err(Error::SpiUnsupported(format!("no chip select {device}")));
                    }
                };
                let spi = Spi::new(bus, ss, clock_hz, Mode::Mode0)
                    .map_err(|e| Error::SpiUnsupported(e.to_string()))?;
                Ok(Box::new(RppalSpi(Mutex::new(spi))))
            }
            SpiConfig::Software {
                clock,
                mosi,
                miso,
                select,
            } => {
                let me: Arc<dyn PinFactory> = self.me.upgrade().ok_or(Error::DeviceClosed)?;
                Ok(Box::new(SoftwareSpi::new(&me, clock, mosi, miso, select)?))
            }
        }
    }

    fn clock(&self) -> Arc<dyn Clock> {
        self.clock.clone()
    }
}

struct RppalSpi(Mutex<rppal::spi::Spi>);

impl fmt::Debug for RppalSpi {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("RppalSpi")
    }
}

impl SpiInterface for RppalSpi {
    fn transfer(&self, tx: &[u8]) -> Result<Vec<u8>> {
        let mut rx = vec![0u8; tx.len()];
        lock(&self.0).transfer(&mut rx, tx).map_err(backend)?;
        Ok(rx)
    }
}

enum Mode {
    Input(InputPin),
    Output(OutputPin),
}

struct State {
    mode: Option<Mode>,
    function: PinFunction,
    pull: Pull,
    bounce: Option<Duration>,
    edges: Edges,
    level: bool,
    pwm: Option<(f64, f64)>,
}

/// A GPIO pin accessed through rppal.
pub struct RppalPin {
    number: u8,
    gpio: Gpio,
    st: Mutex<State>,
    callback: Arc<Mutex<Option<EdgeCallback>>>,
}

impl fmt::Debug for RppalPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RppalPin")
            .field("number", &self.number)
            .finish()
    }
}

impl RppalPin {
    fn open(gpio: Gpio, number: u8) -> Result<Arc<dyn Pin>> {
        let pin = Arc::new(RppalPin {
            number,
            gpio,
            st: Mutex::new(State {
                mode: None,
                function: PinFunction::Input,
                pull: Pull::Floating,
                bounce: None,
                edges: Edges::Both,
                level: false,
                pwm: None,
            }),
            callback: Arc::default(),
        });
        {
            let mut st = lock(&pin.st);
            pin.apply(&mut st)?;
        }
        Ok(pin)
    }

    fn apply(&self, st: &mut State) -> Result<()> {
        st.mode = None;
        let pin = self.gpio.get(self.number).map_err(backend)?;
        match st.function {
            PinFunction::Input => {
                let mut ip = match st.pull {
                    Pull::Up => pin.into_input_pullup(),
                    Pull::Down => pin.into_input_pulldown(),
                    Pull::Floating => pin.into_input(),
                };
                let trigger = match st.edges {
                    Edges::None => None,
                    Edges::Rising => Some(Trigger::RisingEdge),
                    Edges::Falling => Some(Trigger::FallingEdge),
                    Edges::Both => Some(Trigger::Both),
                };
                if let Some(trigger) = trigger {
                    let cb = self.callback.clone();
                    ip.set_async_interrupt(trigger, st.bounce, move |ev| {
                        let f = lock(&cb).clone();
                        if let Some(f) = f {
                            f(ev.timestamp, ev.trigger == Trigger::RisingEdge);
                        }
                    })
                    .map_err(backend)?;
                }
                st.mode = Some(Mode::Input(ip));
            }
            PinFunction::Output => {
                let mut op = if st.level {
                    pin.into_output_high()
                } else {
                    pin.into_output_low()
                };
                if let Some((f, d)) = st.pwm {
                    op.set_pwm_frequency(f, d).map_err(backend)?;
                }
                st.mode = Some(Mode::Output(op));
            }
        }
        Ok(())
    }
}

impl Pin for RppalPin {
    fn number(&self) -> u32 {
        u32::from(self.number)
    }

    fn function(&self) -> PinFunction {
        lock(&self.st).function
    }

    fn set_function(&self, function: PinFunction) -> Result<()> {
        let mut st = lock(&self.st);
        if st.function == function {
            return Ok(());
        }
        st.function = function;
        if function == PinFunction::Input {
            st.pwm = None;
        }
        self.apply(&mut st)
    }

    fn state(&self) -> Result<f64> {
        let st = lock(&self.st);
        if let Some((_, d)) = st.pwm {
            return Ok(d);
        }
        Ok(match &st.mode {
            Some(Mode::Input(p)) => f64::from(u8::from(p.read() == Level::High)),
            Some(Mode::Output(_)) => f64::from(u8::from(st.level)),
            None => return Err(Error::DeviceClosed),
        })
    }

    fn set_state(&self, v: f64) -> Result<()> {
        if !(0.0..=1.0).contains(&v) {
            return invalid(format!("pin state {v} is outside 0..=1"));
        }
        let mut st = lock(&self.st);
        let pwm = st.pwm;
        let Some(Mode::Output(op)) = &mut st.mode else {
            return Err(Error::PinSetInput(u32::from(self.number)));
        };
        if let Some((f, _)) = pwm {
            op.set_pwm_frequency(f, v).map_err(backend)?;
            st.pwm = Some((f, v));
            return Ok(());
        }
        if v != 0.0 && v != 1.0 {
            return Err(Error::PwmUnsupported(u32::from(self.number)));
        }
        op.write(Level::from(v == 1.0));
        st.level = v == 1.0;
        Ok(())
    }

    fn pull(&self) -> Pull {
        lock(&self.st).pull
    }

    fn set_pull(&self, pull: Pull) -> Result<()> {
        let mut st = lock(&self.st);
        st.pull = pull;
        if st.function == PinFunction::Input {
            self.apply(&mut st)?;
        }
        Ok(())
    }

    fn frequency(&self) -> Option<f64> {
        lock(&self.st).pwm.map(|(f, _)| f)
    }

    fn set_frequency(&self, frequency: Option<f64>) -> Result<()> {
        let mut st = lock(&self.st);
        let duty = st
            .pwm
            .map(|(_, d)| d)
            .unwrap_or(f64::from(u8::from(st.level)));
        let Some(Mode::Output(op)) = &mut st.mode else {
            return Err(Error::PinSetInput(u32::from(self.number)));
        };
        match frequency {
            Some(f) => {
                if f <= 0.0 || !f.is_finite() {
                    return invalid(format!("frequency {f} must be positive"));
                }
                op.set_pwm_frequency(f, duty).map_err(backend)?;
                st.pwm = Some((f, duty));
            }
            None => {
                op.clear_pwm().map_err(backend)?;
                op.set_low();
                st.pwm = None;
                st.level = false;
            }
        }
        Ok(())
    }

    fn bounce(&self) -> Option<Duration> {
        lock(&self.st).bounce
    }

    fn set_bounce(&self, bounce: Option<Duration>) -> Result<()> {
        let mut st = lock(&self.st);
        st.bounce = bounce;
        if st.function == PinFunction::Input {
            self.apply(&mut st)?;
        }
        Ok(())
    }

    fn edges(&self) -> Edges {
        lock(&self.st).edges
    }

    fn set_edges(&self, edges: Edges) -> Result<()> {
        let mut st = lock(&self.st);
        st.edges = edges;
        if st.function == PinFunction::Input {
            self.apply(&mut st)?;
        }
        Ok(())
    }

    fn set_when_changed(&self, callback: Option<EdgeCallback>) -> Result<()> {
        *lock(&self.callback) = callback;
        Ok(())
    }

    fn close(&self) {
        *lock(&self.callback) = None;
        let mode = lock(&self.st).mode.take();
        drop(mode);
    }
}
