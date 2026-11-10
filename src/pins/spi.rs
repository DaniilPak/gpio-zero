//! SPI configuration, the [`SpiInterface`] trait and a bit-banged
//! [`SoftwareSpi`] that works on any pin factory.

use std::fmt;
use std::sync::{Arc, Mutex};

use super::{PinFactory, PinFunction, PinHandle, Pull};
use crate::clock::lock;
use crate::error::Result;

/// How to reach an SPI device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SpiConfig {
    /// A kernel/hardware SPI bus (`/dev/spidev{port}.{device}`).
    Hardware {
        /// SPI bus number.
        port: u8,
        /// Chip-select number.
        device: u8,
        /// Clock speed in Hz.
        clock_hz: u32,
    },
    /// Bit-banged SPI on arbitrary GPIO pins (mode 0, chip select active low).
    Software {
        /// Clock pin.
        clock: u32,
        /// Master-out pin (optional for read-only devices).
        mosi: Option<u32>,
        /// Master-in pin (optional for write-only devices).
        miso: Option<u32>,
        /// Chip-select pin.
        select: u32,
    },
}

impl Default for SpiConfig {
    fn default() -> Self {
        SpiConfig::hardware(0, 0)
    }
}

impl SpiConfig {
    /// Hardware SPI on bus `port`, chip select `device`, at 1 MHz.
    pub fn hardware(port: u8, device: u8) -> Self {
        SpiConfig::Hardware {
            port,
            device,
            clock_hz: 1_000_000,
        }
    }

    /// Software SPI on the given pins.
    pub fn software(clock: u32, mosi: u32, miso: u32, select: u32) -> Self {
        SpiConfig::Software {
            clock,
            mosi: Some(mosi),
            miso: Some(miso),
            select,
        }
    }
}

/// A full-duplex SPI interface.
pub trait SpiInterface: Send + Sync + fmt::Debug {
    /// Writes `tx` while reading the same number of bytes.
    fn transfer(&self, tx: &[u8]) -> Result<Vec<u8>>;
}

/// Bit-banged SPI (mode 0, MSB first, active-low chip select).
pub struct SoftwareSpi {
    clock: PinHandle,
    mosi: Option<PinHandle>,
    miso: Option<PinHandle>,
    select: PinHandle,
    busy: Mutex<()>,
}

impl fmt::Debug for SoftwareSpi {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SoftwareSpi")
            .field("clock", &self.clock)
            .field("mosi", &self.mosi)
            .field("miso", &self.miso)
            .field("select", &self.select)
            .finish()
    }
}

impl SoftwareSpi {
    /// Reserves the pins and configures them.
    pub fn new(
        factory: &Arc<dyn PinFactory>,
        clock: u32,
        mosi: Option<u32>,
        miso: Option<u32>,
        select: u32,
    ) -> Result<Self> {
        let clock = PinHandle::new(factory, clock)?;
        let mosi = mosi.map(|n| PinHandle::new(factory, n)).transpose()?;
        let miso = miso.map(|n| PinHandle::new(factory, n)).transpose()?;
        let select = PinHandle::new(factory, select)?;
        select.set_function(PinFunction::Output)?;
        select.set_state(1.0)?;
        clock.set_function(PinFunction::Output)?;
        clock.set_state(0.0)?;
        if let Some(p) = &mosi {
            p.set_function(PinFunction::Output)?;
            p.set_state(0.0)?;
        }
        if let Some(p) = &miso {
            p.set_function(PinFunction::Input)?;
            p.set_pull(Pull::Floating)?;
        }
        Ok(SoftwareSpi {
            clock,
            mosi,
            miso,
            select,
            busy: Mutex::new(()),
        })
    }
}

impl SpiInterface for SoftwareSpi {
    fn transfer(&self, tx: &[u8]) -> Result<Vec<u8>> {
        let _g = lock(&self.busy);
        let mut rx = Vec::with_capacity(tx.len());
        self.select.set_state(0.0)?;
        let result = (|| {
            for &byte in tx {
                let mut inb = 0u8;
                for bit in (0..8).rev() {
                    if let Some(p) = &self.mosi {
                        p.set_state(if byte >> bit & 1 == 1 { 1.0 } else { 0.0 })?;
                    }
                    self.clock.set_state(1.0)?;
                    if let Some(p) = &self.miso
                        && p.state()? >= 0.5
                    {
                        inb |= 1 << bit;
                    }
                    self.clock.set_state(0.0)?;
                }
                rx.push(inb);
            }
            Ok(())
        })();
        self.select.set_state(1.0)?;
        result.map(|()| rx)
    }
}
