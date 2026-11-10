//! SPI analog-to-digital converters: the Microchip MCP3xxx family.

use std::fmt;
use std::sync::{Arc, Mutex};

use crate::clock::lock;
use crate::devices::Values;
use crate::error::{Result, invalid};
use crate::pins::mock::MockSpiDevice;
use crate::pins::{PinFactory, SpiConfig, SpiInterface, resolve};

/// The MCP3xxx chips supported, with their protocol details.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McpChip {
    /// 10-bit, 1 channel.
    Mcp3001,
    /// 10-bit, 2 channels.
    Mcp3002,
    /// 10-bit, 4 channels.
    Mcp3004,
    /// 10-bit, 8 channels.
    Mcp3008,
    /// 12-bit, 1 channel.
    Mcp3201,
    /// 12-bit, 2 channels.
    Mcp3202,
    /// 12-bit, 4 channels.
    Mcp3204,
    /// 12-bit, 8 channels.
    Mcp3208,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Protocol {
    /// No command; 2 sample clocks, null bit, data.
    Single,
    /// start, SGL, ODD, MSBF; null bit; data.
    Two,
    /// start, SGL, D2, D1, D0; sample clock; null bit; data.
    Multi,
}

impl McpChip {
    /// Resolution in bits.
    pub fn bits(self) -> u32 {
        match self {
            McpChip::Mcp3001 | McpChip::Mcp3002 | McpChip::Mcp3004 | McpChip::Mcp3008 => 10,
            _ => 12,
        }
    }

    /// Number of input channels.
    pub fn channels(self) -> u8 {
        match self {
            McpChip::Mcp3001 | McpChip::Mcp3201 => 1,
            McpChip::Mcp3002 | McpChip::Mcp3202 => 2,
            McpChip::Mcp3004 | McpChip::Mcp3204 => 4,
            McpChip::Mcp3008 | McpChip::Mcp3208 => 8,
        }
    }

    fn protocol(self) -> Protocol {
        match self.channels() {
            1 => Protocol::Single,
            2 => Protocol::Two,
            _ => Protocol::Multi,
        }
    }

    fn max_raw(self) -> u32 {
        (1 << self.bits()) - 1
    }

    /// The bytes to send to read `channel`.
    fn command(self, channel: u8, differential: bool) -> Vec<u8> {
        let sgl = u32::from(!differential);
        let ch = u32::from(channel);
        let (word, nbytes) = match self.protocol() {
            Protocol::Single => (0, 2),
            Protocol::Two => ((0b1000 | sgl << 2 | ch << 1 | 1) << (self.bits() + 1), {
                (self.bits() + 5).div_ceil(8)
            }),
            Protocol::Multi => ((0b10000 | sgl << 3 | ch) << (self.bits() + 2), {
                (self.bits() + 7).div_ceil(8)
            }),
        };
        (0..nbytes).rev().map(|i| (word >> (8 * i)) as u8).collect()
    }

    fn parse(self, rx: &[u8]) -> u32 {
        let word = rx.iter().fold(0u32, |acc, b| acc << 8 | u32::from(*b));
        match self.protocol() {
            Protocol::Single => (word >> (16 - 3 - self.bits())) & self.max_raw(),
            _ => word & self.max_raw(),
        }
    }
}

/// Options for the MCP3xxx ADCs.
#[derive(Clone)]
pub struct McpOptions {
    /// Differential mode: read `channel` relative to its pair partner.
    pub differential: bool,
    /// Reference voltage, used by `voltage()` (default 3.3 V).
    pub max_voltage: f64,
    /// SPI bus (default hardware SPI 0, chip select 0).
    pub spi: SpiConfig,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for McpOptions {
    fn default() -> Self {
        McpOptions {
            differential: false,
            max_voltage: 3.3,
            spi: SpiConfig::default(),
            pin_factory: None,
        }
    }
}

impl fmt::Debug for McpOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("McpOptions")
            .field("differential", &self.differential)
            .field("max_voltage", &self.max_voltage)
            .field("spi", &self.spi)
            .finish_non_exhaustive()
    }
}

struct McpInner {
    chip: McpChip,
    channel: u8,
    differential: bool,
    max_voltage: f64,
    spi: Box<dyn SpiInterface>,
}

impl McpInner {
    fn raw(&self) -> Result<u32> {
        let rx = self
            .spi
            .transfer(&self.chip.command(self.channel, self.differential))?;
        Ok(self.chip.parse(&rx))
    }
}

/// A channel of an MCP3xxx ADC. Use the chip-specific aliases
/// ([`Mcp3008`] etc.) to construct one.
#[derive(Clone)]
pub struct McpAdc {
    inner: Arc<McpInner>,
}

impl fmt::Debug for McpAdc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("McpAdc")
            .field("chip", &self.inner.chip)
            .field("channel", &self.inner.channel)
            .finish()
    }
}

impl McpAdc {
    /// Opens `channel` of `chip` with explicit options.
    pub fn open(chip: McpChip, channel: u8, options: McpOptions) -> Result<Self> {
        if channel >= chip.channels() {
            return invalid(format!(
                "channel {channel} is out of range for {chip:?} (0..{})",
                chip.channels()
            ));
        }
        if options.differential && chip.channels() == 1 {
            return invalid(format!("{chip:?} has no differential mode"));
        }
        let factory = resolve(&options.pin_factory)?;
        let spi = factory.spi(&options.spi)?;
        Ok(McpAdc {
            inner: Arc::new(McpInner {
                chip,
                channel,
                differential: options.differential,
                max_voltage: options.max_voltage,
                spi,
            }),
        })
    }

    /// The chip model.
    pub fn chip(&self) -> McpChip {
        self.inner.chip
    }

    /// The channel read.
    pub fn channel(&self) -> u8 {
        self.inner.channel
    }

    /// Whether differential mode is used.
    pub fn differential(&self) -> bool {
        self.inner.differential
    }

    /// Resolution in bits.
    pub fn bits(&self) -> u32 {
        self.inner.chip.bits()
    }

    /// The raw reading (`0..=2^bits - 1`).
    pub fn raw_value(&self) -> Result<u32> {
        self.inner.raw()
    }

    /// The reading scaled to `0.0..=1.0`.
    pub fn value(&self) -> Result<f64> {
        Ok(f64::from(self.inner.raw()?) / f64::from(self.inner.chip.max_raw()))
    }

    /// The reading in volts.
    pub fn voltage(&self) -> Result<f64> {
        Ok(self.value()? * self.inner.max_voltage)
    }

    /// The reference voltage.
    pub fn max_voltage(&self) -> f64 {
        self.inner.max_voltage
    }

    /// An endless iterator over `value()` (ends on a read error).
    pub fn values(&self) -> Values<f64> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || {
            let i = weak.upgrade()?;
            let raw = i.raw().ok()?;
            Some(f64::from(raw) / f64::from(i.chip.max_raw()))
        })
    }
}

macro_rules! mcp {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Debug, Clone, Copy)]
        pub struct $name;

        impl $name {
            #[doc = concat!("Opens `channel` of an ", stringify!($name), " on hardware SPI 0.0.")]
            #[allow(clippy::new_ret_no_self)]
            pub fn new(channel: u8) -> Result<McpAdc> {
                McpAdc::open(McpChip::$name, channel, McpOptions::default())
            }

            #[doc = concat!("Opens `channel` of an ", stringify!($name), " with explicit options.")]
            pub fn with_options(channel: u8, options: McpOptions) -> Result<McpAdc> {
                McpAdc::open(McpChip::$name, channel, options)
            }
        }
    };
}

mcp!(
    /// MCP3001: 10-bit, 1 channel.
    Mcp3001
);
mcp!(
    /// MCP3002: 10-bit, 2 channels.
    Mcp3002
);
mcp!(
    /// MCP3004: 10-bit, 4 channels.
    Mcp3004
);
mcp!(
    /// MCP3008: 10-bit, 8 channels.
    ///
    /// ```no_run
    /// use gpio_zero::{Mcp3008, PwmLed};
    /// let pot = Mcp3008::new(0)?;
    /// let led = PwmLed::new(17)?;
    /// led.set_source(pot.values()); // potentiometer controls brightness
    /// # Ok::<(), gpio_zero::Error>(())
    /// ```
    Mcp3008
);
mcp!(
    /// MCP3201: 12-bit, 1 channel.
    Mcp3201
);
mcp!(
    /// MCP3202: 12-bit, 2 channels.
    Mcp3202
);
mcp!(
    /// MCP3204: 12-bit, 4 channels.
    Mcp3204
);
mcp!(
    /// MCP3208: 12-bit, 8 channels.
    Mcp3208
);

/// A simulated MCP3xxx chip for the mock factory.
///
/// ```
/// use gpio_zero::pins::mock::MockFactory;
/// use gpio_zero::pins::SpiConfig;
/// use gpio_zero::{McpOptions, Mcp3008, MockMcp};
/// use std::sync::Arc;
///
/// let factory = MockFactory::new();
/// let chip = Arc::new(MockMcp::new(gpio_zero::McpChip::Mcp3008));
/// factory.attach_spi_device(&SpiConfig::hardware(0, 0), chip.clone());
/// chip.set_value(3, 0.5);
/// let adc = Mcp3008::with_options(3, McpOptions {
///     pin_factory: Some(factory.as_factory()),
///     ..Default::default()
/// }).unwrap();
/// assert!((adc.value().unwrap() - 0.5).abs() < 0.001);
/// ```
#[derive(Debug)]
pub struct MockMcp {
    chip: McpChip,
    values: Mutex<Vec<f64>>,
}

impl MockMcp {
    /// A chip with every channel at 0.
    pub fn new(chip: McpChip) -> Self {
        MockMcp {
            chip,
            values: Mutex::new(vec![0.0; usize::from(chip.channels())]),
        }
    }

    /// Sets channel `channel`'s input as a fraction of the reference voltage.
    pub fn set_value(&self, channel: u8, value: f64) {
        if let Some(v) = lock(&self.values).get_mut(usize::from(channel)) {
            *v = value.clamp(0.0, 1.0);
        }
    }

    fn reading(&self, channel: usize, single: bool) -> u32 {
        let v = lock(&self.values);
        let x = if single {
            v.get(channel).copied().unwrap_or(0.0)
        } else {
            let pos = v.get(channel).copied().unwrap_or(0.0);
            let neg = v.get(channel ^ 1).copied().unwrap_or(0.0);
            (pos - neg).max(0.0)
        };
        (x * f64::from(self.chip.max_raw())).round() as u32
    }
}

impl MockSpiDevice for MockMcp {
    fn respond(&self, mosi: &[bool]) -> Vec<bool> {
        let bits = self.chip.bits() as usize;
        let mut out = vec![false; mosi.len()];
        let (data_start, value) = match self.chip.protocol() {
            Protocol::Single => (3, self.reading(0, true)),
            proto => {
                let Some(s) = mosi.iter().position(|b| *b) else {
                    return out;
                };
                let cfg_len = if proto == Protocol::Two { 3 } else { 4 };
                if mosi.len() <= s + cfg_len {
                    return out;
                }
                let cfg = &mosi[s + 1..=s + cfg_len];
                let single = cfg[0];
                let channel = match proto {
                    Protocol::Two => usize::from(cfg[1]),
                    _ => usize::from(cfg[1]) << 2 | usize::from(cfg[2]) << 1 | usize::from(cfg[3]),
                };
                let start = s + cfg_len + if proto == Protocol::Two { 2 } else { 3 };
                (start, self.reading(channel, single))
            }
        };
        for i in 0..bits {
            if let Some(o) = out.get_mut(data_start + i) {
                *o = value >> (bits - 1 - i) & 1 == 1;
            }
        }
        out
    }
}
