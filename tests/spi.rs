mod common;

use std::sync::Arc;
use std::time::Duration;

use common::*;
use gpio_zero::prelude::*;
use gpio_zero::{McpChip, MockMcp};

fn opts(spi: SpiConfig, differential: bool) -> McpOptions {
    McpOptions {
        spi,
        differential,
        ..Default::default()
    }
}

#[test]
fn mcp3008_hardware() {
    let f = setup();
    let chip = Arc::new(MockMcp::new(McpChip::Mcp3008));
    f.attach_spi_device(&SpiConfig::hardware(0, 0), chip.clone());
    chip.set_value(0, 0.5);
    chip.set_value(7, 1.0);
    let a0 = Mcp3008::new(0).unwrap();
    let a7 = Mcp3008::new(7).unwrap();
    let a3 = Mcp3008::new(3).unwrap();
    assert_eq!(a0.raw_value().unwrap(), 512);
    assert_eq!(a7.raw_value().unwrap(), 1023);
    assert_approx(a7.value().unwrap(), 1.0);
    assert_approx(a7.voltage().unwrap(), 3.3);
    assert_eq!(a3.raw_value().unwrap(), 0);
    assert_eq!(a0.bits(), 10);
    assert!(Mcp3008::new(8).is_err());
}

#[test]
fn every_chip_round_trips() {
    for chip in [
        McpChip::Mcp3001,
        McpChip::Mcp3002,
        McpChip::Mcp3004,
        McpChip::Mcp3008,
        McpChip::Mcp3201,
        McpChip::Mcp3202,
        McpChip::Mcp3204,
        McpChip::Mcp3208,
    ] {
        let f = setup();
        let mock = Arc::new(MockMcp::new(chip));
        f.attach_spi_device(&SpiConfig::hardware(0, 1), mock.clone());
        let max = (1u32 << chip.bits()) - 1;
        for ch in 0..chip.channels() {
            let v = (f64::from(ch) + 1.0) / 9.0;
            mock.set_value(ch, v);
            let adc = McpAdc::open(chip, ch, opts(SpiConfig::hardware(0, 1), false)).unwrap();
            let expected = (v * f64::from(max)).round() as u32;
            assert_eq!(adc.raw_value().unwrap(), expected, "{chip:?} ch{ch}");
        }
    }
}

#[test]
fn differential_mode() {
    let f = setup();
    let chip = Arc::new(MockMcp::new(McpChip::Mcp3208));
    f.attach_spi_device(&SpiConfig::hardware(0, 0), chip.clone());
    chip.set_value(0, 0.75);
    chip.set_value(1, 0.25);
    let d0 = Mcp3208::with_options(0, opts(SpiConfig::hardware(0, 0), true)).unwrap();
    let d1 = Mcp3208::with_options(1, opts(SpiConfig::hardware(0, 0), true)).unwrap();
    assert!((d0.value().unwrap() - 0.5).abs() < 0.001);
    assert_eq!(d1.raw_value().unwrap(), 0);
    assert!(d0.differential());
    assert!(McpAdc::open(McpChip::Mcp3001, 0, opts(SpiConfig::hardware(0, 0), true)).is_err());
}

#[test]
fn software_spi_bit_banging() {
    let f = setup();
    let cfg = SpiConfig::software(11, 10, 9, 8);
    let chip = Arc::new(MockMcp::new(McpChip::Mcp3208));
    f.attach_spi_device(&cfg, chip.clone());
    chip.set_value(5, 0.3);
    let adc = Mcp3208::with_options(5, opts(cfg.clone(), false)).unwrap();
    assert_eq!(adc.raw_value().unwrap(), (0.3f64 * 4095.0).round() as u32);
    // The pins are reserved by the software SPI.
    assert!(Led::new(11).is_err());
    // Chip select idles high.
    assert_approx(f.mock_pin(8).value(), 1.0);

    let chip2 = Arc::new(MockMcp::new(McpChip::Mcp3002));
    let cfg2 = SpiConfig::software(21, 20, 19, 18);
    f.attach_spi_device(&cfg2, chip2.clone());
    chip2.set_value(1, 1.0);
    let adc2 = Mcp3002::with_options(1, opts(cfg2, false)).unwrap();
    assert_eq!(adc2.raw_value().unwrap(), 1023);
}

#[test]
fn missing_device_is_an_error() {
    let _f = setup();
    assert!(Mcp3008::new(0).is_err());
}

#[test]
fn potentiometer_controls_led() {
    let f = setup();
    let chip = Arc::new(MockMcp::new(McpChip::Mcp3008));
    f.attach_spi_device(&SpiConfig::hardware(0, 0), chip.clone());
    let pot = Mcp3008::new(0).unwrap();
    let led = PwmLed::new(17).unwrap();
    led.set_source(pot.values());
    chip.set_value(0, 1.0);
    f.advance(Duration::from_millis(20));
    assert_approx(led.value(), 1.0);
    chip.set_value(0, 0.0);
    f.advance(Duration::from_millis(20));
    assert_approx(led.value(), 0.0);
}
