# gpio-zero

[![CI](https://github.com/DaniilPak/gpio-zero/actions/workflows/ci.yml/badge.svg)](https://github.com/DaniilPak/gpio-zero/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/gpio-zero.svg)](https://crates.io/crates/gpio-zero)
[![docs.rs](https://docs.rs/gpio-zero/badge.svg)](https://docs.rs/gpio-zero)

A simple, high-level device API for the Raspberry Pi and other Linux GPIO
boards: LEDs, buttons, buzzers, motors, servos, sensors, ADCs and robots.
It is modelled on Python's [gpiozero](https://gpiozero.readthedocs.io/).

> **Independent project.** gpio-zero is not affiliated with or endorsed by the
> gpiozero authors. It reuses gpiozero's device model and API ideas (with
> credit; see [Credits](#credits)) and replaces the unmaintained
> `rust_gpiozero` crate, which has had no release since 2021.

```toml
[dependencies]
gpio-zero = "0.1"
```

- **Works on every Pi, including the Pi 5.** The default backend uses the Linux
  GPIO character device (`/dev/gpiochipN`). It finds lines by name (`GPIO17`),
  so the RP1 chip on the Pi 5 works with no extra setup.
- **The same API as gpiozero:** `Led::new(17)?.blink(...)`,
  `button.when_pressed(...)`, `led.set_source(button.values())`.
- **Testable without hardware.** `MockFactory` simulates pins, SPI chips,
  HC-SR04 echoes and RC charging circuits on a **virtual clock**. You can test
  timing behaviour such as blinking, button holds and debouncing with exact
  timings, without sleeping.

## Recipes

### Button and LED

```rust,no_run
use gpio_zero::{Button, Led};

let led = Led::new(17)?;
let button = Button::new(2)?;           // between GPIO2 and GND (internal pull-up)

let l = led.clone();
button.when_pressed(move || { let _ = l.on(); });
let l = led.clone();
button.when_released(move || { let _ = l.off(); });

button.wait_for_press(None);
# Ok::<(), gpio_zero::Error>(())
```

You can also link the two devices with the source/values pattern:

```rust,no_run
# use gpio_zero::{Button, Led};
# let led = Led::new(17)?; let button = Button::new(2)?;
led.set_source(button.values());                        // led.source = button.values
led.set_source(gpio_zero::tools::negated(button.values())); // inverted
# Ok::<(), gpio_zero::Error>(())
```

`values()` returns an ordinary endless `Iterator`. You can apply `map`, `zip`
and `take` to it, or use the gpiozero-style helpers in `gpio_zero::tools`.
`set_source` runs the loop in a background thread that stops when the device
is dropped.

### Blink, pulse and hold

```rust,no_run
use std::time::Duration;
use gpio_zero::{Blink, Button, PwmLed};

let led = PwmLed::new(18)?;
led.pulse(Duration::from_secs(1), Duration::from_secs(1), None, true)?;   // breathe forever
led.blink_with(Blink::new(Duration::from_millis(200), Duration::from_millis(800)).times(3))?;

let button = Button::new(2)?;
button.when_held(|| println!("held for a second"));
# Ok::<(), gpio_zero::Error>(())
```

### Traffic lights

```rust,no_run
use std::{thread::sleep, time::Duration};
use gpio_zero::TrafficLights;

let lights = TrafficLights::new(2, 3, 4)?;
loop {
    lights.green().on()?;
    sleep(Duration::from_secs(3));
    lights.green().off()?;
    lights.amber().on()?;
    sleep(Duration::from_secs(1));
    lights.amber().off()?;
    lights.red().on()?;
    sleep(Duration::from_secs(3));
    lights.amber().on()?;
    sleep(Duration::from_secs(1));
    lights.off()?;
}
# Ok::<(), gpio_zero::Error>(())
```

### Robot

```rust,no_run
use std::{thread::sleep, time::Duration};
use gpio_zero::Robot;

let robot = Robot::new((4, 14), (17, 18))?;   // (forward, backward) pins per side
for _ in 0..4 {
    robot.forward(0.8)?;
    sleep(Duration::from_secs(2));
    robot.right(0.5)?;
    sleep(Duration::from_millis(500));
}
robot.stop()?;
# Ok::<(), gpio_zero::Error>(())
```

### Potentiometer dims an LED (MCP3008 over SPI)

```rust,no_run
use gpio_zero::{Mcp3008, PwmLed};

let pot = Mcp3008::new(0)?;      // /dev/spidev0.0, channel 0
let led = PwmLed::new(17)?;
led.set_source(pot.values());
# Ok::<(), gpio_zero::Error>(())
```

### Distance sensor

```rust,no_run
use gpio_zero::{DistanceSensor, Led};

let sensor = DistanceSensor::new(24, 23)?;   // echo, trigger
let led = Led::new(16)?;
let l = led.clone();
sensor.when_in_range(move || { let _ = l.on(); });
let l = led.clone();
sensor.when_out_of_range(move || { let _ = l.off(); });
println!("{:.2} m", sensor.distance());
# Ok::<(), gpio_zero::Error>(())
```

## Testing with the mock backend

```rust
use std::time::Duration;
use gpio_zero::pins::mock::MockFactory;
use gpio_zero::{Button, Led};

let factory = MockFactory::new();       // virtual time starts at 0
factory.install_for_thread();           // devices on this thread use it

let led = Led::new(17).unwrap();
let button = Button::new(2).unwrap();
led.set_source(button.values());

factory.mock_pin(2).drive_low();        // press
factory.advance(Duration::from_millis(20));
assert!(led.is_lit());

led.clear_source();
led.blink(Duration::from_secs(1), Duration::from_secs(1), Some(2), true).unwrap();
factory.advance(Duration::from_secs(5));
assert_eq!(factory.mock_pin(17).states(), vec![0.0, 1.0, 0.0, 1.0, 0.0]);
```

`MockFactory::advance` wakes every pending timer in order. After each step it
waits until all background threads are idle again, so an assertion made after
`advance` sees a settled system. Other mock helpers are
`charging_pin` (for `LightSensor`), `trigger_pin` (an HC-SR04 echo at a set
distance), `attach_spi_device` with `MockMcp` (MCP3xxx chips, on hardware or
bit-banged SPI) and `with_real_time()`.

## Backends

| Backend | Feature | Hardware | Notes |
|---|---|---|---|
| `cdev` | `cdev` (default) | Any Linux board with `/dev/gpiochipN`, every Pi including the Pi 5 | uAPI v2 via [`gpiocdev`]; kernel debounce and edge timestamps; software PWM; hardware SPI via `/dev/spidevX.Y` |
| `rppal` | `rppal` | Raspberry Pi 0–5 | [`rppal`] GPIO with its software PWM, interrupts and hardware SPI |
| mock | always | none | Virtual clock, simulated pins, SPI and sensors |

The backend is chosen in this order: the `pin_factory` field of a device's
options, then `pins::set_thread_factory`, then `pins::set_default_factory`,
then the `GPIOZERO_PIN_FACTORY` environment variable (`cdev`, `rppal` or
`mock`), and finally the first compiled backend that works. Any backend can
run software (bit-banged) SPI with `SpiConfig::software(clock, mosi, miso, select)`.

The crate builds on any OS, but the hardware backends are compiled only on
Linux, so on macOS or Windows only the mock backend is available. CI
cross-checks the crate for `aarch64-unknown-linux-gnu` and
`armv7-unknown-linux-gnueabihf`.

## Device coverage vs gpiozero

| gpiozero | gpio-zero | Status |
|---|---|---|
| `LED` | `Led` | ✅ on/off/toggle/blink |
| `PWMLED` | `PwmLed` | ✅ blink with fades, pulse |
| `RGBLED` | `RgbLed` | ✅ PWM or digital, colour blink and pulse, `Color` parsing |
| `Buzzer` | `Buzzer` | ✅ beep |
| `TonalBuzzer` | `TonalBuzzer` + `Tone` | ✅ notes (`"A4"`, MIDI, Hz), octave range |
| `Motor` | `Motor` | ✅ optional enable pin, PWM or digital |
| `PhaseEnableMotor` | `PhaseEnableMotor` | ✅ |
| `Servo` | `Servo` | ✅ min/mid/max/detach, pulse widths |
| `AngularServo` | `AngularServo` | ✅ inverted ranges |
| `OutputDevice` / `DigitalOutputDevice` / `PWMOutputDevice` | same names (`PwmOutputDevice`) | ✅ |
| `Button` | `Button` | ✅ debounce, `when_pressed/released/held`, `hold_repeat`, `wait_for_press` |
| `MotionSensor` | `MotionSensor` | ✅ smoothed queue |
| `LineSensor` | `LineSensor` | ✅ |
| `LightSensor` | `LightSensor` | ✅ RC charge timing |
| `DistanceSensor` | `DistanceSensor` | ✅ HC-SR04, in/out-of-range events |
| `RotaryEncoder` | `RotaryEncoder` | ✅ gpiozero's quadrature state machine, wrap/clamp (no `threshold_steps` yet) |
| `InputDevice` / `DigitalInputDevice` | same names | ✅ |
| `SmoothedInputDevice` | internal (used by the sensors) | ➖ |
| `MCP3001/3002/3004/3008/3201/3202/3204/3208` | `Mcp3001` … `Mcp3208` | ✅ single-ended and differential |
| `MCP3301/3302/3304` | — | ❌ planned |
| `LEDBoard` | `LedBoard` | ✅ indices, blink, pulse (no nested/named boards yet) |
| `LEDBarGraph` | `LedBarGraph` | ✅ PWM partial LEDs, negative values |
| `TrafficLights` | `TrafficLights` | ✅ |
| `ButtonBoard` | `ButtonBoard` | ✅ |
| `Robot` | `Robot` | ✅ curves, spins, source |
| `PhaseEnableRobot`, `LEDCharDisplay`, `TrafficLightsBuzzer`, HAT presets (`TrafficHat`, `PiHutXmasTree`, …), `Energenie` | — | ❌ planned |
| `CPUTemperature`, `LoadAverage`, `DiskUsage`, `TimeOfDay`, `PingServer` | — | ❌ (std/sysinfo cover these) |
| `gpiozero.tools` | `gpio_zero::tools` | ✅ common subset (`negated`, `scaled`, `smoothed`, `all_values`, `sin_values`, …) |
| Pin factories (lgpio, RPi.GPIO, pigpio, native, mock) | `cdev`, `rppal`, `MockFactory` | ✅ |
| `pinout` / `pi_info` | — | ❌ |

## Differences from gpiozero

- Times are `std::time::Duration` values, not floats in seconds.
- Optional arguments are passed in `XxxOptions` structs with `Default`, for
  example `Led::with_options(17, OutputOptions { active_high: false, ..Default::default() })`.
- Callbacks are closures that take no arguments. To use a device inside a
  callback, clone the device handle into the closure.
- Setters return `Result`. Invalid values are errors, not exceptions.
- Instead of a `source` property there are `set_source(iter)` and
  `clear_source()`, and `values()` returns a `Values<T>` iterator.

## Credits

This crate builds on [gpiozero](https://github.com/gpiozero/gpiozero) by Ben
Nuttall, Dave Jones and contributors (BSD-3-Clause). The device API and
default parameters, the rotary-encoder transition table, the blink and pulse
sequences and the MCP3xxx bit layouts are ported from it. gpiozero's
copyright and license notice are kept in [NOTICE](NOTICE). The hardware
access comes from [`gpiocdev`], [`rppal`] and
[`spidev`](https://crates.io/crates/spidev).

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT), at your option. The portions ported from gpiozero
remain under BSD-3-Clause as well (see [NOTICE](NOTICE)).

[`gpiocdev`]: https://crates.io/crates/gpiocdev
[`rppal`]: https://crates.io/crates/rppal
