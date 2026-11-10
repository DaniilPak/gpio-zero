mod common;

use std::time::Duration;

use common::*;
use gpio_zero::prelude::*;
use gpio_zero::{Error, OutputOptions, PwmOptions};

const S: Duration = Duration::from_secs(1);

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

#[test]
fn led_on_off_toggle() {
    let f = setup();
    let led = Led::new(17).unwrap();
    assert!(!led.is_lit());
    led.on().unwrap();
    assert!(led.is_lit());
    assert!(led.value());
    led.off().unwrap();
    led.toggle().unwrap();
    assert!(led.is_active());
    assert_states(&f.mock_pin(17).states(), &[0.0, 1.0, 0.0, 1.0]);
    assert_eq!(led.pin(), 17);
}

#[test]
fn active_low_inverts_pin() {
    let f = setup();
    let led = Led::with_options(
        4,
        OutputOptions {
            active_high: false,
            ..Default::default()
        },
    )
    .unwrap();
    assert_approx(f.mock_pin(4).value(), 1.0);
    led.on().unwrap();
    assert_approx(f.mock_pin(4).value(), 0.0);
    assert!(led.is_lit());
    assert!(!led.active_high());
}

#[test]
fn initial_value_none_leaves_pin() {
    let f = setup();
    let _d = OutputDevice::with_options(
        5,
        OutputOptions {
            initial_value: None,
            ..Default::default()
        },
    )
    .unwrap();
    assert_states(&f.mock_pin(5).states(), &[0.0]);
    let d2 = DigitalOutputDevice::with_options(
        6,
        OutputOptions {
            initial_value: Some(true),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(d2.value());
}

#[test]
fn pin_reservation() {
    let _f = setup();
    let led = Led::new(17).unwrap();
    assert!(matches!(Led::new(17), Err(Error::PinInUse(17))));
    drop(led);
    Led::new(17).unwrap();
}

#[test]
fn clones_share_the_device() {
    let f = setup();
    let led = Led::new(2).unwrap();
    let other = led.clone();
    drop(led);
    other.on().unwrap();
    assert_approx(f.mock_pin(2).value(), 1.0);
    assert!(Led::new(2).is_err());
    drop(other);
    Led::new(2).unwrap();
}

#[test]
fn led_blink_n_times() {
    let f = setup();
    let led = Led::new(17).unwrap();
    led.blink(S, S, Some(2), true).unwrap();
    f.advance(Duration::from_secs(5));
    let h = f.mock_pin(17).history();
    let states: Vec<f64> = h.iter().map(|x| x.1).collect();
    let times: Vec<Duration> = h.iter().map(|x| x.0).collect();
    assert_states(&states, &[0.0, 1.0, 0.0, 1.0, 0.0]);
    assert_eq!(times, vec![Duration::ZERO, Duration::ZERO, S, 2 * S, 3 * S]);
    assert!(!led.is_lit());
}

#[test]
fn blink_is_stopped_by_writes() {
    let f = setup();
    let led = Led::new(17).unwrap();
    led.blink(S, S, None, true).unwrap();
    f.advance(ms(1500));
    assert!(!led.is_lit());
    led.on().unwrap();
    f.advance(Duration::from_secs(10));
    assert!(led.is_lit());
    assert_eq!(f.mock_clock().unwrap().live_tasks(), 0);
}

#[test]
fn dropping_a_blinking_led_stops_its_thread() {
    let f = setup();
    let led = Led::new(17).unwrap();
    led.blink(S, S, None, true).unwrap();
    f.advance(ms(100));
    assert_eq!(f.mock_clock().unwrap().live_tasks(), 1);
    drop(led);
    f.settle();
    assert_eq!(f.mock_clock().unwrap().live_tasks(), 0);
}

#[test]
fn blink_in_foreground_with_real_time() {
    let f = MockFactory::with_real_time();
    let led = Led::with_options(
        17,
        OutputOptions {
            pin_factory: Some(f.as_factory()),
            ..Default::default()
        },
    )
    .unwrap();
    led.blink(ms(5), ms(5), Some(2), false).unwrap();
    assert_states(&f.mock_pin(17).states(), &[0.0, 1.0, 0.0, 1.0, 0.0]);
}

#[test]
fn digital_led_rejects_fades() {
    let _f = setup();
    let led = Led::new(17).unwrap();
    assert!(led.blink_with(Blink::new(S, S).fade(S, S)).is_err());
}

#[test]
fn buzzer_beeps() {
    let f = setup();
    let b = Buzzer::new(3).unwrap();
    b.beep(ms(100), ms(200), Some(1), true).unwrap();
    f.advance(S);
    assert_states(&f.mock_pin(3).states(), &[0.0, 1.0, 0.0]);
}

#[test]
fn pwm_led_values() {
    let f = setup();
    let led = PwmLed::new(18).unwrap();
    assert_approx(led.frequency(), 100.0);
    led.set_value(0.25).unwrap();
    assert_approx(f.mock_pin(18).value(), 0.25);
    led.toggle().unwrap();
    assert_approx(led.value(), 0.75);
    assert!(led.is_lit());
    assert!(matches!(led.set_value(1.5), Err(Error::InvalidValue(_))));
    led.off().unwrap();
    assert!(!led.is_lit());
    led.set_frequency(1000.0).unwrap();
    assert_approx(led.frequency(), 1000.0);
}

#[test]
fn pwm_active_low() {
    let f = setup();
    let led = PwmOutputDevice::with_options(
        18,
        PwmOptions {
            active_high: false,
            initial_value: 0.2,
            ..Default::default()
        },
    )
    .unwrap();
    assert_approx(f.mock_pin(18).value(), 0.8);
    assert_approx(led.value(), 0.2);
}

#[test]
fn pwm_needs_capable_pin() {
    let f = setup();
    f.set_pwm_capable(false);
    assert!(matches!(PwmLed::new(18), Err(Error::PwmUnsupported(18))));
    // The failed device released its reservation.
    f.set_pwm_capable(true);
    Led::new(18).unwrap();
}

#[test]
fn pwm_pulse_fades() {
    let f = setup();
    let led = PwmLed::new(18).unwrap();
    led.pulse(ms(200), ms(200), Some(1), true).unwrap();
    f.advance(S);
    assert_states(
        &f.mock_pin(18).states(),
        &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 0.8, 0.6, 0.4, 0.2, 0.0],
    );
}

#[test]
fn pwm_blink_with_fades_timing() {
    let f = setup();
    let led = PwmLed::new(18).unwrap();
    led.blink_with(Blink::new(S, S).fade(ms(80), Duration::ZERO).times(1))
        .unwrap();
    f.advance(ms(40));
    assert_approx(led.value(), 0.5);
    f.advance(ms(500));
    assert_approx(led.value(), 1.0);
    f.advance(S);
    assert_approx(led.value(), 0.0);
}

#[test]
fn source_values_link_devices() {
    let f = setup();
    let led = Led::new(17).unwrap();
    let button = Button::new(2).unwrap();
    led.set_source(button.values());
    f.advance(ms(20));
    assert!(!led.is_lit());
    f.mock_pin(2).drive_low();
    f.advance(ms(20));
    assert!(led.is_lit());
    assert!(led.has_source());
    led.clear_source();
    assert!(!led.has_source());
    f.mock_pin(2).drive_high();
    f.advance(ms(20));
    assert!(led.is_lit());
}

#[test]
fn finite_source_and_delay() {
    let f = setup();
    let led = PwmLed::new(18).unwrap();
    led.set_source_delay(ms(100));
    led.set_source(vec![0.1, 0.5, 1.0]);
    f.settle();
    assert_approx(led.value(), 0.1);
    f.advance(ms(100));
    assert_approx(led.value(), 0.5);
    f.advance(ms(500));
    assert_approx(led.value(), 1.0);
    assert!(!led.has_source());
}

#[test]
fn bool_source_drives_pwm() {
    let f = setup();
    let led = PwmLed::new(18).unwrap();
    led.set_source(std::iter::once(true));
    f.advance(ms(20));
    assert_approx(led.value(), 1.0);
}

#[test]
fn values_end_when_device_dropped() {
    let _f = setup();
    let led = Led::new(17).unwrap();
    let mut values = led.values();
    assert_eq!(values.next(), Some(false));
    led.on().unwrap();
    assert_eq!(values.next(), Some(true));
    drop(led);
    assert_eq!(values.next(), None);
}

#[test]
fn rgb_led() {
    let f = setup();
    let rgb = RgbLed::new(9, 10, 11).unwrap();
    assert_eq!(rgb.value(), Color::BLACK);
    rgb.set_color(Color::new(1.0, 0.5, 0.0)).unwrap();
    assert_approx(f.mock_pin(10).value(), 0.5);
    assert_approx(rgb.green(), 0.5);
    rgb.set_blue(0.25).unwrap();
    assert_eq!(rgb.value(), Color::new(1.0, 0.5, 0.25));
    rgb.toggle().unwrap();
    assert_eq!(rgb.value(), Color::new(0.0, 0.5, 0.75));
    rgb.on().unwrap();
    assert_eq!(rgb.value(), Color::WHITE);
    assert!(rgb.is_lit());
    rgb.off().unwrap();
    assert!(!rgb.is_lit());
    assert!(rgb.set_value((2.0, 0.0, 0.0)).is_err());
    assert_eq!(rgb.pins(), [9, 10, 11]);
}

#[test]
fn rgb_blink_colours() {
    let f = setup();
    let rgb = RgbLed::new(9, 10, 11).unwrap();
    rgb.blink(S, S, Color::RED, Color::BLUE, Some(1), true)
        .unwrap();
    f.advance(ms(500));
    assert_eq!(rgb.value(), Color::RED);
    f.advance(S);
    assert_eq!(rgb.value(), Color::BLUE);
}

#[test]
fn rgb_without_pwm() {
    let _f = setup();
    let rgb = RgbLed::with_options(
        9,
        10,
        11,
        RgbLedOptions {
            pwm: false,
            ..Default::default()
        },
    )
    .unwrap();
    rgb.set_value(Color::YELLOW).unwrap();
    assert!(rgb.set_value((0.5, 0.0, 0.0)).is_err());
    assert!(
        rgb.pulse(S, S, Color::WHITE, Color::BLACK, Some(1), true)
            .is_err()
    );
}

#[test]
fn colour_parsing() {
    assert_eq!("#ff0000".parse::<Color>().unwrap(), Color::RED);
    assert_eq!("cyan".parse::<Color>().unwrap(), Color::CYAN);
    let c: Color = "336699".parse().unwrap();
    assert_approx(c.red, 0.2);
    assert!("#12".parse::<Color>().is_err());
}

#[test]
fn motor_directions() {
    let f = setup();
    let m = Motor::with_options(
        4,
        14,
        MotorOptions {
            enable: Some(15),
            ..Default::default()
        },
    )
    .unwrap();
    assert_approx(f.mock_pin(15).value(), 1.0);
    m.forward(0.5).unwrap();
    assert_approx(f.mock_pin(4).value(), 0.5);
    assert_approx(f.mock_pin(14).value(), 0.0);
    assert_approx(m.value(), 0.5);
    m.reverse().unwrap();
    assert_approx(m.value(), -0.5);
    assert_approx(f.mock_pin(14).value(), 0.5);
    m.backward(1.0).unwrap();
    assert_approx(m.value(), -1.0);
    assert!(m.is_active());
    m.stop().unwrap();
    assert!(!m.is_active());
    assert!(m.set_value(1.5).is_err());
    assert!(m.forward(-0.1).is_err());
}

#[test]
fn motor_without_pwm() {
    let _f = setup();
    let m = Motor::with_options(
        4,
        14,
        MotorOptions {
            pwm: false,
            ..Default::default()
        },
    )
    .unwrap();
    m.forward(1.0).unwrap();
    assert!(m.forward(0.5).is_err());
}

#[test]
fn phase_enable_motor() {
    let f = setup();
    let m = PhaseEnableMotor::new(5, 12).unwrap();
    m.forward(0.3).unwrap();
    assert_approx(f.mock_pin(5).value(), 0.0);
    assert_approx(f.mock_pin(12).value(), 0.3);
    m.backward(0.6).unwrap();
    assert_approx(f.mock_pin(5).value(), 1.0);
    assert_approx(m.value(), -0.6);
    m.reverse().unwrap();
    assert_approx(m.value(), 0.6);
    m.stop().unwrap();
    assert!(!m.is_active());
}

#[test]
fn motor_source() {
    let f = setup();
    let m = Motor::new(4, 14).unwrap();
    m.set_source(gpio_zero::tools::scaled_full(vec![0.0, 1.0]));
    f.settle();
    assert_approx(m.value(), -1.0);
    f.advance(ms(10));
    assert_approx(m.value(), 1.0);
}

#[test]
fn servo_positions() {
    let f = setup();
    let s = Servo::new(17).unwrap();
    let pin = f.mock_pin(17);
    assert_eq!(pin.frequency(), Some(50.0));
    assert_approx(pin.value(), 0.075);
    assert_eq!(s.value(), Some(0.0));
    assert_eq!(s.pulse_width(), Some(Duration::from_micros(1500)));
    s.min().unwrap();
    assert_approx(pin.value(), 0.05);
    s.max().unwrap();
    assert_approx(pin.value(), 0.1);
    s.set_value(0.5).unwrap();
    assert_eq!(s.value(), Some(0.5));
    s.detach().unwrap();
    assert_eq!(s.value(), None);
    assert!(!s.is_active());
    assert_approx(pin.value(), 0.0);
    assert!(s.set_value(1.1).is_err());
    s.mid().unwrap();
    assert_eq!(s.value(), Some(0.0));
}

#[test]
fn servo_bad_options() {
    let _f = setup();
    let o = ServoOptions {
        min_pulse_width: Duration::from_millis(2),
        max_pulse_width: Duration::from_millis(1),
        ..Default::default()
    };
    assert!(Servo::with_options(17, o).is_err());
}

#[test]
fn angular_servo() {
    let f = setup();
    let s = AngularServo::new(17).unwrap();
    assert_eq!(s.angle(), Some(0.0));
    s.set_angle(45.0).unwrap();
    assert_eq!(s.value(), Some(0.5));
    assert_eq!(s.angle(), Some(45.0));
    s.set_angle(-90.0).unwrap();
    assert_approx(f.mock_pin(17).value(), 0.05);
    assert!(s.set_angle(100.0).is_err());
    s.set_angle(None).unwrap();
    assert_eq!(s.angle(), None);

    let inv = AngularServo::with_options(
        18,
        AngularServoOptions {
            min_angle: 90.0,
            max_angle: -90.0,
            ..Default::default()
        },
    )
    .unwrap();
    inv.set_angle(90.0).unwrap();
    assert_eq!(inv.value(), Some(-1.0));
}

#[test]
fn tonal_buzzer() {
    let f = setup();
    let b = TonalBuzzer::new(20).unwrap();
    assert!(!b.is_active());
    assert_approx(b.min_tone().frequency(), 220.0);
    assert_approx(b.max_tone().frequency(), 880.0);
    b.play("A4".parse::<Tone>().unwrap()).unwrap();
    let pin = f.mock_pin(20);
    assert_eq!(pin.frequency(), Some(440.0));
    assert_approx(pin.value(), 0.5);
    assert_eq!(b.value(), Some(0.0));
    b.play(880.0).unwrap();
    assert_approx(b.value().unwrap(), 1.0);
    b.set_value(-1.0).unwrap();
    assert_approx(b.tone().unwrap().frequency(), 220.0);
    assert!(b.play(2000.0).is_err());
    b.stop().unwrap();
    assert_eq!(b.value(), None);
    assert_eq!(pin.frequency(), None);
    assert_approx(pin.value(), 0.0);
}

#[test]
fn tones() {
    assert_eq!(Tone::from_note("C#5").unwrap().midi(), Some(73));
    assert_eq!(Tone::from_note("Bb3").unwrap().midi(), Some(58));
    assert_eq!(Tone::from_midi(60).note().unwrap(), "C4");
    assert_eq!(Tone::from_midi(69).up(12).frequency(), 880.0);
    assert_eq!(Tone::from_midi(69).down(12).midi(), Some(57));
    assert_eq!(Tone::from_frequency(450.0).note(), None);
    assert_eq!(Tone::from_midi(61).to_string(), "C#4");
    assert!(Tone::from_note("H2").is_err());
    assert!(Tone::from_note("A").is_err());
}
