mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use common::*;
use gpio_zero::prelude::*;
use gpio_zero::{ButtonOptions, Error, InputOptions};

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

#[test]
fn button_press_and_release() {
    let f = setup();
    let b = Button::new(2).unwrap();
    let pin = f.mock_pin(2);
    assert!(b.pull_up());
    assert_approx(pin.value(), 1.0);
    assert!(!b.is_pressed());
    let (pressed, released) = (Counter::new(), Counter::new());
    b.when_pressed(pressed.hit());
    b.when_released(released.hit());
    pin.drive_low();
    assert!(b.is_pressed());
    assert!(b.value());
    pin.drive_low();
    pin.drive_high();
    pin.drive_low();
    pin.drive_high();
    assert_eq!(pressed.get(), 2);
    assert_eq!(released.get(), 2);
    b.clear_callbacks();
    pin.drive_low();
    assert_eq!(pressed.get(), 2);
}

#[test]
fn button_initial_state_pressed() {
    let f = setup();
    f.mock_pin(3).drive_low();
    let b = Button::new(3).unwrap();
    assert!(b.is_pressed());
    assert!(b.wait_for_press(Some(Duration::ZERO)));
}

#[test]
fn button_hold() {
    let f = setup();
    let b = Button::new(2).unwrap();
    let held = Counter::new();
    b.when_held(held.hit());
    let pin = f.mock_pin(2);
    pin.drive_low();
    f.advance(ms(900));
    assert!(!b.is_held());
    assert_eq!(held.get(), 0);
    f.advance(ms(200));
    assert!(b.is_held());
    assert_eq!(held.get(), 1);
    assert_eq!(b.held_time(), Some(ms(100)));
    assert_eq!(b.active_time(), Some(ms(1100)));
    f.advance(Duration::from_secs(5));
    assert_eq!(held.get(), 1);
    pin.drive_high();
    assert!(!b.is_held());
    assert_eq!(b.held_time(), None);
    assert_eq!(b.inactive_time(), Some(Duration::ZERO));
}

#[test]
fn button_short_press_is_not_held() {
    let f = setup();
    let b = Button::new(2).unwrap();
    let held = Counter::new();
    b.when_held(held.hit());
    f.mock_pin(2).drive_low();
    f.advance(ms(500));
    f.mock_pin(2).drive_high();
    f.advance(Duration::from_secs(2));
    assert_eq!(held.get(), 0);
    assert_eq!(f.mock_clock().unwrap().live_tasks(), 0);
}

#[test]
fn button_hold_repeat() {
    let f = setup();
    let b = Button::with_options(
        2,
        ButtonOptions {
            hold_time: ms(500),
            hold_repeat: true,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(b.hold_repeat());
    assert_eq!(b.hold_time(), ms(500));
    let held = Counter::new();
    b.when_held(held.hit());
    f.mock_pin(2).drive_low();
    f.advance(ms(1750));
    assert_eq!(held.get(), 3);
}

#[test]
fn button_debounce() {
    let f = setup();
    let b = Button::with_options(
        2,
        ButtonOptions {
            bounce_time: Some(ms(10)),
            ..Default::default()
        },
    )
    .unwrap();
    let pressed = Counter::new();
    b.when_pressed(pressed.hit());
    let pin = f.mock_pin(2);
    pin.drive_low();
    f.advance(ms(1));
    pin.drive_high();
    pin.drive_low();
    assert_eq!(pressed.get(), 1);
    assert!(b.is_pressed());
    f.advance(ms(20));
    pin.drive_high();
    assert!(!b.is_pressed());
}

#[test]
fn wait_for_press_real_time() {
    let f = MockFactory::with_real_time();
    let b = Button::with_options(
        2,
        ButtonOptions {
            pin_factory: Some(f.as_factory()),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(!b.wait_for_press(Some(ms(10))));
    let pin = f.mock_pin(2);
    let t = std::thread::spawn(move || {
        std::thread::sleep(ms(20));
        pin.drive_low();
    });
    assert!(b.wait_for_press(Some(Duration::from_secs(5))));
    t.join().unwrap();
    assert!(b.is_pressed());
    assert!(!b.wait_for_release(Some(ms(5))));
}

#[test]
fn wait_with_virtual_timeout() {
    let f = setup();
    let b = Button::new(2).unwrap();
    let b2 = b.clone();
    let t = std::thread::spawn(move || b2.wait_for_press(Some(Duration::from_secs(1))));
    // Give the waiter time to register, then let virtual time pass.
    std::thread::sleep(ms(20));
    f.advance(Duration::from_secs(2));
    assert!(!t.join().unwrap());
}

#[test]
fn pull_configuration_errors() {
    let _f = setup();
    let r = InputDevice::with_options(
        4,
        InputOptions {
            pull_up: None,
            ..Default::default()
        },
    );
    assert!(matches!(r, Err(Error::InvalidValue(_))));
    let r = InputDevice::with_options(
        4,
        InputOptions {
            pull_up: Some(true),
            active_state: Some(true),
            ..Default::default()
        },
    );
    assert!(r.is_err());
}

#[test]
fn floating_input_with_active_state() {
    let f = setup();
    let d = DigitalInputDevice::with_options(
        4,
        InputOptions {
            pull_up: None,
            active_state: Some(false),
            ..Default::default()
        },
    )
    .unwrap();
    assert!(!d.pull_up());
    // Floating reads low; active_state = low means active.
    assert!(d.is_active());
    let deact = Counter::new();
    d.when_deactivated(deact.hit());
    f.mock_pin(4).drive_high();
    assert!(!d.is_active());
    assert_eq!(deact.get(), 1);
}

#[test]
fn digital_input_device_events() {
    let f = setup();
    let d = DigitalInputDevice::new(4).unwrap();
    let act = Counter::new();
    d.when_activated(act.hit());
    f.advance(ms(50));
    f.mock_pin(4).drive_high();
    assert_eq!(act.get(), 1);
    f.advance(ms(30));
    assert_eq!(d.active_time(), Some(ms(30)));
    assert!(d.wait_for_active(None));
    let v: Vec<bool> = d.values().take(2).collect();
    assert_eq!(v, vec![true, true]);
    let plain = InputDevice::new(5).unwrap();
    assert!(!plain.is_active());
    f.mock_pin(5).drive_high();
    assert!(plain.value());
}

#[test]
fn motion_sensor() {
    let f = setup();
    let m = MotionSensor::new(4).unwrap();
    let (on, off) = (Counter::new(), Counter::new());
    m.when_motion(on.hit());
    m.when_no_motion(off.hit());
    f.advance(ms(200));
    assert!(!m.motion_detected());
    f.mock_pin(4).drive_high();
    f.advance(ms(100));
    assert!(m.motion_detected());
    assert_approx(m.value(), 1.0);
    assert_eq!(on.get(), 1);
    f.mock_pin(4).drive_low();
    f.advance(ms(100));
    assert!(!m.motion_detected());
    assert_eq!(off.get(), 1);
}

#[test]
fn line_sensor_smooths() {
    let f = setup();
    let l = LineSensor::new(4).unwrap();
    let line = Counter::new();
    l.when_line(line.hit());
    f.mock_pin(4).drive_high(); // white surface: no line
    f.advance(ms(100));
    assert!(!l.line_detected());
    f.mock_pin(4).drive_low(); // black line
    f.advance(ms(10));
    // One low sample out of five: median still high.
    assert!(!l.line_detected());
    f.advance(ms(50));
    assert!(l.line_detected());
    assert_eq!(line.get(), 1);
    assert!(l.wait_for_line(Some(Duration::ZERO)));
}

#[test]
fn light_sensor_measures_charge_time() {
    let f = setup();
    let pin = f.charging_pin(18, ms(2));
    let s = LightSensor::new(18).unwrap();
    let dark = Counter::new();
    s.when_dark(dark.hit());
    f.advance(Duration::from_secs(1));
    assert_approx(s.value(), 0.8);
    assert!(s.light_detected());
    pin.set_charge_time(ms(20));
    f.advance(Duration::from_secs(1));
    assert_approx(s.value(), 0.0);
    assert!(!s.light_detected());
    assert_eq!(dark.get(), 1);
}

#[test]
fn distance_sensor() {
    let f = setup();
    let trigger = f.trigger_pin(17, 18, Some(0.5));
    let s = DistanceSensor::new(18, 17).unwrap();
    let in_range = Counter::new();
    s.when_in_range(in_range.hit());
    f.advance(ms(600));
    assert!((s.distance() - 0.5).abs() < 0.001, "{}", s.distance());
    assert!(!s.in_range());
    trigger.set_distance(Some(0.1));
    f.advance(Duration::from_secs(1));
    assert!((s.distance() - 0.1).abs() < 0.001, "{}", s.distance());
    assert!(s.in_range());
    assert_eq!(in_range.get(), 1);
    trigger.set_distance(None);
    f.advance(Duration::from_secs(2));
    assert_approx(s.distance(), 1.0);
    assert!(!s.in_range());
    assert_eq!(s.pins(), (18, 17));
    assert_approx(s.threshold_distance(), 0.3);
}

fn turn(a: &MockPin, b: &MockPin, clockwise: bool) {
    let (first, second) = if clockwise { (b, a) } else { (a, b) };
    first.drive_low();
    second.drive_low();
    first.drive_high();
    second.drive_high();
}

use gpio_zero::pins::mock::MockPin;

#[test]
fn rotary_encoder() {
    let f = setup();
    let e = RotaryEncoder::new(20, 21).unwrap();
    let (cw, ccw, any) = (Counter::new(), Counter::new(), Counter::new());
    e.when_rotated_clockwise(cw.hit());
    e.when_rotated_counter_clockwise(ccw.hit());
    e.when_rotated(any.hit());
    let (a, b) = (f.mock_pin(20), f.mock_pin(21));
    turn(&a, &b, true);
    turn(&a, &b, true);
    assert_eq!(e.steps(), 2);
    assert_approx(e.value(), 2.0 / 16.0);
    turn(&a, &b, false);
    assert_eq!(e.steps(), 1);
    assert_eq!((cw.get(), ccw.get(), any.get()), (2, 1, 3));
    // A half turn that goes back does not count.
    b.drive_low();
    b.drive_high();
    assert_eq!(e.steps(), 1);
    e.set_steps(-3);
    assert_eq!(e.steps(), -3);
}

#[test]
fn rotary_encoder_limits() {
    let f = setup();
    let clamp = RotaryEncoder::with_options(
        20,
        21,
        RotaryEncoderOptions {
            max_steps: 2,
            ..Default::default()
        },
    )
    .unwrap();
    let (a, b) = (f.mock_pin(20), f.mock_pin(21));
    for _ in 0..5 {
        turn(&a, &b, true);
    }
    assert_eq!(clamp.steps(), 2);
    assert_approx(clamp.value(), 1.0);
    drop(clamp);

    let wrap = RotaryEncoder::with_options(
        20,
        21,
        RotaryEncoderOptions {
            max_steps: 2,
            wrap: true,
            ..Default::default()
        },
    )
    .unwrap();
    for _ in 0..3 {
        turn(&a, &b, true);
    }
    assert_eq!(wrap.steps(), -2);
    turn(&a, &b, false);
    assert_eq!(wrap.steps(), 2);
}

#[test]
fn rotary_wait_for_rotate() {
    let f = setup();
    let e = RotaryEncoder::new(20, 21).unwrap();
    let e2 = e.clone();
    let got = Arc::new(Mutex::new(None));
    let g = got.clone();
    let t = std::thread::spawn(move || {
        *g.lock().unwrap() = Some(e2.wait_for_rotate_clockwise(None));
    });
    std::thread::sleep(ms(20));
    turn(&f.mock_pin(20), &f.mock_pin(21), true);
    t.join().unwrap();
    assert_eq!(*got.lock().unwrap(), Some(true));
}
