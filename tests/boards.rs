mod common;

use std::time::Duration;

use common::*;
use gpio_zero::prelude::*;

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

#[test]
fn led_board_basics() {
    let f = setup();
    let b = LedBoard::new(&[2, 3, 4]).unwrap();
    assert_eq!(b.len(), 3);
    assert!(!b.is_empty());
    assert!(!b.is_lit());
    b.on().unwrap();
    assert_eq!(b.value(), vec![1.0; 3]);
    b.off_at(&[1]).unwrap();
    assert_eq!(b.value(), vec![1.0, 0.0, 1.0]);
    b.toggle().unwrap();
    assert_eq!(b.value(), vec![0.0, 1.0, 0.0]);
    b.toggle_at(&[0, 1]).unwrap();
    assert_eq!(b.value(), vec![1.0, 0.0, 0.0]);
    b.on_at(&[2]).unwrap();
    assert_approx(f.mock_pin(4).value(), 1.0);
    assert!(b.on_at(&[7]).is_err());
    assert!(b.set_value(&[1.0]).is_err());
    assert!(b.set_value(&[0.5, 0.0, 0.0]).is_err());
    b.set_value(&[0.0, 1.0, 1.0]).unwrap();
    let led = b.led(0).unwrap();
    led.on().unwrap();
    assert_eq!(b.value(), vec![1.0; 3]);
    assert!(b.pwm_led(0).is_none());
    assert_eq!(b.pins(), vec![2, 3, 4]);
    assert!(LedBoard::new(&[]).is_err());
}

#[test]
fn led_board_blink_and_pulse() {
    let f = setup();
    let b = LedBoard::new(&[2, 3]).unwrap();
    b.blink(
        Duration::from_secs(1),
        Duration::from_secs(1),
        Some(1),
        true,
    )
    .unwrap();
    f.advance(ms(500));
    assert_eq!(b.value(), vec![1.0, 1.0]);
    f.advance(Duration::from_secs(1));
    assert_eq!(b.value(), vec![0.0, 0.0]);
    assert!(b.pulse(ms(100), ms(100), None, true).is_err());
    drop(b);

    let p = LedBoard::with_options(
        &[2, 3],
        LedBoardOptions {
            pwm: true,
            ..Default::default()
        },
    )
    .unwrap();
    p.pulse(ms(200), ms(200), Some(1), true).unwrap();
    f.advance(ms(80));
    assert_approx(p.value()[0], 0.4);
    assert_approx(p.value()[1], 0.4);
    p.pwm_led(1).unwrap().set_value(0.3).unwrap();
    f.advance(Duration::from_secs(1));
    assert_eq!(p.value(), vec![0.0, 0.0]);
}

#[test]
fn led_board_source() {
    let f = setup();
    let b = LedBoard::new(&[2, 3]).unwrap();
    b.set_source(vec![vec![1.0, 0.0], vec![0.0, 1.0]]);
    f.settle();
    assert_eq!(b.value(), vec![1.0, 0.0]);
    f.advance(ms(10));
    assert_eq!(b.value(), vec![0.0, 1.0]);
    assert_eq!(b.values().next(), Some(vec![0.0, 1.0]));
}

#[test]
fn bar_graph() {
    let _f = setup();
    let g = LedBarGraph::new(&[1, 2, 3, 4, 5]).unwrap();
    g.set_value(0.4).unwrap();
    assert_eq!(g.board_value(), vec![1.0, 1.0, 0.0, 0.0, 0.0]);
    assert_approx(g.value(), 0.4);
    g.set_value(-0.4).unwrap();
    assert_eq!(g.board_value(), vec![0.0, 0.0, 0.0, 1.0, 1.0]);
    assert_approx(g.value(), -0.4);
    g.set_lit_count(5.0).unwrap();
    assert_approx(g.lit_count(), 5.0);
    g.set_value(0.5).unwrap(); // 2.5 LEDs: rounds down without PWM
    assert_approx(g.lit_count(), 2.0);
    assert!(g.set_value(1.5).is_err());
}

trait BoardValue {
    fn board_value(&self) -> Vec<f64>;
}

impl BoardValue for LedBarGraph {
    fn board_value(&self) -> Vec<f64> {
        LedBoard::value(self)
    }
}

#[test]
fn pwm_bar_graph_and_source() {
    let f = setup();
    let g = LedBarGraph::with_options(
        &[1, 2, 3, 4],
        LedBoardOptions {
            pwm: true,
            ..Default::default()
        },
    )
    .unwrap();
    g.set_value(0.625).unwrap();
    assert_eq!(g.board_value(), vec![1.0, 1.0, 0.5, 0.0]);
    assert_approx(g.value(), 0.625);
    g.set_source(vec![0.25]);
    f.advance(ms(20));
    assert_eq!(g.board_value(), vec![1.0, 0.0, 0.0, 0.0]);
    assert_approx(g.values().next().unwrap(), 0.25);
}

#[test]
fn traffic_lights() {
    let f = setup();
    let t = TrafficLights::new(2, 3, 4).unwrap();
    t.green().on().unwrap();
    assert_eq!(t.value(), vec![0.0, 0.0, 1.0]);
    t.green().off().unwrap();
    t.amber().on().unwrap();
    assert!(t.yellow().is_lit());
    t.red().on().unwrap();
    assert_approx(f.mock_pin(2).value(), 1.0);
    t.off().unwrap();
    assert!(!t.is_lit());
}

#[test]
fn button_board() {
    let f = setup();
    let bb = ButtonBoard::new(&[5, 6]).unwrap();
    let (pressed, released, changed) = (Counter::new(), Counter::new(), Counter::new());
    bb.when_pressed(pressed.hit());
    bb.when_released(released.hit());
    bb.when_changed(changed.hit());
    assert!(!bb.is_pressed());
    f.mock_pin(5).drive_low();
    f.mock_pin(6).drive_low();
    assert_eq!(bb.value(), vec![true, true]);
    assert!(bb.is_active());
    assert_eq!((pressed.get(), changed.get()), (1, 2));
    f.mock_pin(5).drive_high();
    assert_eq!(released.get(), 0);
    f.mock_pin(6).drive_high();
    assert_eq!((pressed.get(), released.get(), changed.get()), (1, 1, 4));
    assert!(bb.wait_for_release(Some(Duration::ZERO)));
    assert!(!bb.wait_for_press(Some(Duration::ZERO)));
    // Individual buttons keep their own callbacks.
    let own = Counter::new();
    bb.button(0).unwrap().when_pressed(own.hit());
    f.mock_pin(5).drive_low();
    assert_eq!((own.get(), pressed.get()), (1, 2));
    assert_eq!(bb.values().next(), Some(vec![true, false]));
    assert_eq!(bb.len(), 2);
}

#[test]
fn robot() {
    let f = setup();
    let r = Robot::new((4, 14), (17, 18)).unwrap();
    r.forward(1.0).unwrap();
    assert_eq!(r.value(), (1.0, 1.0));
    r.left(0.5).unwrap();
    assert_eq!(r.value(), (-0.5, 0.5));
    r.right(1.0).unwrap();
    assert_eq!(r.value(), (1.0, -1.0));
    assert_approx(f.mock_pin(18).value(), 1.0);
    r.reverse().unwrap();
    assert_eq!(r.value(), (-1.0, 1.0));
    r.forward_curved(1.0, 0.5, 0.0).unwrap();
    assert_eq!(r.value(), (0.5, 1.0));
    r.backward_curved(0.8, 0.0, 0.5).unwrap();
    assert_approx(r.value().0, -0.8);
    assert_approx(r.value().1, -0.4);
    assert!(r.forward_curved(1.0, 0.5, 0.5).is_err());
    r.stop().unwrap();
    assert!(!r.is_active());
    r.set_source(vec![(0.2, 0.3)]);
    f.advance(ms(20));
    assert_eq!(r.values().next(), Some((0.2, 0.3)));
    assert_approx(r.left_motor().value(), 0.2);
    assert_approx(r.right_motor().value(), 0.3);
}
