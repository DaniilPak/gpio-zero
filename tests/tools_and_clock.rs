mod common;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use common::*;
use gpio_zero::clock::{Clock, Event, MockClock, RealClock};
use gpio_zero::pins::{Edges, PinFactory, PinFunction, Pull};
use gpio_zero::tools::*;

#[test]
fn tool_adapters() {
    assert_eq!(
        negated(vec![true, false]).collect::<Vec<_>>(),
        vec![false, true]
    );
    assert_eq!(inverted(vec![0.25]).collect::<Vec<_>>(), vec![0.75]);
    assert_eq!(
        scaled_full(vec![0.0, 0.5, 1.0]).collect::<Vec<_>>(),
        vec![-1.0, 0.0, 1.0]
    );
    assert_eq!(
        scaled_half(vec![-1.0, 1.0]).collect::<Vec<_>>(),
        vec![0.0, 1.0]
    );
    assert_eq!(
        clamped(vec![-2.0, 0.5, 2.0], 0.0, 1.0).collect::<Vec<_>>(),
        vec![0.0, 0.5, 1.0]
    );
    assert_eq!(absoluted(vec![-0.5]).collect::<Vec<_>>(), vec![0.5]);
    assert_eq!(
        booleanized(vec![0.2, 0.6], 0.5, 1.0).collect::<Vec<_>>(),
        vec![false, true]
    );
    assert_eq!(
        quantized(vec![0.0, 0.49, 0.5, 1.0], 4).collect::<Vec<_>>(),
        vec![0.0, 0.25, 0.5, 0.75]
    );
    assert_eq!(
        smoothed(vec![1.0, 3.0, 5.0], 2).collect::<Vec<_>>(),
        vec![1.0, 2.0, 4.0]
    );
    assert_eq!(
        all_values(vec![true, true], vec![true, false]).collect::<Vec<_>>(),
        vec![true, false]
    );
    assert_eq!(
        any_values(vec![false, false], vec![true, false]).collect::<Vec<_>>(),
        vec![true, false]
    );
    assert_eq!(
        averaged(vec![1.0], vec![0.0]).collect::<Vec<_>>(),
        vec![0.5]
    );
    assert_eq!(summed(vec![1.0], vec![2.0]).collect::<Vec<_>>(), vec![3.0]);
    assert_eq!(
        multiplied(vec![0.5], vec![0.5]).collect::<Vec<_>>(),
        vec![0.25]
    );
    assert_eq!(
        alternating_values(true).take(3).collect::<Vec<_>>(),
        vec![true, false, true]
    );
    let s: Vec<f64> = sin_values(4).take(5).collect();
    assert_approx(s[1], 1.0);
    assert_approx(s[3], -1.0);
    assert_approx(s[4], 0.0);
    assert_approx(cos_values(4).next().unwrap(), 1.0);
    assert_eq!(
        ramping_values(4).take(4).collect::<Vec<_>>(),
        vec![0.0, 0.5, 1.0, 0.5]
    );
    assert!(random_values().take(100).all(|v| (0.0..1.0).contains(&v)));
}

#[test]
fn mock_clock_orders_sleepers() {
    let c = Arc::new(MockClock::new());
    let log = Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut handles = Vec::new();
    for d in [30u64, 10, 20] {
        let (c2, log2) = (c.clone(), log.clone());
        handles.push(c.spawn(
            "t",
            Box::new(move || {
                c2.sleep(Duration::from_millis(d));
                log2.lock().unwrap().push((d, c2.now()));
            }),
        ));
    }
    c.advance(Duration::from_millis(25));
    assert_eq!(
        *log.lock().unwrap(),
        vec![
            (10, Duration::from_millis(10)),
            (20, Duration::from_millis(20))
        ]
    );
    assert_eq!(c.now(), Duration::from_millis(25));
    c.advance(Duration::from_millis(25));
    assert_eq!(log.lock().unwrap().len(), 3);
    for h in handles {
        h.join().unwrap();
    }
    assert_eq!(c.live_tasks(), 0);
}

#[test]
fn events_wake_waiters() {
    let c = Arc::new(MockClock::new());
    let ev = Event::new();
    let done = Arc::new(AtomicBool::new(false));
    let (c2, ev2, d2) = (c.clone(), ev.clone(), done.clone());
    let h = c.spawn(
        "w",
        Box::new(move || {
            let got = c2.wait(&ev2, None);
            d2.store(got, Ordering::SeqCst);
        }),
    );
    c.settle();
    assert!(!done.load(Ordering::SeqCst));
    ev.set();
    h.join().unwrap();
    assert!(done.load(Ordering::SeqCst));
    assert!(ev.is_set());
    ev.clear();
    assert!(!ev.is_set());
    assert!(!ev.wait_real(Some(Duration::from_millis(1))));
}

#[test]
fn real_clock_moves() {
    let c = RealClock::new();
    let a = c.now();
    c.sleep(Duration::from_millis(2));
    assert!(c.now() >= a + Duration::from_millis(2));
}

#[test]
fn mock_pin_behaviour() {
    let f = MockFactory::new();
    let pin = f.reserve_pin(7).unwrap();
    assert!(f.reserve_pin(7).is_err());
    assert_eq!(pin.function(), PinFunction::Input);
    pin.set_pull(Pull::Up).unwrap();
    assert_approx(pin.state().unwrap(), 1.0);
    assert!(pin.set_state(1.0).is_err());
    let hits = Counter::new();
    let h = hits.hit();
    pin.set_edges(Edges::Falling).unwrap();
    pin.set_when_changed(Some(Arc::new(move |_, _| h())))
        .unwrap();
    let m = f.mock_pin(7);
    m.drive_low();
    m.drive_high();
    assert_eq!(hits.get(), 1);
    m.release();
    assert_approx(m.value(), 1.0);
    pin.set_function(PinFunction::Output).unwrap();
    assert!(pin.set_state(0.5).is_err());
    pin.set_frequency(Some(100.0)).unwrap();
    pin.set_state(0.5).unwrap();
    pin.set_frequency(None).unwrap();
    assert_approx(pin.state().unwrap(), 0.0);
    assert!(pin.set_frequency(Some(-1.0)).is_err());
    m.clear_history();
    assert_eq!(m.states().len(), 1);
    pin.close();
    f.release_pin(7);
    assert_eq!(pin.function(), PinFunction::Input);
    f.reserve_pin(7).unwrap();
    assert_eq!(f.name(), "mock");
    f.reset();
    f.reserve_pin(7).unwrap();
}

#[test]
fn default_factory_from_thread() {
    let f = setup();
    let d = gpio_zero::pins::default_factory().unwrap();
    assert_eq!(d.name(), "mock");
    let _ = f;
    gpio_zero::pins::set_thread_factory(None);
}
