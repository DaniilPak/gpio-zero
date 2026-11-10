#![allow(dead_code)]

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

pub use gpio_zero::pins::mock::MockFactory;

/// A fresh mock factory installed as this test thread's default.
pub fn setup() -> Arc<MockFactory> {
    let f = MockFactory::new();
    f.install_for_thread();
    f
}

pub fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

#[track_caller]
pub fn assert_approx(a: f64, b: f64) {
    assert!(approx(a, b), "{a} != {b}");
}

#[track_caller]
pub fn assert_states(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len(), "{actual:?} vs {expected:?}");
    for (a, e) in actual.iter().zip(expected) {
        assert!(approx(*a, *e), "{actual:?} vs {expected:?}");
    }
}

/// A shareable counter for callbacks.
#[derive(Clone, Default)]
pub struct Counter(Arc<AtomicUsize>);

impl Counter {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn hit(&self) -> impl Fn() + Send + Sync + 'static {
        let c = self.0.clone();
        move || {
            c.fetch_add(1, Ordering::SeqCst);
        }
    }
    pub fn get(&self) -> usize {
        self.0.load(Ordering::SeqCst)
    }
}
