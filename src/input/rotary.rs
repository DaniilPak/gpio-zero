use std::fmt;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::clock::{Clock, Event, lock};
use crate::devices::{Callback, Values};
use crate::error::Result;
use crate::pins::{Edges, Pin, PinFactory, PinFunction, PinHandle, Pull, resolve};

/// Options for [`RotaryEncoder`].
#[derive(Clone)]
pub struct RotaryEncoderOptions {
    /// Debounce period for both pins.
    pub bounce_time: Option<Duration>,
    /// Steps at which `value()` reaches ±1 (default 16). `0` = unbounded
    /// (value is then always 0).
    pub max_steps: u32,
    /// Wrap from `max_steps` to `-max_steps` instead of clamping.
    pub wrap: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for RotaryEncoderOptions {
    fn default() -> Self {
        RotaryEncoderOptions {
            bounce_time: None,
            max_steps: 16,
            wrap: false,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for RotaryEncoderOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RotaryEncoderOptions")
            .field("bounce_time", &self.bounce_time)
            .field("max_steps", &self.max_steps)
            .field("wrap", &self.wrap)
            .finish_non_exhaustive()
    }
}

// Quadrature decoder states (from gpiozero's RotaryEncoder).
const IDLE: u8 = 0;
const CCW1: u8 = 1;
const CCW2: u8 = 2;
const CCW3: u8 = 3;
const CW1: u8 = 4;
const CW2: u8 = 5;
const CW3: u8 = 6;
const PLUS: u8 = 7;
const MINUS: u8 = 8;

/// `TRANSITIONS[state][a_active | b_active << 1]`.
const TRANSITIONS: [[u8; 4]; 7] = [
    [IDLE, CCW1, CW1, IDLE],
    [IDLE, CCW1, CCW3, CCW2],
    [IDLE, CCW1, CCW3, CCW2],
    [MINUS, IDLE, CCW3, CCW2],
    [IDLE, CW3, CW1, CW2],
    [IDLE, CW3, CW1, CW2],
    [PLUS, CW3, IDLE, CW2],
];

struct EncState {
    state: u8,
    steps: i64,
}

struct EncShared {
    clock: Arc<dyn Clock>,
    a: Arc<dyn Pin>,
    b: Arc<dyn Pin>,
    max_steps: i64,
    wrap: bool,
    st: Mutex<EncState>,
    rotated: Event,
    rotated_cw: Event,
    rotated_ccw: Event,
    when_rotated: Mutex<Option<Callback>>,
    when_cw: Mutex<Option<Callback>>,
    when_ccw: Mutex<Option<Callback>>,
}

impl EncShared {
    fn on_edge(&self) {
        // Pull-ups: a pin is "active" when low.
        let a = self.a.state().map(|s| s < 0.5).unwrap_or(false);
        let b = self.b.state().map(|s| s < 0.5).unwrap_or(false);
        let index = usize::from(a) | usize::from(b) << 1;
        let dir = {
            let mut st = lock(&self.st);
            let next = TRANSITIONS[usize::from(st.state)][index];
            let dir = match next {
                PLUS => 1,
                MINUS => -1,
                _ => 0,
            };
            st.state = if dir == 0 { next } else { IDLE };
            if dir != 0 {
                let mut s = st.steps + dir;
                if self.max_steps > 0 {
                    let m = self.max_steps;
                    if self.wrap {
                        if s > m {
                            s = -m + (s - m - 1);
                        } else if s < -m {
                            s = m - (-m - s - 1);
                        }
                    } else {
                        s = s.clamp(-m, m);
                    }
                }
                st.steps = s;
            }
            dir
        };
        if dir == 0 {
            return;
        }
        let (ev, cb) = if dir > 0 {
            (&self.rotated_cw, &self.when_cw)
        } else {
            (&self.rotated_ccw, &self.when_ccw)
        };
        self.rotated.set();
        ev.set();
        let cb = lock(cb).clone();
        let any = lock(&self.when_rotated).clone();
        if let Some(cb) = cb {
            cb();
        }
        if let Some(cb) = any {
            cb();
        }
    }
}

struct EncInner {
    shared: Arc<EncShared>,
    a: PinHandle,
    b: PinHandle,
}

impl Drop for EncInner {
    fn drop(&mut self) {
        let _ = self.a.set_when_changed(None);
        let _ = self.b.set_when_changed(None);
    }
}

/// A quadrature rotary encoder (e.g. KY-040) on two pins with pull-ups.
#[derive(Clone)]
pub struct RotaryEncoder {
    inner: Arc<EncInner>,
}

impl fmt::Debug for RotaryEncoder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RotaryEncoder")
            .field("steps", &self.steps())
            .finish()
    }
}

impl RotaryEncoder {
    /// Creates an encoder on pins `a` and `b`.
    pub fn new(a: u32, b: u32) -> Result<Self> {
        Self::with_options(a, b, RotaryEncoderOptions::default())
    }

    /// Creates an encoder with explicit options.
    pub fn with_options(a: u32, b: u32, o: RotaryEncoderOptions) -> Result<Self> {
        let factory = resolve(&o.pin_factory)?;
        let pa = PinHandle::new(&factory, a)?;
        let pb = PinHandle::new(&factory, b)?;
        for p in [&pa, &pb] {
            p.set_function(PinFunction::Input)?;
            p.set_pull(Pull::Up)?;
            p.set_bounce(o.bounce_time)?;
            p.set_edges(Edges::Both)?;
        }
        let shared = Arc::new(EncShared {
            clock: factory.clock(),
            a: pa.pin.clone(),
            b: pb.pin.clone(),
            max_steps: i64::from(o.max_steps),
            wrap: o.wrap,
            st: Mutex::new(EncState {
                state: IDLE,
                steps: 0,
            }),
            rotated: Event::new(),
            rotated_cw: Event::new(),
            rotated_ccw: Event::new(),
            when_rotated: Mutex::new(None),
            when_cw: Mutex::new(None),
            when_ccw: Mutex::new(None),
        });
        for p in [&pa, &pb] {
            let s = shared.clone();
            p.set_when_changed(Some(Arc::new(move |_, _| s.on_edge())))?;
        }
        Ok(RotaryEncoder {
            inner: Arc::new(EncInner {
                shared,
                a: pa,
                b: pb,
            }),
        })
    }

    /// Steps counted (clockwise positive).
    pub fn steps(&self) -> i64 {
        lock(&self.inner.shared.st).steps
    }

    /// Resets the step counter.
    pub fn set_steps(&self, steps: i64) {
        let m = self.inner.shared.max_steps;
        let s = if m > 0 { steps.clamp(-m, m) } else { steps };
        lock(&self.inner.shared.st).steps = s;
    }

    /// `steps / max_steps`, `-1.0..=1.0` (0 when unbounded).
    pub fn value(&self) -> f64 {
        let m = self.inner.shared.max_steps;
        if m == 0 {
            0.0
        } else {
            self.steps() as f64 / m as f64
        }
    }

    /// Sets the position as a value (`steps = value * max_steps`).
    pub fn set_value(&self, value: f64) {
        let m = self.inner.shared.max_steps;
        self.set_steps((value.clamp(-1.0, 1.0) * m as f64).round() as i64);
    }

    /// The configured `max_steps`.
    pub fn max_steps(&self) -> u32 {
        self.inner.shared.max_steps as u32
    }

    /// Whether the counter wraps.
    pub fn wrap(&self) -> bool {
        self.inner.shared.wrap
    }

    /// Runs `f` on every detent, in either direction.
    pub fn when_rotated(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.inner.shared.when_rotated) = Some(Arc::new(f));
    }

    /// Runs `f` on every clockwise detent.
    pub fn when_rotated_clockwise(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.inner.shared.when_cw) = Some(Arc::new(f));
    }

    /// Runs `f` on every counter-clockwise detent.
    pub fn when_rotated_counter_clockwise(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.inner.shared.when_ccw) = Some(Arc::new(f));
    }

    fn wait(&self, ev: &Event, timeout: Option<Duration>) -> bool {
        ev.clear();
        self.inner.shared.clock.wait(ev, timeout)
    }

    /// Blocks until the encoder is turned (or `timeout`).
    pub fn wait_for_rotate(&self, timeout: Option<Duration>) -> bool {
        self.wait(&self.inner.shared.rotated, timeout)
    }

    /// Blocks until the encoder is turned clockwise (or `timeout`).
    pub fn wait_for_rotate_clockwise(&self, timeout: Option<Duration>) -> bool {
        self.wait(&self.inner.shared.rotated_cw, timeout)
    }

    /// Blocks until the encoder is turned counter-clockwise (or `timeout`).
    pub fn wait_for_rotate_counter_clockwise(&self, timeout: Option<Duration>) -> bool {
        self.wait(&self.inner.shared.rotated_ccw, timeout)
    }

    /// An endless iterator over `value()`.
    pub fn values(&self) -> Values<f64> {
        let me = Arc::downgrade(&self.inner);
        Values::from_fn(move || {
            me.upgrade().map(|i| {
                let m = i.shared.max_steps;
                if m == 0 {
                    0.0
                } else {
                    lock(&i.shared.st).steps as f64 / m as f64
                }
            })
        })
    }
}
