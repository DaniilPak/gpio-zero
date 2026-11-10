//! A fully simulated pin factory for tests and development without hardware.
//!
//! ```
//! use gpio_zero::pins::mock::MockFactory;
//! use gpio_zero::{Button, Led};
//! use std::time::Duration;
//!
//! let factory = MockFactory::new();
//! factory.install_for_thread();
//!
//! let led = Led::new(17).unwrap();
//! let button = Button::new(2).unwrap();
//! let l = led.clone();
//! button.when_pressed(move || l.on().unwrap());
//!
//! factory.mock_pin(2).drive_low(); // pull-up button: low = pressed
//! assert!(led.is_lit());
//!
//! led.blink(Duration::from_secs(1), Duration::from_secs(1), Some(2), true).unwrap();
//! factory.advance(Duration::from_secs(4));
//! assert!(!led.is_lit());
//! ```

use std::collections::HashMap;
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use super::{
    EdgeCallback, Edges, Pin, PinFactory, PinFunction, Pull, Reservations, SoftwareSpi, SpiConfig,
    SpiInterface,
};
use crate::clock::{Clock, MockClock, RealClock, Task, lock};
use crate::error::{Error, Result, invalid};

/// Hook called after a mock pin's output state is written: `(old, new)`.
pub type WriteHook = Arc<dyn Fn(f64, f64) + Send + Sync>;

enum Behavior {
    Plain,
    /// Simulates an RC circuit (LDR + capacitor): after switching to input the
    /// pin reads high once `charge_time` has elapsed.
    Charging {
        charge_time: Duration,
    },
    /// Simulates an HC-SR04 trigger: a falling edge makes the echo pin pulse
    /// high for `echo_time` (no pulse when `None`).
    Trigger {
        echo: Weak<MockPin>,
        echo_time: Option<Duration>,
    },
}

struct State {
    function: PinFunction,
    pull: Pull,
    frequency: Option<f64>,
    bounce: Option<Duration>,
    edges: Edges,
    output: f64,
    driven: Option<bool>,
    input: bool,
    last_edge: Option<Duration>,
    callback: Option<EdgeCallback>,
    history: Vec<(Duration, f64)>,
    write_hook: Option<WriteHook>,
    behavior: Behavior,
    task: Option<Task>,
}

impl State {
    fn input_level(&self) -> bool {
        self.driven.unwrap_or(self.pull == Pull::Up)
    }

    fn current(&self) -> f64 {
        match self.function {
            PinFunction::Input => f64::from(u8::from(self.input)),
            PinFunction::Output => self.output,
        }
    }

    /// Appends the current state to the history if it changed.
    fn record(&mut self, now: Duration) {
        let v = self.current();
        if self.history.last().map(|h| h.1) != Some(v) {
            self.history.push((now, v));
        }
    }
}

/// A simulated pin. Obtain one with [`MockFactory::mock_pin`].
pub struct MockPin {
    number: u32,
    pwm: bool,
    clock: Arc<dyn Clock>,
    me: Weak<MockPin>,
    st: Mutex<State>,
}

impl fmt::Debug for MockPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let st = lock(&self.st);
        f.debug_struct("MockPin")
            .field("number", &self.number)
            .field("function", &st.function)
            .field("state", &st.current())
            .finish()
    }
}

/// What a pending edge notification needs.
type Pending = Option<(EdgeCallback, Duration, bool)>;

impl MockPin {
    fn create(number: u32, pwm: bool, clock: Arc<dyn Clock>, behavior: Behavior) -> Arc<Self> {
        let now = clock.now();
        Arc::new_cyclic(|me| MockPin {
            number,
            pwm,
            clock,
            me: me.clone(),
            st: Mutex::new(State {
                function: PinFunction::Input,
                pull: Pull::Floating,
                frequency: None,
                bounce: None,
                edges: Edges::Both,
                output: 0.0,
                driven: None,
                input: false,
                last_edge: None,
                callback: None,
                history: vec![(now, 0.0)],
                write_hook: None,
                behavior,
                task: None,
            }),
        })
    }

    /// Recomputes the input level, records history and returns the edge
    /// notification to deliver (outside the lock).
    fn update_input(&self, st: &mut State, notify: bool) -> Pending {
        let level = st.input_level();
        if st.function != PinFunction::Input || level == st.input {
            return None;
        }
        st.input = level;
        let now = self.clock.now();
        st.record(now);
        if !notify || !st.edges.matches(level) {
            return None;
        }
        if let (Some(b), Some(last)) = (st.bounce, st.last_edge)
            && now.saturating_sub(last) < b
        {
            return None;
        }
        let cb = st.callback.clone()?;
        st.last_edge = Some(now);
        Some((cb, now, level))
    }

    fn deliver(pending: Pending) {
        if let Some((cb, ticks, level)) = pending {
            cb(ticks, level);
        }
    }

    /// Simulates an external signal driving the pin to `level`.
    pub fn drive(&self, level: bool) {
        let pending = {
            let mut st = lock(&self.st);
            st.driven = Some(level);
            self.update_input(&mut st, true)
        };
        Self::deliver(pending);
    }

    /// Drives the pin high (e.g. releases a pull-down button).
    pub fn drive_high(&self) {
        self.drive(true);
    }

    /// Drives the pin low (e.g. presses a pull-up button).
    pub fn drive_low(&self) {
        self.drive(false);
    }

    /// Stops driving the pin; it floats to its pull level.
    pub fn release(&self) {
        let pending = {
            let mut st = lock(&self.st);
            st.driven = None;
            self.update_input(&mut st, true)
        };
        Self::deliver(pending);
    }

    /// The current physical state (`Pin::state` without the `Result`).
    pub fn value(&self) -> f64 {
        lock(&self.st).current()
    }

    /// Every state the pin has had, with the clock time of each change.
    pub fn history(&self) -> Vec<(Duration, f64)> {
        lock(&self.st).history.clone()
    }

    /// Just the states from [`history`](MockPin::history).
    pub fn states(&self) -> Vec<f64> {
        lock(&self.st).history.iter().map(|(_, v)| *v).collect()
    }

    /// Forgets the history (keeping the current state as its only entry).
    pub fn clear_history(&self) {
        let mut st = lock(&self.st);
        let now = self.clock.now();
        let cur = st.current();
        st.history = vec![(now, cur)];
    }

    /// Installs a hook run after every output write.
    pub fn set_write_hook(&self, hook: Option<WriteHook>) {
        lock(&self.st).write_hook = hook;
    }

    /// For trigger pins: sets the simulated echo pulse length (`None` = no echo).
    pub fn set_echo_time(&self, echo_time: Option<Duration>) {
        if let Behavior::Trigger { echo_time: t, .. } = &mut lock(&self.st).behavior {
            *t = echo_time;
        }
    }

    /// For trigger pins: sets the echo pulse for an object `meters` away.
    pub fn set_distance(&self, meters: Option<f64>) {
        self.set_echo_time(
            meters.map(|m| Duration::from_secs_f64(2.0 * m / crate::input::SPEED_OF_SOUND)),
        );
    }

    /// For charging pins: sets the simulated charge time.
    pub fn set_charge_time(&self, charge_time: Duration) {
        if let Behavior::Charging { charge_time: t } = &mut lock(&self.st).behavior {
            *t = charge_time;
        }
    }

    fn start_charging(&self, st: &mut State) -> Option<Task> {
        let Behavior::Charging { charge_time } = st.behavior else {
            return None;
        };
        let old = st.task.take();
        let me = self.me.clone();
        let clock = self.clock.clone();
        st.task = Some(Task::spawn(&self.clock, "mock-charge", move |stop| {
            if !clock.wait(&stop, Some(charge_time))
                && let Some(p) = me.upgrade()
            {
                p.drive(true);
            }
        }));
        old
    }
}

impl Pin for MockPin {
    fn number(&self) -> u32 {
        self.number
    }

    fn function(&self) -> PinFunction {
        lock(&self.st).function
    }

    fn set_function(&self, function: PinFunction) -> Result<()> {
        let mut old_task = None;
        {
            let mut st = lock(&self.st);
            if st.function == function {
                return Ok(());
            }
            let now = self.clock.now();
            st.function = function;
            match function {
                PinFunction::Input => {
                    st.input = st.input_level();
                    st.frequency = None;
                    st.record(now);
                    if matches!(st.behavior, Behavior::Charging { .. }) {
                        old_task = self.start_charging(&mut st);
                    }
                }
                PinFunction::Output => {
                    if matches!(st.behavior, Behavior::Charging { .. }) {
                        old_task = st.task.take();
                        st.driven = Some(false);
                    }
                    st.record(now);
                }
            }
        }
        drop(old_task);
        Ok(())
    }

    fn state(&self) -> Result<f64> {
        Ok(lock(&self.st).current())
    }

    fn set_state(&self, value: f64) -> Result<()> {
        if !(0.0..=1.0).contains(&value) {
            return invalid(format!("pin state {value} is outside 0..=1"));
        }
        let mut old_task = None;
        let (hook, old) = {
            let mut st = lock(&self.st);
            if st.function == PinFunction::Input {
                return Err(Error::PinSetInput(self.number));
            }
            if st.frequency.is_none() && value != 0.0 && value != 1.0 {
                return Err(Error::PwmUnsupported(self.number));
            }
            let old = st.output;
            st.output = value;
            let now = self.clock.now();
            st.record(now);
            if let Behavior::Trigger {
                echo,
                echo_time: Some(echo_time),
            } = &st.behavior
                && old >= 0.5
                && value < 0.5
                && let Some(echo) = echo.upgrade()
            {
                let echo_time = *echo_time;
                let clock = self.clock.clone();
                old_task = st.task.take();
                st.task = Some(Task::spawn(&self.clock, "mock-echo", move |stop| {
                    if clock.wait(&stop, Some(Duration::from_micros(100))) {
                        return;
                    }
                    echo.drive(true);
                    clock.wait(&stop, Some(echo_time));
                    echo.drive(false);
                }));
            }
            (st.write_hook.clone(), old)
        };
        drop(old_task);
        if let Some(h) = hook {
            h(old, value);
        }
        Ok(())
    }

    fn pull(&self) -> Pull {
        lock(&self.st).pull
    }

    fn set_pull(&self, pull: Pull) -> Result<()> {
        let pending = {
            let mut st = lock(&self.st);
            st.pull = pull;
            self.update_input(&mut st, true)
        };
        Self::deliver(pending);
        Ok(())
    }

    fn frequency(&self) -> Option<f64> {
        lock(&self.st).frequency
    }

    fn set_frequency(&self, frequency: Option<f64>) -> Result<()> {
        let mut st = lock(&self.st);
        match frequency {
            Some(f) => {
                if !self.pwm {
                    return Err(Error::PwmUnsupported(self.number));
                }
                if f <= 0.0 || !f.is_finite() {
                    return invalid(format!("frequency {f} must be positive"));
                }
                st.frequency = Some(f);
            }
            None => {
                st.frequency = None;
                if st.output != 0.0 && st.output != 1.0 {
                    st.output = 0.0;
                    let now = self.clock.now();
                    st.record(now);
                }
            }
        }
        Ok(())
    }

    fn bounce(&self) -> Option<Duration> {
        lock(&self.st).bounce
    }

    fn set_bounce(&self, bounce: Option<Duration>) -> Result<()> {
        lock(&self.st).bounce = bounce;
        Ok(())
    }

    fn edges(&self) -> Edges {
        lock(&self.st).edges
    }

    fn set_edges(&self, edges: Edges) -> Result<()> {
        lock(&self.st).edges = edges;
        Ok(())
    }

    fn set_when_changed(&self, callback: Option<EdgeCallback>) -> Result<()> {
        lock(&self.st).callback = callback;
        Ok(())
    }

    fn close(&self) {
        let task = {
            let mut st = lock(&self.st);
            st.callback = None;
            st.frequency = None;
            st.function = PinFunction::Input;
            st.pull = Pull::Floating;
            st.bounce = None;
            st.edges = Edges::Both;
            st.input = st.input_level();
            let now = self.clock.now();
            st.record(now);
            st.task.take()
        };
        drop(task);
    }
}

/// A simulated SPI peripheral.
pub trait MockSpiDevice: Send + Sync + fmt::Debug {
    /// Given the bits clocked in so far (MSB first), returns the bits the
    /// device drives on MISO at the same positions. Bit `i` of the output may
    /// only depend on input bits before `i`.
    fn respond(&self, mosi: &[bool]) -> Vec<bool>;
}

fn bytes_to_bits(bytes: &[u8]) -> Vec<bool> {
    bytes
        .iter()
        .flat_map(|b| (0..8).rev().map(move |i| b >> i & 1 == 1))
        .collect()
}

fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().fold(0u8, |acc, &b| acc << 1 | u8::from(b)))
        .collect()
}

#[derive(Debug)]
struct MockHardwareSpi(Arc<dyn MockSpiDevice>);

impl SpiInterface for MockHardwareSpi {
    fn transfer(&self, tx: &[u8]) -> Result<Vec<u8>> {
        let mut out = self.0.respond(&bytes_to_bits(tx));
        out.resize(tx.len() * 8, false);
        Ok(bits_to_bytes(&out))
    }
}

enum MockClockKind {
    Virtual(MockClock),
    Real,
}

/// The mock pin factory. See the [module docs](self).
pub struct MockFactory {
    me: Weak<MockFactory>,
    clock: Arc<dyn Clock>,
    kind: MockClockKind,
    pins: Mutex<HashMap<u32, Arc<MockPin>>>,
    reservations: Reservations,
    spi: Mutex<HashMap<(u8, u8), Arc<dyn MockSpiDevice>>>,
    pwm: AtomicBool,
}

impl fmt::Debug for MockFactory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MockFactory")
            .field("clock", &self.clock)
            .finish_non_exhaustive()
    }
}

impl MockFactory {
    /// A mock factory with virtual time (see [`MockClock`]).
    pub fn new() -> Arc<Self> {
        let mc = MockClock::new();
        Arc::new_cyclic(|me| MockFactory {
            me: me.clone(),
            clock: Arc::new(mc.clone()),
            kind: MockClockKind::Virtual(mc),
            pins: Mutex::default(),
            reservations: Reservations::default(),
            spi: Mutex::default(),
            pwm: AtomicBool::new(true),
        })
    }

    /// A mock factory whose background tasks run in real time.
    pub fn with_real_time() -> Arc<Self> {
        Arc::new_cyclic(|me| MockFactory {
            me: me.clone(),
            clock: Arc::new(RealClock),
            kind: MockClockKind::Real,
            pins: Mutex::default(),
            reservations: Reservations::default(),
            spi: Mutex::default(),
            pwm: AtomicBool::new(true),
        })
    }

    /// Makes this factory the default for devices created on this thread.
    pub fn install_for_thread(self: &Arc<Self>) {
        super::set_thread_factory(Some(self.clone() as Arc<dyn PinFactory>));
    }

    /// Makes this factory the process-wide default.
    pub fn install(self: &Arc<Self>) {
        super::set_default_factory(self.clone() as Arc<dyn PinFactory>);
    }

    /// Returns this factory as a trait object (for device options).
    pub fn as_factory(self: &Arc<Self>) -> Arc<dyn PinFactory> {
        self.clone() as Arc<dyn PinFactory>
    }

    /// Whether pins created from now on support PWM (default `true`).
    pub fn set_pwm_capable(&self, pwm: bool) {
        self.pwm.store(pwm, Ordering::SeqCst);
    }

    fn get_or_create(&self, number: u32, behavior: impl FnOnce() -> Behavior) -> Arc<MockPin> {
        lock(&self.pins)
            .entry(number)
            .or_insert_with(|| {
                MockPin::create(
                    number,
                    self.pwm.load(Ordering::SeqCst),
                    self.clock.clone(),
                    behavior(),
                )
            })
            .clone()
    }

    /// The simulated pin `number` (created on first use).
    pub fn mock_pin(&self, number: u32) -> Arc<MockPin> {
        self.get_or_create(number, || Behavior::Plain)
    }

    /// Creates pin `number` as an RC-charging pin for [`LightSensor`](crate::LightSensor)
    /// tests. Call before creating the device.
    pub fn charging_pin(&self, number: u32, charge_time: Duration) -> Arc<MockPin> {
        let p = self.get_or_create(number, || Behavior::Charging { charge_time });
        p.set_charge_time(charge_time);
        p
    }

    /// Creates pin `trigger` as an HC-SR04 trigger wired to `echo`, simulating
    /// an object `distance` meters away (`None` = nothing in range). Call
    /// before creating the [`DistanceSensor`](crate::DistanceSensor).
    pub fn trigger_pin(&self, trigger: u32, echo: u32, distance: Option<f64>) -> Arc<MockPin> {
        let echo_pin = self.mock_pin(echo);
        let p = self.get_or_create(trigger, || Behavior::Trigger {
            echo: Arc::downgrade(&echo_pin),
            echo_time: None,
        });
        p.set_distance(distance);
        p
    }

    /// Attaches a simulated SPI device. For [`SpiConfig::Hardware`] the device
    /// answers [`PinFactory::spi`] directly; for [`SpiConfig::Software`] it is
    /// wired to the mock pins and driven by a real [`SoftwareSpi`].
    pub fn attach_spi_device(&self, config: &SpiConfig, device: Arc<dyn MockSpiDevice>) {
        match *config {
            SpiConfig::Hardware {
                port, device: dev, ..
            } => {
                lock(&self.spi).insert((port, dev), device);
            }
            SpiConfig::Software {
                clock,
                mosi,
                miso,
                select,
            } => {
                let bits: Arc<Mutex<Vec<bool>>> = Arc::default();
                let b = bits.clone();
                self.mock_pin(select)
                    .set_write_hook(Some(Arc::new(move |_, new| {
                        if new < 0.5 {
                            lock(&b).clear();
                        }
                    })));
                let mosi_pin = mosi.map(|n| Arc::downgrade(&self.mock_pin(n)));
                let miso_pin = miso.map(|n| Arc::downgrade(&self.mock_pin(n)));
                self.mock_pin(clock)
                    .set_write_hook(Some(Arc::new(move |old, new| {
                        if old >= 0.5 || new < 0.5 {
                            return;
                        }
                        let bit = mosi_pin
                            .as_ref()
                            .and_then(Weak::upgrade)
                            .is_some_and(|p| p.value() >= 0.5);
                        let out = {
                            let mut v = lock(&bits);
                            v.push(bit);
                            device
                                .respond(&v)
                                .get(v.len() - 1)
                                .copied()
                                .unwrap_or(false)
                        };
                        if let Some(p) = miso_pin.as_ref().and_then(Weak::upgrade) {
                            p.drive(out);
                        }
                    })));
            }
        }
    }

    /// Moves virtual time forward (sleeps for real with a real-time factory).
    pub fn advance(&self, duration: Duration) {
        match &self.kind {
            MockClockKind::Virtual(c) => c.advance(duration),
            MockClockKind::Real => std::thread::sleep(duration),
        }
    }

    /// Waits until background tasks are idle (virtual time only).
    pub fn settle(&self) {
        if let MockClockKind::Virtual(c) = &self.kind {
            c.settle();
        }
    }

    /// Current clock time.
    pub fn now(&self) -> Duration {
        self.clock.now()
    }

    /// The virtual clock, if this factory uses one.
    pub fn mock_clock(&self) -> Option<&MockClock> {
        match &self.kind {
            MockClockKind::Virtual(c) => Some(c),
            MockClockKind::Real => None,
        }
    }

    /// Forgets all pins, reservations and SPI devices.
    pub fn reset(&self) {
        lock(&self.pins).clear();
        lock(&self.spi).clear();
        *lock(&self.reservations.0) = Default::default();
    }
}

impl PinFactory for MockFactory {
    fn name(&self) -> &str {
        "mock"
    }

    fn reserve_pin(&self, number: u32) -> Result<Arc<dyn Pin>> {
        self.reservations.reserve(number)?;
        Ok(self.mock_pin(number))
    }

    fn release_pin(&self, number: u32) {
        self.reservations.release(number);
    }

    fn spi(&self, config: &SpiConfig) -> Result<Box<dyn SpiInterface>> {
        match *config {
            SpiConfig::Hardware { port, device, .. } => lock(&self.spi)
                .get(&(port, device))
                .cloned()
                .map(|d| Box::new(MockHardwareSpi(d)) as Box<dyn SpiInterface>)
                .ok_or_else(|| {
                    Error::SpiUnsupported(format!(
                        "no mock SPI device attached at port {port} device {device}"
                    ))
                }),
            SpiConfig::Software {
                clock,
                mosi,
                miso,
                select,
            } => {
                let me: Arc<dyn PinFactory> = self.me.upgrade().ok_or(Error::DeviceClosed)?;
                Ok(Box::new(SoftwareSpi::new(&me, clock, mosi, miso, select)?))
            }
        }
    }

    fn clock(&self) -> Arc<dyn Clock> {
        self.clock.clone()
    }
}
