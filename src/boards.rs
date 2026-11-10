//! Composite devices: LED boards, bar graphs, traffic lights, button boards
//! and robots.

use std::fmt;
use std::ops::Deref;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::clock::{Clock, Event, Task, lock};
use crate::devices::{
    Blink, Callback, SourceSlot, Values, animate, blink_frames, check_signed, check_unit, stop_task,
};
use crate::error::{Result, invalid};
use crate::input::{Button, ButtonOptions, DigitalShared};
use crate::output::{Led, Motor, MotorOptions, OutputInner, PwmLed};
use crate::pins::{PinFactory, resolve};

/// Options for [`LedBoard`], [`LedBarGraph`] and [`TrafficLights`].
#[derive(Clone)]
pub struct LedBoardOptions {
    /// Use PWM LEDs (default `false`).
    pub pwm: bool,
    /// `true` (default): high = lit.
    pub active_high: bool,
    /// Initial value of every LED (default `0.0`).
    pub initial_value: f64,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for LedBoardOptions {
    fn default() -> Self {
        LedBoardOptions {
            pwm: false,
            active_high: true,
            initial_value: 0.0,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for LedBoardOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LedBoardOptions")
            .field("pwm", &self.pwm)
            .field("active_high", &self.active_high)
            .field("initial_value", &self.initial_value)
            .finish_non_exhaustive()
    }
}

struct BoardInner {
    leds: Vec<Arc<OutputInner>>,
    pwm: bool,
    clock: Arc<dyn Clock>,
    blink: Mutex<Option<Task>>,
    source: SourceSlot,
}

impl BoardInner {
    fn check(&self, v: f64) -> Result<f64> {
        let v = check_unit(v, "LED value")?;
        if !self.pwm && v != 0.0 && v != 1.0 {
            return invalid("non-PWM boards only accept 0 or 1");
        }
        Ok(v)
    }

    fn set_all(&self, values: &[f64]) -> Result<()> {
        if values.len() != self.leds.len() {
            return invalid(format!(
                "expected {} values, got {}",
                self.leds.len(),
                values.len()
            ));
        }
        for v in values {
            self.check(*v)?;
        }
        stop_task(&self.blink);
        for (led, v) in self.leds.iter().zip(values) {
            led.set(*v)?;
        }
        Ok(())
    }

    fn read(&self) -> Vec<f64> {
        self.leds.iter().map(|l| l.read()).collect()
    }
}

impl Drop for BoardInner {
    fn drop(&mut self) {
        self.source.stop();
        stop_task(&self.blink);
    }
}

/// A collection of LEDs controlled together.
///
/// ```no_run
/// use gpio_zero::LedBoard;
/// let board = LedBoard::new(&[5, 6, 13, 19, 26])?;
/// board.on()?;
/// board.set_value(&[1.0, 0.0, 1.0, 0.0, 1.0])?;
/// # Ok::<(), gpio_zero::Error>(())
/// ```
#[derive(Clone)]
pub struct LedBoard {
    inner: Arc<BoardInner>,
}

impl fmt::Debug for LedBoard {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LedBoard")
            .field("value", &self.value())
            .finish()
    }
}

impl LedBoard {
    /// Creates a board of digital LEDs on `pins`.
    pub fn new(pins: &[u32]) -> Result<Self> {
        Self::with_options(pins, LedBoardOptions::default())
    }

    /// Creates a board with explicit options.
    pub fn with_options(pins: &[u32], o: LedBoardOptions) -> Result<Self> {
        if pins.is_empty() {
            return invalid("an LED board needs at least one pin");
        }
        let factory = resolve(&o.pin_factory)?;
        let freq = o.pwm.then_some(100.0);
        let init = check_unit(o.initial_value, "initial_value")?;
        if !o.pwm && init != 0.0 && init != 1.0 {
            return invalid("non-PWM boards only accept 0 or 1");
        }
        let leds = pins
            .iter()
            .map(|p| OutputInner::new(&factory, *p, o.active_high, freq, Some(init)))
            .collect::<Result<Vec<_>>>()?;
        Ok(LedBoard {
            inner: Arc::new(BoardInner {
                leds,
                pwm: o.pwm,
                clock: factory.clock(),
                blink: Mutex::new(None),
                source: SourceSlot::default(),
            }),
        })
    }

    /// Number of LEDs.
    pub fn len(&self) -> usize {
        self.inner.leds.len()
    }

    /// Always `false` (boards have at least one LED).
    pub fn is_empty(&self) -> bool {
        self.inner.leds.is_empty()
    }

    /// Whether the LEDs are PWM.
    pub fn pwm(&self) -> bool {
        self.inner.pwm
    }

    /// A handle to LED `index` with the on/off API (works on PWM boards too).
    pub fn led(&self, index: usize) -> Option<Led> {
        self.inner.leds.get(index).map(|i| Led { inner: i.clone() })
    }

    /// A handle to LED `index` with the brightness API (PWM boards only).
    pub fn pwm_led(&self, index: usize) -> Option<PwmLed> {
        if !self.inner.pwm {
            return None;
        }
        self.inner
            .leds
            .get(index)
            .map(|i| PwmLed { inner: i.clone() })
    }

    /// Turns every LED on.
    pub fn on(&self) -> Result<()> {
        self.inner.set_all(&vec![1.0; self.len()])
    }

    /// Turns every LED off.
    pub fn off(&self) -> Result<()> {
        self.inner.set_all(&vec![0.0; self.len()])
    }

    /// Toggles every LED (`1 - value`).
    pub fn toggle(&self) -> Result<()> {
        let v: Vec<f64> = self.value().iter().map(|v| 1.0 - v).collect();
        self.inner.set_all(&v)
    }

    fn update(&self, indices: &[usize], f: impl Fn(f64) -> f64) -> Result<()> {
        let mut v = self.value();
        for &i in indices {
            match v.get_mut(i) {
                Some(x) => *x = f(*x),
                None => return invalid(format!("LED index {i} out of range")),
            }
        }
        self.inner.set_all(&v)
    }

    /// Turns on only the LEDs at `indices` (others unchanged).
    pub fn on_at(&self, indices: &[usize]) -> Result<()> {
        self.update(indices, |_| 1.0)
    }

    /// Turns off the LEDs at `indices`.
    pub fn off_at(&self, indices: &[usize]) -> Result<()> {
        self.update(indices, |_| 0.0)
    }

    /// Toggles the LEDs at `indices`.
    pub fn toggle_at(&self, indices: &[usize]) -> Result<()> {
        self.update(indices, |x| 1.0 - x)
    }

    /// Every LED's value.
    pub fn value(&self) -> Vec<f64> {
        self.inner.read()
    }

    /// Sets every LED's value (`values.len()` must equal `len()`).
    pub fn set_value(&self, values: &[f64]) -> Result<()> {
        self.inner.set_all(values)
    }

    /// `true` if any LED is lit.
    pub fn is_active(&self) -> bool {
        self.value().iter().any(|v| *v > 0.0)
    }

    /// Alias of [`is_active`](Self::is_active).
    pub fn is_lit(&self) -> bool {
        self.is_active()
    }

    /// Blinks all LEDs together.
    pub fn blink(
        &self,
        on_time: Duration,
        off_time: Duration,
        n: Option<u32>,
        background: bool,
    ) -> Result<()> {
        self.blink_with(Blink {
            on_time,
            off_time,
            n,
            background,
            ..Blink::default()
        })
    }

    /// Blinks with a full [`Blink`] spec (fades need `pwm`).
    pub fn blink_with(&self, spec: Blink) -> Result<()> {
        if spec.has_fade() && !self.inner.pwm {
            return invalid("fades require pwm = true");
        }
        let n = self.len();
        let frames = blink_frames(&spec, &vec![1.0; n], &vec![0.0; n]);
        for l in &self.inner.leds {
            stop_task(&l.blink);
        }
        let writers = self.inner.leds.iter().map(|l| l.writer()).collect();
        animate(
            &self.inner.blink,
            &self.inner.clock,
            writers,
            frames,
            spec.n,
            spec.background,
        );
        Ok(())
    }

    /// Pulses all LEDs together (PWM boards only).
    pub fn pulse(
        &self,
        fade_in: Duration,
        fade_out: Duration,
        n: Option<u32>,
        background: bool,
    ) -> Result<()> {
        self.blink_with(Blink {
            n,
            background,
            ..Blink::pulse(fade_in, fade_out)
        })
    }

    /// An endless iterator over all values.
    pub fn values(&self) -> Values<Vec<f64>> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || weak.upgrade().map(|i| i.read()))
    }

    /// Continuously sets all values from `source`.
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator<Item = Vec<f64>>,
        I::IntoIter: Send + 'static,
    {
        let weak = Arc::downgrade(&self.inner);
        self.inner
            .source
            .start(&self.inner.clock, source, move |v| match weak.upgrade() {
                Some(i) => {
                    let _ = i.set_all(&v);
                    true
                }
                None => false,
            });
    }

    /// Stops the `source` loop.
    pub fn clear_source(&self) {
        self.inner.source.stop();
    }

    /// GPIO numbers, in order.
    pub fn pins(&self) -> Vec<u32> {
        self.inner.leds.iter().map(|l| l.pin.number()).collect()
    }
}

/// A row of LEDs showing a proportion, like a bar graph.
///
/// `value` runs from `-1.0` to `1.0`; positive values light LEDs from the
/// first pin, negative from the last.
#[derive(Clone, Debug)]
pub struct LedBarGraph {
    board: LedBoard,
}

impl LedBarGraph {
    /// Creates a bar graph on `pins`.
    pub fn new(pins: &[u32]) -> Result<Self> {
        Self::with_options(pins, LedBoardOptions::default())
    }

    /// Creates a bar graph with explicit options (`initial_value` is a
    /// bar-graph value, `-1.0..=1.0`).
    pub fn with_options(pins: &[u32], o: LedBoardOptions) -> Result<Self> {
        let init = check_signed(o.initial_value, "initial_value")?;
        let g = LedBarGraph {
            board: LedBoard::with_options(
                pins,
                LedBoardOptions {
                    initial_value: 0.0,
                    ..o
                },
            )?,
        };
        g.set_value(init)?;
        Ok(g)
    }

    /// The displayed proportion.
    pub fn value(&self) -> f64 {
        bar_value(&self.board.value())
    }

    /// Displays `value` (`-1.0..=1.0`).
    pub fn set_value(&self, value: f64) -> Result<()> {
        let v = check_signed(value, "bar graph value")?;
        self.board
            .set_value(&bar_levels(v, self.board.len(), self.board.pwm()))
    }

    /// Number of LEDs lit (fractional with PWM; negative from the end).
    pub fn lit_count(&self) -> f64 {
        self.value() * self.board.len() as f64
    }

    /// Lights `count` LEDs.
    pub fn set_lit_count(&self, count: f64) -> Result<()> {
        self.set_value(count / self.board.len() as f64)
    }

    /// An endless iterator over the bar-graph value.
    pub fn values(&self) -> Values<f64> {
        let mut inner = self.board.values();
        Values::from_fn(move || inner.next().map(|v| bar_value(&v)))
    }

    /// Continuously displays values from `source` (`-1.0..=1.0`).
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator,
        I::Item: Into<f64> + 'static,
        I::IntoIter: Send + 'static,
    {
        let (n, pwm) = (self.board.len(), self.board.pwm());
        self.board.set_source(
            source
                .into_iter()
                .map(Into::into)
                .filter(|v: &f64| (-1.0..=1.0).contains(v))
                .map(move |v| bar_levels(v, n, pwm)),
        );
    }
}

fn bar_levels(value: f64, n: usize, pwm: bool) -> Vec<f64> {
    let count = value.abs() * n as f64;
    let mut v: Vec<f64> = (0..n)
        .map(|i| {
            if pwm {
                (count - i as f64).clamp(0.0, 1.0)
            } else if value.abs() >= (i + 1) as f64 / n as f64 - 1e-9 {
                1.0
            } else {
                0.0
            }
        })
        .collect();
    if value < 0.0 {
        v.reverse();
    }
    v
}

fn bar_value(v: &[f64]) -> f64 {
    let n = v.len() as f64;
    let sum: f64 = v.iter().sum();
    let neg = v.first().copied().unwrap_or(0.0) < v.last().copied().unwrap_or(0.0);
    if neg { -sum / n } else { sum / n }
}

impl Deref for LedBarGraph {
    type Target = LedBoard;
    fn deref(&self) -> &LedBoard {
        &self.board
    }
}

/// Red, amber and green traffic lights.
///
/// ```no_run
/// use gpio_zero::TrafficLights;
/// let lights = TrafficLights::new(2, 3, 4)?;
/// lights.green().on()?;
/// # Ok::<(), gpio_zero::Error>(())
/// ```
#[derive(Clone, Debug)]
pub struct TrafficLights {
    board: LedBoard,
}

impl TrafficLights {
    /// Creates traffic lights on the given pins.
    pub fn new(red: u32, amber: u32, green: u32) -> Result<Self> {
        Self::with_options(red, amber, green, LedBoardOptions::default())
    }

    /// Creates traffic lights with explicit options.
    pub fn with_options(red: u32, amber: u32, green: u32, o: LedBoardOptions) -> Result<Self> {
        Ok(TrafficLights {
            board: LedBoard::with_options(&[red, amber, green], o)?,
        })
    }

    /// The red light.
    pub fn red(&self) -> Led {
        self.board.led(0).expect("three LEDs")
    }

    /// The amber light.
    pub fn amber(&self) -> Led {
        self.board.led(1).expect("three LEDs")
    }

    /// Alias of [`amber`](Self::amber).
    pub fn yellow(&self) -> Led {
        self.amber()
    }

    /// The green light.
    pub fn green(&self) -> Led {
        self.board.led(2).expect("three LEDs")
    }
}

impl Deref for TrafficLights {
    type Target = LedBoard;
    fn deref(&self) -> &LedBoard {
        &self.board
    }
}

struct ButtonBoardShared {
    buttons: Vec<Arc<DigitalShared>>,
    any: Mutex<bool>,
    pressed: Event,
    released: Event,
    when_pressed: Mutex<Option<Callback>>,
    when_released: Mutex<Option<Callback>>,
    when_changed: Mutex<Option<Callback>>,
}

impl ButtonBoardShared {
    fn on_change(&self) {
        let any = self.buttons.iter().any(|b| b.is_active());
        let prev = std::mem::replace(&mut *lock(&self.any), any);
        if any {
            self.released.clear();
            self.pressed.set();
        } else {
            self.pressed.clear();
            self.released.set();
        }
        let mut cbs = vec![lock(&self.when_changed).clone()];
        if any != prev {
            cbs.push(if any {
                lock(&self.when_pressed).clone()
            } else {
                lock(&self.when_released).clone()
            });
        }
        for cb in cbs.into_iter().flatten() {
            cb();
        }
    }
}

/// A collection of buttons.
#[derive(Clone)]
pub struct ButtonBoard {
    buttons: Vec<Button>,
    shared: Arc<ButtonBoardShared>,
    clock: Arc<dyn Clock>,
}

impl fmt::Debug for ButtonBoard {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ButtonBoard")
            .field("value", &self.value())
            .finish()
    }
}

impl ButtonBoard {
    /// Creates a board of pull-up buttons on `pins`.
    pub fn new(pins: &[u32]) -> Result<Self> {
        Self::with_options(pins, ButtonOptions::default())
    }

    /// Creates a board; every button uses `options`.
    pub fn with_options(pins: &[u32], options: ButtonOptions) -> Result<Self> {
        if pins.is_empty() {
            return invalid("a button board needs at least one pin");
        }
        let factory = resolve(&options.pin_factory)?;
        let o = ButtonOptions {
            pin_factory: Some(factory.clone()),
            ..options
        };
        let buttons = pins
            .iter()
            .map(|p| Button::with_options(*p, o.clone()))
            .collect::<Result<Vec<_>>>()?;
        let any = buttons.iter().any(Button::is_pressed);
        let shared = Arc::new(ButtonBoardShared {
            buttons: buttons.iter().map(|b| b.inner.shared.clone()).collect(),
            any: Mutex::new(any),
            pressed: Event::new(),
            released: Event::new(),
            when_pressed: Mutex::new(None),
            when_released: Mutex::new(None),
            when_changed: Mutex::new(None),
        });
        if any {
            shared.pressed.set();
        } else {
            shared.released.set();
        }
        for b in &buttons {
            let weak = Arc::downgrade(&shared);
            lock(&b.inner.shared.listeners).push(Arc::new(move |_| {
                if let Some(s) = weak.upgrade() {
                    s.on_change();
                }
            }));
        }
        Ok(ButtonBoard {
            buttons,
            shared,
            clock: factory.clock(),
        })
    }

    /// Number of buttons.
    pub fn len(&self) -> usize {
        self.buttons.len()
    }

    /// Always `false`.
    pub fn is_empty(&self) -> bool {
        self.buttons.is_empty()
    }

    /// Button `index`.
    pub fn button(&self, index: usize) -> Option<&Button> {
        self.buttons.get(index)
    }

    /// All buttons.
    pub fn buttons(&self) -> &[Button] {
        &self.buttons
    }

    /// Each button's pressed state.
    pub fn value(&self) -> Vec<bool> {
        self.buttons.iter().map(Button::is_pressed).collect()
    }

    /// `true` if any button is pressed.
    pub fn is_pressed(&self) -> bool {
        self.buttons.iter().any(Button::is_pressed)
    }

    /// Alias of [`is_pressed`](Self::is_pressed).
    pub fn is_active(&self) -> bool {
        self.is_pressed()
    }

    /// Runs `f` when the first button is pressed (none → some).
    pub fn when_pressed(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.shared.when_pressed) = Some(Arc::new(f));
    }

    /// Runs `f` when the last button is released (some → none).
    pub fn when_released(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.shared.when_released) = Some(Arc::new(f));
    }

    /// Runs `f` whenever any button changes.
    pub fn when_changed(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.shared.when_changed) = Some(Arc::new(f));
    }

    /// Blocks until any button is pressed (or `timeout`).
    pub fn wait_for_press(&self, timeout: Option<Duration>) -> bool {
        self.clock.wait(&self.shared.pressed, timeout)
    }

    /// Blocks until all buttons are released (or `timeout`).
    pub fn wait_for_release(&self, timeout: Option<Duration>) -> bool {
        self.clock.wait(&self.shared.released, timeout)
    }

    /// An endless iterator over every button's state.
    pub fn values(&self) -> Values<Vec<bool>> {
        let weak = Arc::downgrade(&self.shared);
        Values::from_fn(move || {
            weak.upgrade()
                .map(|s| s.buttons.iter().map(|b| b.is_active()).collect())
        })
    }
}

/// Options for [`Robot`].
#[derive(Clone)]
pub struct RobotOptions {
    /// Use PWM speed control (default `true`).
    pub pwm: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for RobotOptions {
    fn default() -> Self {
        RobotOptions {
            pwm: true,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for RobotOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RobotOptions")
            .field("pwm", &self.pwm)
            .finish_non_exhaustive()
    }
}

struct RobotInner {
    left: Motor,
    right: Motor,
    clock: Arc<dyn Clock>,
    source: SourceSlot,
}

impl RobotInner {
    fn set(&self, (l, r): (f64, f64)) -> Result<()> {
        check_signed(l, "left motor value")?;
        check_signed(r, "right motor value")?;
        self.left.set_value(l)?;
        self.right.set_value(r)
    }
}

impl Drop for RobotInner {
    fn drop(&mut self) {
        self.source.stop();
    }
}

/// A two-wheeled robot: two [`Motor`]s.
///
/// ```no_run
/// use gpio_zero::Robot;
/// let robot = Robot::new((4, 14), (17, 18))?;
/// robot.forward(1.0)?;
/// robot.right(0.5)?;
/// robot.stop()?;
/// # Ok::<(), gpio_zero::Error>(())
/// ```
#[derive(Clone)]
pub struct Robot {
    inner: Arc<RobotInner>,
}

impl fmt::Debug for Robot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Robot")
            .field("value", &self.value())
            .finish()
    }
}

impl Robot {
    /// Creates a robot from `(forward, backward)` pin pairs for each side.
    pub fn new(left: (u32, u32), right: (u32, u32)) -> Result<Self> {
        Self::with_options(left, right, RobotOptions::default())
    }

    /// Creates a robot with explicit options.
    pub fn with_options(left: (u32, u32), right: (u32, u32), o: RobotOptions) -> Result<Self> {
        let factory = resolve(&o.pin_factory)?;
        let mo = MotorOptions {
            enable: None,
            pwm: o.pwm,
            pin_factory: Some(factory.clone()),
        };
        Ok(Robot {
            inner: Arc::new(RobotInner {
                left: Motor::with_options(left.0, left.1, mo.clone())?,
                right: Motor::with_options(right.0, right.1, mo)?,
                clock: factory.clock(),
                source: SourceSlot::default(),
            }),
        })
    }

    /// The left motor.
    pub fn left_motor(&self) -> &Motor {
        &self.inner.left
    }

    /// The right motor.
    pub fn right_motor(&self) -> &Motor {
        &self.inner.right
    }

    /// Drives forward at `speed`.
    pub fn forward(&self, speed: f64) -> Result<()> {
        self.forward_curved(speed, 0.0, 0.0)
    }

    /// Drives forward, slowing one side by `curve_left` / `curve_right`
    /// (`0.0..=1.0`, at most one non-zero) to steer.
    pub fn forward_curved(&self, speed: f64, curve_left: f64, curve_right: f64) -> Result<()> {
        let (l, r) = curved(speed, curve_left, curve_right)?;
        self.inner.set((l, r))
    }

    /// Drives backward at `speed`.
    pub fn backward(&self, speed: f64) -> Result<()> {
        self.backward_curved(speed, 0.0, 0.0)
    }

    /// Drives backward with curving (see [`forward_curved`](Self::forward_curved)).
    pub fn backward_curved(&self, speed: f64, curve_left: f64, curve_right: f64) -> Result<()> {
        let (l, r) = curved(speed, curve_left, curve_right)?;
        self.inner.set((-l, -r))
    }

    /// Spins left on the spot.
    pub fn left(&self, speed: f64) -> Result<()> {
        let s = check_unit(speed, "speed")?;
        self.inner.set((-s, s))
    }

    /// Spins right on the spot.
    pub fn right(&self, speed: f64) -> Result<()> {
        let s = check_unit(speed, "speed")?;
        self.inner.set((s, -s))
    }

    /// Reverses both motors.
    pub fn reverse(&self) -> Result<()> {
        let (l, r) = self.value();
        self.inner.set((-l, -r))
    }

    /// Stops both motors.
    pub fn stop(&self) -> Result<()> {
        self.inner.set((0.0, 0.0))
    }

    /// `(left, right)` velocities.
    pub fn value(&self) -> (f64, f64) {
        (self.inner.left.value(), self.inner.right.value())
    }

    /// Sets `(left, right)` velocities.
    pub fn set_value(&self, value: (f64, f64)) -> Result<()> {
        self.inner.set(value)
    }

    /// `true` if either motor is moving.
    pub fn is_active(&self) -> bool {
        self.inner.left.is_active() || self.inner.right.is_active()
    }

    /// An endless iterator over `(left, right)`.
    pub fn values(&self) -> Values<(f64, f64)> {
        let weak = Arc::downgrade(&self.inner);
        Values::from_fn(move || weak.upgrade().map(|i| (i.left.value(), i.right.value())))
    }

    /// Continuously sets `(left, right)` from `source`.
    pub fn set_source<I>(&self, source: I)
    where
        I: IntoIterator<Item = (f64, f64)>,
        I::IntoIter: Send + 'static,
    {
        let weak = Arc::downgrade(&self.inner);
        self.inner
            .source
            .start(&self.inner.clock, source, move |v| match weak.upgrade() {
                Some(i) => {
                    let _ = i.set(v);
                    true
                }
                None => false,
            });
    }

    /// Stops the `source` loop.
    pub fn clear_source(&self) {
        self.inner.source.stop();
    }
}

fn curved(speed: f64, curve_left: f64, curve_right: f64) -> Result<(f64, f64)> {
    let s = check_unit(speed, "speed")?;
    let cl = check_unit(curve_left, "curve_left")?;
    let cr = check_unit(curve_right, "curve_right")?;
    if cl != 0.0 && cr != 0.0 {
        return invalid("curve_left and curve_right cannot both be set");
    }
    Ok((s * (1.0 - cl), s * (1.0 - cr)))
}
