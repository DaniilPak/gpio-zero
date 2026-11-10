//! Input devices: buttons, sensors and encoders.

use std::fmt;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use crate::clock::{Clock, Event, Task, lock};
use crate::devices::{Callback, Values};
use crate::error::{Result, invalid};
use crate::pins::{Edges, PinFactory, PinFunction, PinHandle, Pull, resolve};

mod distance;
mod rotary;
mod smoothed;

pub use distance::{DistanceSensor, DistanceSensorOptions};
pub use rotary::{RotaryEncoder, RotaryEncoderOptions};
pub use smoothed::{
    LightSensor, LightSensorOptions, LineSensor, MotionSensor, SmoothedInputOptions,
};

/// Speed of sound in air at 20 °C, m/s (used by [`DistanceSensor`]).
pub const SPEED_OF_SOUND: f64 = 343.26;

pub(crate) type Listener = Arc<dyn Fn(bool) + Send + Sync>;

/// Works out the pull resistor and active level from gpiozero-style
/// `pull_up` / `active_state` arguments.
pub(crate) fn resolve_pull(
    pull_up: Option<bool>,
    active_state: Option<bool>,
) -> Result<(Pull, bool)> {
    match (pull_up, active_state) {
        (Some(true), None) => Ok((Pull::Up, false)),
        (Some(false), None) => Ok((Pull::Down, true)),
        (None, Some(s)) => Ok((Pull::Floating, s)),
        (None, None) => invalid("active_state must be given when pull_up is None"),
        (Some(_), Some(_)) => invalid("active_state cannot be given together with pull_up"),
    }
}

fn fire(cb: &Mutex<Option<Callback>>) {
    let cb = lock(cb).clone();
    if let Some(cb) = cb {
        cb();
    }
}

struct Hold {
    hold_time: Mutex<Duration>,
    repeat: Mutex<bool>,
    task: Mutex<Option<Task>>,
    held_since: Mutex<Option<Duration>>,
    when_held: Mutex<Option<Callback>>,
}

struct DState {
    active: bool,
    changed_at: Option<Duration>,
}

/// State shared between a digital input device and its pin callback.
pub(crate) struct DigitalShared {
    clock: Arc<dyn Clock>,
    active_state: bool,
    st: Mutex<DState>,
    activated: Event,
    deactivated: Event,
    when_activated: Mutex<Option<Callback>>,
    when_deactivated: Mutex<Option<Callback>>,
    pub(crate) listeners: Mutex<Vec<Listener>>,
    hold: Option<Hold>,
}

impl DigitalShared {
    pub(crate) fn is_active(&self) -> bool {
        lock(&self.st).active
    }

    fn on_edge(self: &Arc<Self>, ticks: Duration, level: bool) {
        let active = level == self.active_state;
        {
            let mut st = lock(&self.st);
            if st.active == active {
                return;
            }
            st.active = active;
            st.changed_at = Some(ticks);
        }
        if active {
            self.deactivated.clear();
            self.activated.set();
            self.start_hold();
            fire(&self.when_activated);
        } else {
            self.activated.clear();
            self.deactivated.set();
            self.stop_hold();
            fire(&self.when_deactivated);
        }
        let listeners = lock(&self.listeners).clone();
        for l in listeners {
            l(active);
        }
    }

    fn start_hold(self: &Arc<Self>) {
        let Some(hold) = &self.hold else { return };
        let weak: Weak<Self> = Arc::downgrade(self);
        let clock = self.clock.clone();
        let hold_time = *lock(&hold.hold_time);
        let task = Task::spawn(&self.clock, "gpio-zero-hold", move |stop| {
            loop {
                if clock.wait(&stop, Some(hold_time)) {
                    return;
                }
                let Some(sh) = weak.upgrade() else { return };
                if !sh.is_active() {
                    return;
                }
                let Some(hold) = &sh.hold else { return };
                {
                    let mut since = lock(&hold.held_since);
                    if since.is_none() {
                        *since = Some(clock.now());
                    }
                }
                fire(&hold.when_held);
                if !*lock(&hold.repeat) {
                    return;
                }
            }
        });
        let old = lock(&hold.task).replace(task);
        drop(old);
    }

    fn stop_hold(&self) {
        if let Some(hold) = &self.hold {
            *lock(&hold.held_since) = None;
            let old = lock(&hold.task).take();
            drop(old);
        }
    }
}

/// The pin plus shared state of a digital input.
pub(crate) struct DigitalInner {
    pub(crate) shared: Arc<DigitalShared>,
    pub(crate) pin: PinHandle,
    pull: Pull,
}

impl DigitalInner {
    pub(crate) fn new(
        factory: &Arc<dyn PinFactory>,
        pin: u32,
        pull_up: Option<bool>,
        active_state: Option<bool>,
        bounce_time: Option<Duration>,
        hold: Option<(Duration, bool)>,
    ) -> Result<Arc<Self>> {
        let (pull, active_state) = resolve_pull(pull_up, active_state)?;
        let handle = PinHandle::new(factory, pin)?;
        handle.set_function(PinFunction::Input)?;
        handle.set_pull(pull)?;
        handle.set_bounce(bounce_time)?;
        handle.set_edges(Edges::Both)?;
        let level = handle.state()? >= 0.5;
        let shared = Arc::new(DigitalShared {
            clock: factory.clock(),
            active_state,
            st: Mutex::new(DState {
                active: level == active_state,
                changed_at: None,
            }),
            activated: Event::new(),
            deactivated: Event::new(),
            when_activated: Mutex::new(None),
            when_deactivated: Mutex::new(None),
            listeners: Mutex::new(Vec::new()),
            hold: hold.map(|(t, r)| Hold {
                hold_time: Mutex::new(t),
                repeat: Mutex::new(r),
                task: Mutex::new(None),
                held_since: Mutex::new(None),
                when_held: Mutex::new(None),
            }),
        });
        if level == active_state {
            shared.activated.set();
        } else {
            shared.deactivated.set();
        }
        let s = shared.clone();
        handle.set_when_changed(Some(Arc::new(move |ticks, level| s.on_edge(ticks, level))))?;
        Ok(Arc::new(DigitalInner {
            shared,
            pin: handle,
            pull,
        }))
    }

    fn active_time(&self) -> Option<Duration> {
        let st = lock(&self.shared.st);
        st.active.then(|| {
            self.shared
                .clock
                .now()
                .saturating_sub(st.changed_at.unwrap_or_default())
        })
    }

    fn inactive_time(&self) -> Option<Duration> {
        let st = lock(&self.shared.st);
        (!st.active).then(|| {
            self.shared
                .clock
                .now()
                .saturating_sub(st.changed_at.unwrap_or_default())
        })
    }

    fn wait(&self, active: bool, timeout: Option<Duration>) -> bool {
        let ev = if active {
            &self.shared.activated
        } else {
            &self.shared.deactivated
        };
        self.shared.clock.wait(ev, timeout)
    }

    fn values(self: &Arc<Self>) -> Values<bool> {
        let weak = Arc::downgrade(self);
        Values::from_fn(move || weak.upgrade().map(|i| i.shared.is_active()))
    }
}

impl Drop for DigitalInner {
    fn drop(&mut self) {
        let _ = self.pin.set_when_changed(None);
        self.shared.stop_hold();
    }
}

fn boxed(f: impl Fn() + Send + Sync + 'static) -> Option<Callback> {
    Some(Arc::new(f))
}

/// Options for [`InputDevice`] and [`DigitalInputDevice`].
#[derive(Clone)]
pub struct InputOptions {
    /// `Some(true)`: internal pull-up, active when low. `Some(false)`
    /// (default): pull-down, active when high. `None`: floating; then
    /// `active_state` is required.
    pub pull_up: Option<bool>,
    /// With `pull_up = None`: `true` = active when high.
    pub active_state: Option<bool>,
    /// Debounce period (ignore edges closer together than this).
    pub bounce_time: Option<Duration>,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for InputOptions {
    fn default() -> Self {
        InputOptions {
            pull_up: Some(false),
            active_state: None,
            bounce_time: None,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for InputOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InputOptions")
            .field("pull_up", &self.pull_up)
            .field("active_state", &self.active_state)
            .field("bounce_time", &self.bounce_time)
            .finish_non_exhaustive()
    }
}

macro_rules! digital_input_common {
    () => {
        /// The GPIO number.
        pub fn pin(&self) -> u32 {
            self.inner.pin.number()
        }

        /// `true` if the internal pull-up is used.
        pub fn pull_up(&self) -> bool {
            self.inner.pull == Pull::Up
        }

        /// `true` if the device is active.
        pub fn is_active(&self) -> bool {
            self.inner.shared.is_active()
        }

        /// `true` if the device is active.
        pub fn value(&self) -> bool {
            self.is_active()
        }

        /// An endless iterator over the active state.
        pub fn values(&self) -> Values<bool> {
            self.inner.values()
        }

        /// Closes the device, releasing the pin.
        pub fn close(self) {}
    };
}

/// A generic digital input (gpiozero's `InputDevice`).
#[derive(Clone)]
pub struct InputDevice {
    inner: Arc<DigitalInner>,
}

impl fmt::Debug for InputDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InputDevice")
            .field("pin", &self.pin())
            .field("is_active", &self.is_active())
            .finish()
    }
}

impl InputDevice {
    /// Creates an input on GPIO `pin` (pull-down, active high).
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, InputOptions::default())
    }

    /// Creates an input with explicit options.
    pub fn with_options(pin: u32, o: InputOptions) -> Result<Self> {
        let factory = resolve(&o.pin_factory)?;
        Ok(InputDevice {
            inner: DigitalInner::new(
                &factory,
                pin,
                o.pull_up,
                o.active_state,
                o.bounce_time,
                None,
            )?,
        })
    }

    digital_input_common!();
}

macro_rules! digital_input_events {
    ($act:ident, $deact:ident, $wact:ident, $wdeact:ident, $what:literal) => {
        #[doc = concat!("Runs `f` (in the pin's event thread) when ", $what, " becomes active.")]
        pub fn $act(&self, f: impl Fn() + Send + Sync + 'static) {
            *lock(&self.inner.shared.when_activated) = boxed(f);
        }

        #[doc = concat!("Runs `f` when ", $what, " becomes inactive.")]
        pub fn $deact(&self, f: impl Fn() + Send + Sync + 'static) {
            *lock(&self.inner.shared.when_deactivated) = boxed(f);
        }

        #[doc = concat!("Blocks until ", $what, " is active (or `timeout`). Returns `false` on timeout.")]
        pub fn $wact(&self, timeout: Option<Duration>) -> bool {
            self.inner.wait(true, timeout)
        }

        #[doc = concat!("Blocks until ", $what, " is inactive (or `timeout`).")]
        pub fn $wdeact(&self, timeout: Option<Duration>) -> bool {
            self.inner.wait(false, timeout)
        }

        /// Removes all event callbacks.
        pub fn clear_callbacks(&self) {
            *lock(&self.inner.shared.when_activated) = None;
            *lock(&self.inner.shared.when_deactivated) = None;
            if let Some(h) = &self.inner.shared.hold {
                *lock(&h.when_held) = None;
            }
        }

        /// How long the device has been active (`None` if inactive).
        pub fn active_time(&self) -> Option<Duration> {
            self.inner.active_time()
        }

        /// How long the device has been inactive (`None` if active).
        pub fn inactive_time(&self) -> Option<Duration> {
            self.inner.inactive_time()
        }
    };
}

/// A digital input with event callbacks (gpiozero's `DigitalInputDevice`).
#[derive(Clone)]
pub struct DigitalInputDevice {
    inner: Arc<DigitalInner>,
}

impl fmt::Debug for DigitalInputDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DigitalInputDevice")
            .field("pin", &self.pin())
            .field("is_active", &self.is_active())
            .finish()
    }
}

impl DigitalInputDevice {
    /// Creates a digital input on GPIO `pin` (pull-down, active high).
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, InputOptions::default())
    }

    /// Creates a digital input with explicit options.
    pub fn with_options(pin: u32, o: InputOptions) -> Result<Self> {
        let factory = resolve(&o.pin_factory)?;
        Ok(DigitalInputDevice {
            inner: DigitalInner::new(
                &factory,
                pin,
                o.pull_up,
                o.active_state,
                o.bounce_time,
                None,
            )?,
        })
    }

    digital_input_common!();
    digital_input_events!(
        when_activated,
        when_deactivated,
        wait_for_active,
        wait_for_inactive,
        "the device"
    );
}

/// Options for [`Button`].
#[derive(Clone)]
pub struct ButtonOptions {
    /// Default `Some(true)`: button wired between the pin and ground.
    pub pull_up: Option<bool>,
    /// Required when `pull_up` is `None`.
    pub active_state: Option<bool>,
    /// Debounce period.
    pub bounce_time: Option<Duration>,
    /// How long the button must be held before `when_held` fires (default 1 s).
    pub hold_time: Duration,
    /// Fire `when_held` every `hold_time` while held (default `false`).
    pub hold_repeat: bool,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for ButtonOptions {
    fn default() -> Self {
        ButtonOptions {
            pull_up: Some(true),
            active_state: None,
            bounce_time: None,
            hold_time: Duration::from_secs(1),
            hold_repeat: false,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for ButtonOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ButtonOptions")
            .field("pull_up", &self.pull_up)
            .field("active_state", &self.active_state)
            .field("bounce_time", &self.bounce_time)
            .field("hold_time", &self.hold_time)
            .field("hold_repeat", &self.hold_repeat)
            .finish_non_exhaustive()
    }
}

/// A push button (or any switch).
///
/// ```no_run
/// use gpio_zero::{Button, Led};
/// let led = Led::new(17)?;
/// let button = Button::new(2)?;
/// let l = led.clone();
/// button.when_pressed(move || { let _ = l.on(); });
/// let l = led.clone();
/// button.when_released(move || { let _ = l.off(); });
/// button.wait_for_press(None);
/// # Ok::<(), gpio_zero::Error>(())
/// ```
#[derive(Clone)]
pub struct Button {
    pub(crate) inner: Arc<DigitalInner>,
}

impl fmt::Debug for Button {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Button")
            .field("pin", &self.pin())
            .field("is_pressed", &self.is_pressed())
            .finish()
    }
}

impl Button {
    /// Creates a button on GPIO `pin` (internal pull-up, pressed = low).
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, ButtonOptions::default())
    }

    /// Creates a button with explicit options.
    pub fn with_options(pin: u32, o: ButtonOptions) -> Result<Self> {
        let factory = resolve(&o.pin_factory)?;
        Ok(Button {
            inner: DigitalInner::new(
                &factory,
                pin,
                o.pull_up,
                o.active_state,
                o.bounce_time,
                Some((o.hold_time, o.hold_repeat)),
            )?,
        })
    }

    digital_input_common!();
    digital_input_events!(
        when_pressed,
        when_released,
        wait_for_press,
        wait_for_release,
        "the button"
    );

    /// `true` while pressed.
    pub fn is_pressed(&self) -> bool {
        self.is_active()
    }

    fn hold(&self) -> &Hold {
        self.inner
            .shared
            .hold
            .as_ref()
            .expect("buttons always have hold state")
    }

    /// Runs `f` once the button has been held for [`hold_time`](Self::hold_time)
    /// (repeatedly with `hold_repeat`).
    pub fn when_held(&self, f: impl Fn() + Send + Sync + 'static) {
        *lock(&self.hold().when_held) = boxed(f);
    }

    /// `true` once `when_held` would have fired for the current press.
    pub fn is_held(&self) -> bool {
        lock(&self.hold().held_since).is_some()
    }

    /// Time since the button became "held" (`None` if not held).
    pub fn held_time(&self) -> Option<Duration> {
        let since = *lock(&self.hold().held_since);
        since.map(|s| self.inner.shared.clock.now().saturating_sub(s))
    }

    /// The hold threshold.
    pub fn hold_time(&self) -> Duration {
        *lock(&self.hold().hold_time)
    }

    /// Changes the hold threshold (applies from the next press).
    pub fn set_hold_time(&self, t: Duration) {
        *lock(&self.hold().hold_time) = t;
    }

    /// Whether `when_held` repeats.
    pub fn hold_repeat(&self) -> bool {
        *lock(&self.hold().repeat)
    }

    /// Sets whether `when_held` repeats.
    pub fn set_hold_repeat(&self, repeat: bool) {
        *lock(&self.hold().repeat) = repeat;
    }
}
