//! Time sources.
//!
//! Every background activity in this crate (blinking, pulsing, hold detection,
//! sensor sampling, `source` loops) sleeps through a [`Clock`] owned by the pin
//! factory. Real factories use [`RealClock`]; the mock factory uses a
//! [`MockClock`] whose time only moves when a test calls
//! [`MockClock::advance`], which makes timing-dependent behaviour fully
//! deterministic.

use std::collections::HashMap;
use std::fmt;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// A manually-reset event flag (like Python's `threading.Event`).
#[derive(Clone, Default)]
pub struct Event {
    inner: Arc<(Mutex<bool>, Condvar)>,
}

impl fmt::Debug for Event {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Event")
            .field("set", &self.is_set())
            .finish()
    }
}

impl Event {
    /// Creates a cleared event.
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the flag and wakes all waiters.
    pub fn set(&self) {
        *lock(&self.inner.0) = true;
        self.inner.1.notify_all();
    }

    /// Clears the flag.
    pub fn clear(&self) {
        *lock(&self.inner.0) = false;
    }

    /// Returns whether the flag is set.
    pub fn is_set(&self) -> bool {
        *lock(&self.inner.0)
    }

    /// Blocks (in real time) until the flag is set or `timeout` elapses.
    /// Returns `true` if the flag was set.
    pub fn wait_real(&self, timeout: Option<Duration>) -> bool {
        let mut g = lock(&self.inner.0);
        match timeout {
            None => {
                while !*g {
                    g = self.inner.1.wait(g).unwrap_or_else(|e| e.into_inner());
                }
                true
            }
            Some(t) => {
                let end = Instant::now() + t;
                while !*g {
                    let now = Instant::now();
                    if now >= end {
                        return false;
                    }
                    g = self
                        .inner
                        .1
                        .wait_timeout(g, end - now)
                        .unwrap_or_else(|e| e.into_inner())
                        .0;
                }
                true
            }
        }
    }
}

/// A source of monotonic time that can also put threads to sleep and spawn
/// background tasks.
pub trait Clock: Send + Sync + fmt::Debug {
    /// Monotonic time ("ticks"). Edge-callback timestamps use the same base.
    fn now(&self) -> Duration;

    /// Waits until `event` is set or `timeout` elapses (`None` = forever).
    /// Returns `true` if the event was set.
    fn wait(&self, event: &Event, timeout: Option<Duration>) -> bool;

    /// Sleeps for `duration`.
    fn sleep(&self, duration: Duration) {
        self.wait(&Event::new(), Some(duration));
    }

    /// Spawns a background task. Mock clocks track tasks so that
    /// [`MockClock::advance`] can wait for them to settle.
    fn spawn(&self, name: &str, f: Box<dyn FnOnce() + Send>) -> JoinHandle<()>;
}

/// Wall-clock time (`CLOCK_MONOTONIC` on Linux, so that kernel GPIO event
/// timestamps share its time base).
#[derive(Debug, Default, Clone, Copy)]
pub struct RealClock;

impl RealClock {
    /// Creates a real clock.
    pub fn new() -> Self {
        RealClock
    }
}

#[cfg(target_os = "linux")]
fn monotonic_now() -> Duration {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, writable timespec.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    Duration::new(ts.tv_sec as u64, ts.tv_nsec as u32)
}

#[cfg(not(target_os = "linux"))]
fn monotonic_now() -> Duration {
    static EPOCH: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    EPOCH.get_or_init(Instant::now).elapsed()
}

impl Clock for RealClock {
    fn now(&self) -> Duration {
        monotonic_now()
    }

    fn wait(&self, event: &Event, timeout: Option<Duration>) -> bool {
        event.wait_real(timeout)
    }

    fn spawn(&self, name: &str, f: Box<dyn FnOnce() + Send>) -> JoinHandle<()> {
        thread::Builder::new()
            .name(name.to_string())
            .spawn(f)
            .expect("failed to spawn thread")
    }
}

struct Sleeper {
    deadline: Option<Duration>,
    event: Event,
    is_task: bool,
}

#[derive(Default)]
struct MockState {
    now: Duration,
    next_id: u64,
    sleepers: HashMap<u64, Sleeper>,
    live_tasks: usize,
}

thread_local! {
    static IS_MOCK_TASK: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Virtual time for tests. Time starts at zero and only moves forward when
/// [`advance`](MockClock::advance) is called.
///
/// `advance` steps through every pending deadline in order and, after each
/// step, waits until every background task spawned through this clock is
/// asleep again (or finished). Assertions made right after `advance` returns
/// therefore see a settled system.
#[derive(Default, Clone)]
pub struct MockClock {
    shared: Arc<MockShared>,
}

#[derive(Default)]
struct MockShared {
    state: Mutex<MockState>,
    cv: Condvar,
}

impl fmt::Debug for MockClock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let st = lock(&self.shared.state);
        f.debug_struct("MockClock")
            .field("now", &st.now)
            .field("live_tasks", &st.live_tasks)
            .finish()
    }
}

const POLL: Duration = Duration::from_millis(1);
const SETTLE_LIMIT: Duration = Duration::from_secs(5);

impl MockShared {
    fn idle_tasks(st: &MockState) -> usize {
        st.sleepers
            .values()
            .filter(|s| s.is_task && !s.event.is_set())
            .count()
    }

    fn settle<'a>(&'a self, mut st: MutexGuard<'a, MockState>) -> MutexGuard<'a, MockState> {
        let start = Instant::now();
        while Self::idle_tasks(&st) < st.live_tasks && start.elapsed() < SETTLE_LIMIT {
            st = self
                .cv
                .wait_timeout(st, POLL)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
        st
    }
}

impl MockClock {
    /// Creates a clock at time zero.
    pub fn new() -> Self {
        Self::default()
    }

    /// Waits (in real time) until all background tasks are asleep or done,
    /// without moving time.
    pub fn settle(&self) {
        let st = lock(&self.shared.state);
        drop(self.shared.settle(st));
    }

    /// Moves virtual time forward by `duration`, waking sleepers in deadline
    /// order and letting background tasks run between steps.
    pub fn advance(&self, duration: Duration) {
        let sh = &self.shared;
        let mut st = lock(&sh.state);
        let target = st.now + duration;
        loop {
            st = sh.settle(st);
            let next = st
                .sleepers
                .values()
                .filter(|s| !s.event.is_set())
                .filter_map(|s| s.deadline)
                .filter(|d| *d <= target)
                .min();
            match next {
                Some(t) => {
                    if t > st.now {
                        st.now = t;
                    }
                    let now = st.now;
                    st.sleepers
                        .retain(|_, s| s.deadline.is_none_or(|d| d > now));
                    sh.cv.notify_all();
                }
                None => {
                    st.now = target;
                    sh.cv.notify_all();
                    break;
                }
            }
        }
        drop(sh.settle(st));
    }

    /// Number of live background tasks (useful in tests).
    pub fn live_tasks(&self) -> usize {
        lock(&self.shared.state).live_tasks
    }
}

impl Clock for MockClock {
    fn now(&self) -> Duration {
        lock(&self.shared.state).now
    }

    fn wait(&self, event: &Event, timeout: Option<Duration>) -> bool {
        if event.is_set() {
            return true;
        }
        if timeout == Some(Duration::ZERO) {
            return false;
        }
        let sh = &self.shared;
        let mut st = lock(&sh.state);
        let deadline = timeout.map(|t| st.now + t);
        let id = st.next_id;
        st.next_id += 1;
        st.sleepers.insert(
            id,
            Sleeper {
                deadline,
                event: event.clone(),
                is_task: IS_MOCK_TASK.with(|c| c.get()),
            },
        );
        sh.cv.notify_all();
        loop {
            if event.is_set() {
                st.sleepers.remove(&id);
                sh.cv.notify_all();
                return true;
            }
            if !st.sleepers.contains_key(&id) {
                return event.is_set();
            }
            st = sh
                .cv
                .wait_timeout(st, POLL)
                .unwrap_or_else(|e| e.into_inner())
                .0;
        }
    }

    fn spawn(&self, name: &str, f: Box<dyn FnOnce() + Send>) -> JoinHandle<()> {
        lock(&self.shared.state).live_tasks += 1;
        let done = TaskDone(self.shared.clone());
        thread::Builder::new()
            .name(name.to_string())
            .spawn(move || {
                IS_MOCK_TASK.with(|c| c.set(true));
                let _done = done;
                f();
            })
            .expect("failed to spawn thread")
    }
}

/// Decrements the live-task counter when a mock task finishes (even on panic).
struct TaskDone(Arc<MockShared>);

impl Drop for TaskDone {
    fn drop(&mut self) {
        let mut st = lock(&self.0.state);
        st.live_tasks = st.live_tasks.saturating_sub(1);
        self.0.cv.notify_all();
    }
}

/// A background task with a stop flag. Dropping it stops and joins it.
pub(crate) struct Task {
    stop: Event,
    handle: Option<JoinHandle<()>>,
}

impl Task {
    pub(crate) fn spawn<F>(clock: &Arc<dyn Clock>, name: &str, f: F) -> Task
    where
        F: FnOnce(Event) + Send + 'static,
    {
        let stop = Event::new();
        let s = stop.clone();
        let handle = clock.spawn(name, Box::new(move || f(s)));
        Task {
            stop,
            handle: Some(handle),
        }
    }

    /// Sets the stop flag without waiting.
    pub(crate) fn signal(&mut self) {
        self.stop.set();
    }

    pub(crate) fn stop(&mut self) {
        self.stop.set();
        if let Some(h) = self.handle.take()
            && h.thread().id() != thread::current().id()
        {
            let _ = h.join();
        }
    }

    pub(crate) fn is_finished(&self) -> bool {
        self.handle.as_ref().is_none_or(|h| h.is_finished())
    }
}

impl Drop for Task {
    fn drop(&mut self) {
        self.stop();
    }
}
