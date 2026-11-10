//! Linux GPIO character-device backend (`/dev/gpiochipN`, uAPI v2).
//!
//! Lines are found by name (`GPIO17` on Raspberry Pi kernels, including the
//! Pi 5's RP1), falling back to offset `N` on `/dev/gpiochip0`, or on the
//! chip given to [`CdevFactory::with_chip`]. Edge events use kernel
//! timestamps and kernel debouncing. PWM is generated in software.
//! Hardware SPI uses `/dev/spidevX.Y`.

use std::fmt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use gpiocdev::line::{Bias, EdgeDetection, EdgeKind, Value};
use gpiocdev::request::{Config, Request};

use super::{
    EdgeCallback, Edges, Pin, PinFactory, PinFunction, Pull, Reservations, SoftwareSpi, SpiConfig,
    SpiInterface,
};
use crate::clock::{Clock, RealClock, lock};
use crate::error::{Error, Result, invalid};

fn backend(e: impl fmt::Display) -> Error {
    Error::Backend(e.to_string())
}

/// The character-device pin factory.
pub struct CdevFactory {
    me: Weak<CdevFactory>,
    chip: Option<PathBuf>,
    reservations: Reservations,
    clock: Arc<dyn Clock>,
}

impl fmt::Debug for CdevFactory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CdevFactory")
            .field("chip", &self.chip)
            .finish_non_exhaustive()
    }
}

impl CdevFactory {
    /// Opens the factory, checking that at least one GPIO chip exists.
    pub fn new() -> Result<Arc<Self>> {
        let chips = gpiocdev::chip::chips().map_err(backend)?;
        if chips.is_empty() {
            return Err(Error::NoPinFactory(
                "no /dev/gpiochip* devices found".into(),
            ));
        }
        Ok(Self::build(None))
    }

    /// Uses `chip` (e.g. `/dev/gpiochip4`) and treats pin numbers as line
    /// offsets on it.
    pub fn with_chip(chip: impl AsRef<Path>) -> Result<Arc<Self>> {
        let path = gpiocdev::chip::is_chip(chip.as_ref()).map_err(backend)?;
        Ok(Self::build(Some(path)))
    }

    fn build(chip: Option<PathBuf>) -> Arc<Self> {
        Arc::new_cyclic(|me| CdevFactory {
            me: me.clone(),
            chip,
            reservations: Reservations::default(),
            clock: Arc::new(RealClock),
        })
    }

    fn locate(&self, number: u32) -> (PathBuf, u32) {
        if let Some(chip) = &self.chip {
            return (chip.clone(), number);
        }
        if let Some(found) = gpiocdev::find_named_line(&format!("GPIO{number}")) {
            return (found.chip, found.info.offset);
        }
        (PathBuf::from("/dev/gpiochip0"), number)
    }
}

impl PinFactory for CdevFactory {
    fn name(&self) -> &str {
        "cdev"
    }

    fn reserve_pin(&self, number: u32) -> Result<Arc<dyn Pin>> {
        self.reservations.reserve(number)?;
        let (chip, offset) = self.locate(number);
        match CdevPin::open(number, chip, offset) {
            Ok(p) => Ok(p),
            Err(e) => {
                self.reservations.release(number);
                Err(e)
            }
        }
    }

    fn release_pin(&self, number: u32) {
        self.reservations.release(number);
    }

    fn spi(&self, config: &SpiConfig) -> Result<Box<dyn SpiInterface>> {
        match *config {
            SpiConfig::Hardware {
                port,
                device,
                clock_hz,
            } => Ok(Box::new(SpidevInterface::open(port, device, clock_hz)?)),
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

struct Worker {
    stop: Arc<AtomicBool>,
    handle: JoinHandle<()>,
}

impl Worker {
    fn stop(self) {
        self.stop.store(true, Ordering::SeqCst);
        if self.handle.thread().id() != thread::current().id() {
            let _ = self.handle.join();
        }
    }
}

struct PwmParams {
    frequency: f64,
    duty: f64,
}

struct State {
    req: Option<Arc<Request>>,
    function: PinFunction,
    pull: Pull,
    bounce: Option<Duration>,
    edges: Edges,
    level: bool,
    pwm: Option<(Arc<Mutex<PwmParams>>, Worker)>,
    watcher: Option<Worker>,
}

/// A GPIO line requested through the character device.
pub struct CdevPin {
    number: u32,
    chip: PathBuf,
    offset: u32,
    st: Mutex<State>,
    callback: Arc<Mutex<Option<EdgeCallback>>>,
}

impl fmt::Debug for CdevPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CdevPin")
            .field("number", &self.number)
            .field("chip", &self.chip)
            .field("offset", &self.offset)
            .finish()
    }
}

fn value(level: bool) -> Value {
    if level {
        Value::Active
    } else {
        Value::Inactive
    }
}

impl CdevPin {
    fn open(number: u32, chip: PathBuf, offset: u32) -> Result<Arc<dyn Pin>> {
        let pin = Arc::new(CdevPin {
            number,
            chip,
            offset,
            st: Mutex::new(State {
                req: None,
                function: PinFunction::Input,
                pull: Pull::Floating,
                bounce: None,
                edges: Edges::Both,
                level: false,
                pwm: None,
                watcher: None,
            }),
            callback: Arc::default(),
        });
        {
            let mut st = lock(&pin.st);
            pin.apply(&mut st)?;
        }
        Ok(pin)
    }

    fn config(&self, st: &State) -> Config {
        let mut cfg = Config::default();
        cfg.on_chip(&self.chip).with_line(self.offset);
        match st.function {
            PinFunction::Input => {
                cfg.as_input()
                    .with_bias(match st.pull {
                        Pull::Up => Bias::PullUp,
                        Pull::Down => Bias::PullDown,
                        Pull::Floating => Bias::Disabled,
                    })
                    .with_edge_detection(match st.edges {
                        Edges::None => None,
                        Edges::Rising => Some(EdgeDetection::RisingEdge),
                        Edges::Falling => Some(EdgeDetection::FallingEdge),
                        Edges::Both => Some(EdgeDetection::BothEdges),
                    })
                    .with_debounce_period(st.bounce.unwrap_or(Duration::ZERO));
            }
            PinFunction::Output => {
                cfg.as_output(value(st.level));
            }
        }
        cfg
    }

    /// Requests or reconfigures the line to match `st`.
    fn apply(&self, st: &mut State) -> Result<()> {
        let cfg = self.config(st);
        match &st.req {
            Some(req) => req.reconfigure(&cfg).map_err(backend)?,
            None => {
                let req = Request::from_config(cfg)
                    .with_consumer("gpio-zero")
                    .request()
                    .map_err(|e| Error::InvalidPin(format!("GPIO{}: {e}", self.number)))?;
                let req = Arc::new(req);
                st.watcher = Some(self.spawn_watcher(req.clone()));
                st.req = Some(req);
            }
        }
        Ok(())
    }

    fn spawn_watcher(&self, req: Arc<Request>) -> Worker {
        let stop = Arc::new(AtomicBool::new(false));
        let s = stop.clone();
        let cb = self.callback.clone();
        let handle = thread::Builder::new()
            .name(format!("gpio-zero-cdev-{}", self.number))
            .spawn(move || {
                while !s.load(Ordering::SeqCst) {
                    match req.wait_edge_event(Duration::from_millis(100)) {
                        Ok(true) => {
                            if let Ok(ev) = req.read_edge_event() {
                                let f = lock(&cb).clone();
                                if let Some(f) = f {
                                    f(
                                        Duration::from_nanos(ev.timestamp_ns),
                                        ev.kind == EdgeKind::Rising,
                                    );
                                }
                            }
                        }
                        Ok(false) => {}
                        Err(_) => thread::sleep(Duration::from_millis(100)),
                    }
                }
            })
            .expect("failed to spawn thread");
        Worker { stop, handle }
    }

    fn req(st: &State) -> Result<&Arc<Request>> {
        st.req.as_ref().ok_or(Error::DeviceClosed)
    }

    fn stop_pwm(st: &mut State) {
        if let Some((_, w)) = st.pwm.take() {
            w.stop();
        }
    }

    fn start_pwm(&self, st: &mut State, frequency: f64) -> Result<()> {
        let req = Self::req(st)?.clone();
        let params = Arc::new(Mutex::new(PwmParams {
            frequency,
            duty: f64::from(u8::from(st.level)),
        }));
        let p = params.clone();
        let stop = Arc::new(AtomicBool::new(false));
        let s = stop.clone();
        let handle = thread::Builder::new()
            .name(format!("gpio-zero-pwm-{}", self.number))
            .spawn(move || {
                while !s.load(Ordering::SeqCst) {
                    let (freq, duty) = {
                        let p = lock(&p);
                        (p.frequency, p.duty)
                    };
                    let period = 1.0 / freq;
                    let on = period * duty;
                    if duty > 0.0 {
                        let _ = req.set_lone_value(Value::Active);
                        thread::sleep(Duration::from_secs_f64(on));
                    }
                    if duty < 1.0 {
                        let _ = req.set_lone_value(Value::Inactive);
                        thread::sleep(Duration::from_secs_f64(period - on));
                    }
                }
            })
            .expect("failed to spawn thread");
        st.pwm = Some((params, Worker { stop, handle }));
        Ok(())
    }
}

impl Pin for CdevPin {
    fn number(&self) -> u32 {
        self.number
    }

    fn function(&self) -> PinFunction {
        lock(&self.st).function
    }

    fn set_function(&self, function: PinFunction) -> Result<()> {
        let mut st = lock(&self.st);
        if st.function == function {
            return Ok(());
        }
        if function == PinFunction::Input {
            Self::stop_pwm(&mut st);
        }
        st.function = function;
        self.apply(&mut st)
    }

    fn state(&self) -> Result<f64> {
        let st = lock(&self.st);
        if let Some((p, _)) = &st.pwm {
            return Ok(lock(p).duty);
        }
        match st.function {
            PinFunction::Output => Ok(f64::from(u8::from(st.level))),
            PinFunction::Input => {
                let v = Self::req(&st)?.lone_value().map_err(backend)?;
                Ok(if v == Value::Active { 1.0 } else { 0.0 })
            }
        }
    }

    fn set_state(&self, v: f64) -> Result<()> {
        if !(0.0..=1.0).contains(&v) {
            return invalid(format!("pin state {v} is outside 0..=1"));
        }
        let mut st = lock(&self.st);
        if st.function == PinFunction::Input {
            return Err(Error::PinSetInput(self.number));
        }
        if let Some((p, _)) = &st.pwm {
            lock(p).duty = v;
            return Ok(());
        }
        if v != 0.0 && v != 1.0 {
            return Err(Error::PwmUnsupported(self.number));
        }
        st.level = v == 1.0;
        Self::req(&st)?
            .set_lone_value(value(st.level))
            .map_err(backend)
    }

    fn pull(&self) -> Pull {
        lock(&self.st).pull
    }

    fn set_pull(&self, pull: Pull) -> Result<()> {
        let mut st = lock(&self.st);
        st.pull = pull;
        if st.function == PinFunction::Input {
            self.apply(&mut st)?;
        }
        Ok(())
    }

    fn frequency(&self) -> Option<f64> {
        lock(&self.st).pwm.as_ref().map(|(p, _)| lock(p).frequency)
    }

    fn set_frequency(&self, frequency: Option<f64>) -> Result<()> {
        let mut st = lock(&self.st);
        match frequency {
            Some(f) => {
                if f <= 0.0 || !f.is_finite() {
                    return invalid(format!("frequency {f} must be positive"));
                }
                if st.function != PinFunction::Output {
                    return Err(Error::PinSetInput(self.number));
                }
                if let Some((p, _)) = &st.pwm {
                    lock(p).frequency = f;
                    return Ok(());
                }
                self.start_pwm(&mut st, f)
            }
            None => {
                Self::stop_pwm(&mut st);
                if st.function == PinFunction::Output {
                    st.level = false;
                    Self::req(&st)?
                        .set_lone_value(Value::Inactive)
                        .map_err(backend)?;
                }
                Ok(())
            }
        }
    }

    fn bounce(&self) -> Option<Duration> {
        lock(&self.st).bounce
    }

    fn set_bounce(&self, bounce: Option<Duration>) -> Result<()> {
        let mut st = lock(&self.st);
        st.bounce = bounce;
        if st.function == PinFunction::Input {
            self.apply(&mut st)?;
        }
        Ok(())
    }

    fn edges(&self) -> Edges {
        lock(&self.st).edges
    }

    fn set_edges(&self, edges: Edges) -> Result<()> {
        let mut st = lock(&self.st);
        st.edges = edges;
        if st.function == PinFunction::Input {
            self.apply(&mut st)?;
        }
        Ok(())
    }

    fn set_when_changed(&self, callback: Option<EdgeCallback>) -> Result<()> {
        *lock(&self.callback) = callback;
        Ok(())
    }

    fn close(&self) {
        *lock(&self.callback) = None;
        let (watcher, req) = {
            let mut st = lock(&self.st);
            Self::stop_pwm(&mut st);
            (st.watcher.take(), st.req.take())
        };
        if let Some(w) = watcher {
            w.stop();
        }
        drop(req);
    }
}

impl Drop for CdevPin {
    fn drop(&mut self) {
        self.close();
    }
}

/// Hardware SPI through `/dev/spidevX.Y` (mode 0).
pub struct SpidevInterface {
    dev: Mutex<spidev::Spidev>,
    path: String,
}

impl fmt::Debug for SpidevInterface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SpidevInterface")
            .field("path", &self.path)
            .finish()
    }
}

impl SpidevInterface {
    /// Opens `/dev/spidev{port}.{device}`.
    pub fn open(port: u8, device: u8, clock_hz: u32) -> Result<Self> {
        let path = format!("/dev/spidev{port}.{device}");
        let mut dev = spidev::Spidev::open(&path)
            .map_err(|e| Error::SpiUnsupported(format!("{path}: {e}")))?;
        dev.configure(
            &spidev::SpidevOptions::new()
                .bits_per_word(8)
                .max_speed_hz(clock_hz)
                .mode(spidev::SpiModeFlags::SPI_MODE_0)
                .build(),
        )?;
        Ok(SpidevInterface {
            dev: Mutex::new(dev),
            path,
        })
    }
}

impl SpiInterface for SpidevInterface {
    fn transfer(&self, tx: &[u8]) -> Result<Vec<u8>> {
        let mut rx = vec![0u8; tx.len()];
        {
            let dev = lock(&self.dev);
            let mut t = spidev::SpidevTransfer::read_write(tx, &mut rx);
            dev.transfer(&mut t)?;
        }
        Ok(rx)
    }
}
