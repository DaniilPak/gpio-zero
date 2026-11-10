use std::fmt;
use std::str::FromStr;
use std::sync::Arc;

use super::OutputInner;
use crate::clock::lock;
use crate::devices::check_signed;
use crate::error::{Error, Result, invalid};
use crate::pins::{PinFactory, resolve};

const NOTE_NAMES: [&str; 12] = [
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
];

/// A musical tone, stored as a frequency in Hz.
///
/// ```
/// use gpio_zero::Tone;
/// let a4: Tone = "A4".parse().unwrap();
/// assert_eq!(a4.frequency(), 440.0);
/// assert_eq!(a4.midi(), Some(69));
/// assert_eq!(Tone::from_midi(60).note().as_deref(), Some("C4"));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Tone(f64);

impl Tone {
    /// A tone of `hz` Hz.
    pub fn from_frequency(hz: f64) -> Self {
        Tone(hz)
    }

    /// The tone for MIDI note `note` (69 = A4 = 440 Hz).
    pub fn from_midi(note: u8) -> Self {
        Tone(440.0 * 2f64.powf((f64::from(note) - 69.0) / 12.0))
    }

    /// Parses a note name such as `"A4"`, `"C#5"` or `"Bb3"`.
    pub fn from_note(name: &str) -> Result<Self> {
        let bad = || Error::InvalidValue(format!("invalid note name {name:?}"));
        let mut chars = name.chars();
        let letter = chars.next().ok_or_else(bad)?.to_ascii_uppercase();
        let base: i32 = match letter {
            'C' => 0,
            'D' => 2,
            'E' => 4,
            'F' => 5,
            'G' => 7,
            'A' => 9,
            'B' => 11,
            _ => return Err(bad()),
        };
        let rest: String = chars.collect();
        let (shift, octave) = if let Some(r) = rest.strip_prefix(['#', '♯']) {
            (1, r)
        } else if let Some(r) = rest.strip_prefix(['b', '♭']) {
            (-1, r)
        } else {
            (0, rest.as_str())
        };
        let octave: i32 = octave.parse().map_err(|_| bad())?;
        let midi = (octave + 1) * 12 + base + shift;
        let midi = u8::try_from(midi)
            .ok()
            .filter(|m| *m <= 127)
            .ok_or_else(bad)?;
        Ok(Tone::from_midi(midi))
    }

    /// Frequency in Hz.
    pub fn frequency(self) -> f64 {
        self.0
    }

    /// The MIDI note number, if the frequency is (within 1 cent of) one.
    pub fn midi(self) -> Option<u8> {
        let m = 69.0 + 12.0 * (self.0 / 440.0).log2();
        let r = m.round();
        ((m - r).abs() < 0.01 && (0.0..=127.0).contains(&r)).then_some(r as u8)
    }

    /// The note name (e.g. `"A4"`), if this is an exact MIDI note.
    pub fn note(self) -> Option<String> {
        self.midi().map(|m| {
            let octave = i32::from(m) / 12 - 1;
            format!("{}{}", NOTE_NAMES[usize::from(m % 12)], octave)
        })
    }

    /// The tone `n` semitones higher.
    pub fn up(self, n: i32) -> Self {
        Tone(self.0 * 2f64.powf(f64::from(n) / 12.0))
    }

    /// The tone `n` semitones lower.
    pub fn down(self, n: i32) -> Self {
        self.up(-n)
    }
}

impl From<f64> for Tone {
    fn from(hz: f64) -> Self {
        Tone(hz)
    }
}

impl FromStr for Tone {
    type Err = Error;
    fn from_str(s: &str) -> Result<Self> {
        Tone::from_note(s)
    }
}

impl fmt::Display for Tone {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.note() {
            Some(n) => write!(f, "{n}"),
            None => write!(f, "{:.2}Hz", self.0),
        }
    }
}

/// Options for [`TonalBuzzer`].
#[derive(Clone)]
pub struct TonalBuzzerOptions {
    /// Initial value (`-1.0..=1.0`) or `None` for silent (default).
    pub initial_value: Option<f64>,
    /// The tone at value 0 (default A4, 440 Hz).
    pub mid_tone: Tone,
    /// Octaves either side of `mid_tone` (default 1).
    pub octaves: u32,
    /// Pin factory to use.
    pub pin_factory: Option<Arc<dyn PinFactory>>,
}

impl Default for TonalBuzzerOptions {
    fn default() -> Self {
        TonalBuzzerOptions {
            initial_value: None,
            mid_tone: Tone(440.0),
            octaves: 1,
            pin_factory: None,
        }
    }
}

impl fmt::Debug for TonalBuzzerOptions {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TonalBuzzerOptions")
            .field("initial_value", &self.initial_value)
            .field("mid_tone", &self.mid_tone)
            .field("octaves", &self.octaves)
            .finish_non_exhaustive()
    }
}

struct TonalInner {
    out: Arc<OutputInner>,
    mid: Tone,
    octaves: u32,
    tone: std::sync::Mutex<Option<Tone>>,
}

/// A passive buzzer driven with PWM to play tones.
#[derive(Clone)]
pub struct TonalBuzzer {
    inner: Arc<TonalInner>,
}

impl fmt::Debug for TonalBuzzer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TonalBuzzer")
            .field("tone", &self.tone())
            .finish()
    }
}

impl TonalBuzzer {
    /// Creates a tonal buzzer on GPIO `pin` with default options.
    pub fn new(pin: u32) -> Result<Self> {
        Self::with_options(pin, TonalBuzzerOptions::default())
    }

    /// Creates a tonal buzzer with explicit options.
    pub fn with_options(pin: u32, options: TonalBuzzerOptions) -> Result<Self> {
        if options.octaves == 0 {
            return invalid("octaves must be at least 1");
        }
        let factory = resolve(&options.pin_factory)?;
        let out = OutputInner::new(&factory, pin, true, None, Some(0.0))?;
        let b = TonalBuzzer {
            inner: Arc::new(TonalInner {
                out,
                mid: options.mid_tone,
                octaves: options.octaves,
                tone: std::sync::Mutex::new(None),
            }),
        };
        b.set_value(options.initial_value)?;
        Ok(b)
    }

    /// Lowest playable tone.
    pub fn min_tone(&self) -> Tone {
        Tone(self.inner.mid.0 / 2f64.powi(self.inner.octaves as i32))
    }

    /// Middle tone (value 0).
    pub fn mid_tone(&self) -> Tone {
        self.inner.mid
    }

    /// Highest playable tone.
    pub fn max_tone(&self) -> Tone {
        Tone(self.inner.mid.0 * 2f64.powi(self.inner.octaves as i32))
    }

    /// Octave range either side of the middle tone.
    pub fn octaves(&self) -> u32 {
        self.inner.octaves
    }

    /// Plays `tone` (a [`Tone`], a frequency `f64`, or use `"A4".parse()`).
    pub fn play(&self, tone: impl Into<Tone>) -> Result<()> {
        let tone = tone.into();
        let (lo, hi) = (self.min_tone().0, self.max_tone().0);
        if !(lo - 1e-9..=hi + 1e-9).contains(&tone.0) {
            return invalid(format!("tone {tone} is outside {lo:.2}..={hi:.2} Hz"));
        }
        let pin = &self.inner.out.pin;
        pin.set_frequency(Some(tone.0))?;
        pin.set_state(0.5)?;
        *lock(&self.inner.tone) = Some(tone);
        Ok(())
    }

    /// Silences the buzzer.
    pub fn stop(&self) -> Result<()> {
        let pin = &self.inner.out.pin;
        if pin.frequency().is_some() {
            pin.set_state(0.0)?;
            pin.set_frequency(None)?;
        }
        *lock(&self.inner.tone) = None;
        Ok(())
    }

    /// The tone being played, if any.
    pub fn tone(&self) -> Option<Tone> {
        *lock(&self.inner.tone)
    }

    /// `true` while playing.
    pub fn is_active(&self) -> bool {
        self.tone().is_some()
    }

    /// The tone as a value: `-1.0` = `min_tone`, `0.0` = `mid_tone`,
    /// `1.0` = `max_tone`; `None` when silent.
    pub fn value(&self) -> Option<f64> {
        self.tone()
            .map(|t| (t.0 / self.inner.mid.0).log2() / f64::from(self.inner.octaves))
    }

    /// Plays the tone for `value` (see [`value`](Self::value)) or stops with `None`.
    pub fn set_value(&self, value: impl Into<Option<f64>>) -> Result<()> {
        match value.into() {
            None => self.stop(),
            Some(v) => {
                let v = check_signed(v, "tonal buzzer value")?;
                self.play(Tone(
                    self.inner.mid.0 * 2f64.powf(v * f64::from(self.inner.octaves)),
                ))
            }
        }
    }
}
