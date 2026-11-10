//! Iterator helpers for the source/values pattern (gpiozero's
//! `gpiozero.tools`).
//!
//! Every device's `values()` is an ordinary iterator, so most of
//! `gpiozero.tools` is just `Iterator::map`/`zip`. These functions cover the
//! common cases by name:
//!
//! ```no_run
//! use gpio_zero::{Button, Led, tools};
//! let led = Led::new(17)?;
//! let button = Button::new(2)?;
//! led.set_source(tools::negated(button.values())); // LED on while NOT pressed
//! # Ok::<(), gpio_zero::Error>(())
//! ```

use std::f64::consts::TAU;
use std::iter;

/// Logical NOT of each boolean.
pub fn negated<I: IntoIterator<Item = bool>>(values: I) -> impl Iterator<Item = bool> {
    values.into_iter().map(|v| !v)
}

/// `1 - v` for values in `0..=1`.
pub fn inverted<I: IntoIterator<Item = f64>>(values: I) -> impl Iterator<Item = f64> {
    values.into_iter().map(|v| 1.0 - v)
}

/// Linearly maps `in_min..=in_max` onto `out_min..=out_max`.
pub fn scaled<I: IntoIterator<Item = f64>>(
    values: I,
    out_min: f64,
    out_max: f64,
    in_min: f64,
    in_max: f64,
) -> impl Iterator<Item = f64> {
    values
        .into_iter()
        .map(move |v| out_min + (v - in_min) / (in_max - in_min) * (out_max - out_min))
}

/// Maps `0..=1` onto `-1..=1` (e.g. potentiometer → motor).
pub fn scaled_full<I: IntoIterator<Item = f64>>(values: I) -> impl Iterator<Item = f64> {
    scaled(values, -1.0, 1.0, 0.0, 1.0)
}

/// Maps `-1..=1` onto `0..=1`.
pub fn scaled_half<I: IntoIterator<Item = f64>>(values: I) -> impl Iterator<Item = f64> {
    scaled(values, 0.0, 1.0, -1.0, 1.0)
}

/// Clamps each value to `min..=max`.
pub fn clamped<I: IntoIterator<Item = f64>>(
    values: I,
    min: f64,
    max: f64,
) -> impl Iterator<Item = f64> {
    values.into_iter().map(move |v| v.clamp(min, max))
}

/// Absolute value of each item.
pub fn absoluted<I: IntoIterator<Item = f64>>(values: I) -> impl Iterator<Item = f64> {
    values.into_iter().map(f64::abs)
}

/// `true` when the value is within `min..=max`.
pub fn booleanized<I: IntoIterator<Item = f64>>(
    values: I,
    min: f64,
    max: f64,
) -> impl Iterator<Item = bool> {
    values.into_iter().map(move |v| (min..=max).contains(&v))
}

/// Rounds `0..=1` values down to `steps` levels.
pub fn quantized<I: IntoIterator<Item = f64>>(values: I, steps: u32) -> impl Iterator<Item = f64> {
    let s = f64::from(steps.max(1));
    values
        .into_iter()
        .map(move |v| (v * s).floor().min(s - 1.0).max(0.0) / s)
}

/// Moving average over the last `size` values.
pub fn smoothed<I: IntoIterator<Item = f64>>(values: I, size: usize) -> impl Iterator<Item = f64> {
    let size = size.max(1);
    let mut window = std::collections::VecDeque::with_capacity(size);
    values.into_iter().map(move |v| {
        window.push_back(v);
        if window.len() > size {
            window.pop_front();
        }
        window.iter().sum::<f64>() / window.len() as f64
    })
}

/// `true` when both inputs are `true`.
pub fn all_values<A, B>(a: A, b: B) -> impl Iterator<Item = bool>
where
    A: IntoIterator<Item = bool>,
    B: IntoIterator<Item = bool>,
{
    a.into_iter().zip(b).map(|(x, y)| x && y)
}

/// `true` when either input is `true`.
pub fn any_values<A, B>(a: A, b: B) -> impl Iterator<Item = bool>
where
    A: IntoIterator<Item = bool>,
    B: IntoIterator<Item = bool>,
{
    a.into_iter().zip(b).map(|(x, y)| x || y)
}

/// Mean of two inputs.
pub fn averaged<A, B>(a: A, b: B) -> impl Iterator<Item = f64>
where
    A: IntoIterator<Item = f64>,
    B: IntoIterator<Item = f64>,
{
    a.into_iter().zip(b).map(|(x, y)| (x + y) / 2.0)
}

/// Sum of two inputs.
pub fn summed<A, B>(a: A, b: B) -> impl Iterator<Item = f64>
where
    A: IntoIterator<Item = f64>,
    B: IntoIterator<Item = f64>,
{
    a.into_iter().zip(b).map(|(x, y)| x + y)
}

/// Product of two inputs.
pub fn multiplied<A, B>(a: A, b: B) -> impl Iterator<Item = f64>
where
    A: IntoIterator<Item = f64>,
    B: IntoIterator<Item = f64>,
{
    a.into_iter().zip(b).map(|(x, y)| x * y)
}

/// `true, false, true, ...` (starting with `initial`).
pub fn alternating_values(initial: bool) -> impl Iterator<Item = bool> {
    iter::successors(Some(initial), |v| Some(!v))
}

/// An endless sine wave in `-1..=1`, `period` samples long.
pub fn sin_values(period: u32) -> impl Iterator<Item = f64> {
    let p = f64::from(period.max(1));
    (0u64..).map(move |i| (TAU * (i as f64 % p) / p).sin())
}

/// An endless cosine wave in `-1..=1`, `period` samples long.
pub fn cos_values(period: u32) -> impl Iterator<Item = f64> {
    let p = f64::from(period.max(1));
    (0u64..).map(move |i| (TAU * (i as f64 % p) / p).cos())
}

/// An endless triangle ramp `0 → 1 → 0`, `period` samples long.
pub fn ramping_values(period: u32) -> impl Iterator<Item = f64> {
    let p = f64::from(period.max(2));
    (0u64..).map(move |i| {
        let t = (i as f64 % p) / p * 2.0;
        if t <= 1.0 { t } else { 2.0 - t }
    })
}

/// Endless pseudo-random values in `0..1` (xorshift; not for cryptography).
pub fn random_values() -> impl Iterator<Item = f64> {
    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x2545_F491_4F6C_DD1D)
        | 1;
    iter::successors(Some(seed), |x| {
        let mut x = *x;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        Some(x)
    })
    .skip(1)
    .map(|x| (x >> 11) as f64 / (1u64 << 53) as f64)
}
