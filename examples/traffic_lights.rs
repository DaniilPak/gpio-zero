//! The classic traffic-light sequence on GPIO 2 (red), 3 (amber), 4 (green).
use std::thread::sleep;
use std::time::Duration;

use gpio_zero::TrafficLights;

fn main() -> gpio_zero::Result<()> {
    let lights = TrafficLights::new(2, 3, 4)?;
    for _ in 0..3 {
        lights.green().on()?;
        sleep(Duration::from_secs(3));
        lights.green().off()?;
        lights.amber().on()?;
        sleep(Duration::from_secs(1));
        lights.amber().off()?;
        lights.red().on()?;
        sleep(Duration::from_secs(3));
        lights.amber().on()?;
        sleep(Duration::from_secs(1));
        lights.off()?;
    }
    Ok(())
}
