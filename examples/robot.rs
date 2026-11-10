//! Drive a two-motor robot in a square. Left motor on GPIO 4/14, right on 17/18.
use std::thread::sleep;
use std::time::Duration;

use gpio_zero::Robot;

fn main() -> gpio_zero::Result<()> {
    let robot = Robot::new((4, 14), (17, 18))?;
    for _ in 0..4 {
        robot.forward(0.8)?;
        sleep(Duration::from_secs(2));
        robot.right(0.5)?;
        sleep(Duration::from_millis(500));
    }
    robot.stop()
}
