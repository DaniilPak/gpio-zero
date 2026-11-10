//! LED follows a button. Wiring: LED on GPIO17, button between GPIO2 and GND.
//!
//! Try it without hardware: `GPIOZERO_PIN_FACTORY=mock cargo run --example button_led`
use gpio_zero::{Button, Led};

fn main() -> gpio_zero::Result<()> {
    let led = Led::new(17)?;
    let button = Button::new(2)?;

    let l = led.clone();
    button.when_pressed(move || {
        let _ = l.on();
    });
    let l = led.clone();
    button.when_released(move || {
        let _ = l.off();
    });

    println!("Press the button (waiting up to 10 s)...");
    if button.wait_for_press(Some(std::time::Duration::from_secs(10))) {
        println!("Pressed!");
        button.wait_for_release(Some(std::time::Duration::from_secs(10)));
    }
    Ok(())
}
