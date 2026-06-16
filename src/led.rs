use core::sync::atomic::{AtomicU32, Ordering};

use lilos::time::Millis;
use num_traits::Pow as _;
use stm32_hal2::gpio::Pin;

fn duty_cycle_mode(elapsed: Millis, value: f32) -> bool {
    const PERIOD: u32 = 700;
    let duty = 0.03 + value.pow(0.7) * 0.95;
    let phase = (elapsed.0 as u32) % PERIOD;
    (phase as f32) < (PERIOD as f32) * duty
}

pub struct LedFlasher<'a> {
    pin: Pin,
    control: &'a AtomicU32,
}

impl<'a> LedFlasher<'a> {
    pub fn new(pin: Pin, control: &'a AtomicU32) -> Self {
        Self { pin, control }
    }

    pub fn run(&mut self, elapsed: Millis) {
        const DISPLAY_MAX_MA: f32 = 15000.0;
        const THRESHOLD_CURRENT: f32 = 100.0;
        let current_ma = self.control.load(Ordering::Relaxed) as f32;
        if current_ma < THRESHOLD_CURRENT {
            self.pin.set_high();
            return;
        }

        let normalized = current_ma.min(DISPLAY_MAX_MA) / DISPLAY_MAX_MA;

        if duty_cycle_mode(elapsed, normalized) {
            self.pin.set_low();
        } else {
            self.pin.set_high();
        }
    }
}
