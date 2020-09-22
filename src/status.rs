
use stm32f1xx_hal::gpio;

pub enum LedState {
    On, Off, FlashSlow, FlashFast
}
pub type TriLedState = (LedState, LedState, LedState);

pub type StatusPin = gpio::Pxx<gpio::Output<gpio::PushPull>>;

struct StatusHandler {

}

impl StatusHandler {
    
    fn new() -> Self {
        StatusHandler {

        }
    }

    fn new_with_state() -> Self {
        StatusHandler {
            
        }
    }

    fn set_off(&mut self) {}

    fn set_state(&mut self, state: TriLedState) {

    }

}
