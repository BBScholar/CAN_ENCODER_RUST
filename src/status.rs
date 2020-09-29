
use stm32f1xx_hal::gpio;

pub enum LedState { On, Off, FlashSlow, FlashFast, NoSet }

pub struct LedStateTriple(LedState, LedState, LedState);

impl LedStateTriple {
    const OFF: LedStateTriple = LedStateTriple { 0: LedState::Off, 1: LedState::Off, 2: LedState::Off };
    const ON: LedStateTriple = LedStateTriple  { 0: LedState::On,  1: LedState::On,  2: LedState::On  };
}

pub type StatusPin = gpio::Pxx<gpio::Output<gpio::PushPull>>;

pub struct StatusHandler {
    status1: StatusPin,
    status2: StatusPin,
    status3: StatusPin,
    current_status: LedStateTriple
}

impl StatusHandler {
    
    pub fn new(status1: StatusPin, status2: StatusPin, status3: StatusPin) -> Self {
        Self::new_with_status(status1, status2, status3, LedStateTriple::OFF)
    }

    pub fn new_with_status(status1: StatusPin, status2: StatusPin, status3: StatusPin, led_status: LedStateTriple) -> Self {
        StatusHandler {
            status1, status2, status3, current_status: led_status
        }
    }

    fn set_off(&mut self) {
        self.set_state(LedStateTriple::OFF);
    }

    pub fn set_state(&mut self, state: LedStateTriple) {
        self.current_status = state;
    }

    pub fn update(&mut self) {

    }

}
