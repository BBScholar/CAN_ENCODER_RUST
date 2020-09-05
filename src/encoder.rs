use stm32f1xx_hal::{
    spi,
    gpio,
    gpio::ExtiPin
};

use embedded_hal::digital::v2::InputPin;

// pub type EncoderChannelA = gpio::gpioa::PA8<gpio::Input<gpio::PullDown>>;
// pub type EncoderChannelB = gpio::gpioa::PA9<gpio::Input<gpio::PullDown>>;
// pub type EncoderChannelI = gpio::gpioa::PA10<gpio::Input<gpio::PullDown>>;

pub type EncoderChannel = gpio::Pxx<gpio::Input<gpio::PullDown>>;

pub enum Address {

    // Volatile
    NOP       = 0x0000,
    ERRFL     = 0x0001,
    PROG      = 0x0003,
    DIAAGC    = 0x3FFC,
    MAG       = 0x3FFD,
    ANGLEUNC  = 0x3FFE,
    ANGLECOM  = 0x3FFF,

    // Non-Volatile
    ZPOSM     = 0x0016,
    ZPOSL     = 0x0017,
    SETTINGS1 = 0x0018,
    SETTINGS2 = 0x0019,
    RED       = 0x001A
}

pub struct Settings {
    zero_offset: u16
}

struct Diagnostics {

}

pub struct Encoder<SPI, REMAP, PINS> {
    count: i32,
    inverted: bool,
    absolute_offset: u16,
    a: EncoderChannel,
    b: EncoderChannel,
    i: EncoderChannel,
    spi: spi::Spi<SPI, REMAP, PINS>
}

impl <SPI, REMAP, PINS> Encoder<SPI, REMAP, PINS> {

    // look up table for encoder values
    const LUT: [i32; 16] = [0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0];

    pub fn new(
        count: i32, inverted: bool, absolute_offset: u16, 
        a: EncoderChannel, b: EncoderChannel, i: EncoderChannel,
        spi: spi::Spi<SPI, REMAP, PINS>
    ) -> Self {
        Encoder {
            count, inverted, absolute_offset,
            a, b, i, spi
        }
    }

    #[inline(always)]
    pub fn count(&self) -> i32 { self.count }

    #[inline(always)]
    pub fn inverted(&self) -> bool { self.inverted }

    #[inline(always)]
    pub fn absolute_offset(&self) -> u16 { self.absolute_offset }

    #[inline(always)]
    pub fn absolute_position() -> u16 {
        0
    }

    #[inline(always)]
    pub fn reset_count(&mut self) { self.count = 0; }

    // this will always be called from the exti interrupt
    pub fn handle_encoder_interrupt(&mut self) {
        static mut prev_gpio_value: i32 = 0;
        
        // we arent using I for now, just return if so
        if self.i.check_interrupt() {
            self.i.clear_interrupt_pending_bit();
            return
        }

        // check to see if either of the interrupts are actually being called
        if !self.a.check_interrupt() && !self.b.check_interrupt() {
            return;
        }

        let mut current_gpio_value: i32 = 0;
        current_gpio_value |= (self.a.is_high().unwrap() as i32) << 1;
        current_gpio_value |= (self.b.is_high().unwrap() as i32) << 0;
        let increment: i32 = Self::LUT[(prev_gpio_value * 4 + current_gpio_value) as usize];

        if increment == 2 {
            // this is bad, send error message or something
        } else {
            self.count += if !self.inverted { increment } else { -increment };
        }
        prev_gpio_value = current_gpio_value;

        // clear both interrupt bit (dont think this is an issue)
        self.a.clear_interrupt_pending_bit();
        self.b.clear_interrupt_pending_bit();
    }

    pub fn update_spi_values(&mut self) {
        // TODO: fill this in
    }

}


