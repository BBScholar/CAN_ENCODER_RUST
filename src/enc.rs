use stm32f1xx_hal::{
    spi::SpiReadWrite,
    gpio,
    gpio::ExtiPin,
    spi::FullDuplex
};

use embedded_hal::digital::v2::InputPin;

use volatile::Volatile;

use crate::hardware_types::{
    EncoderChannelA, EncoderChannelB, EncoderChannelI
};

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

pub struct Encoder<SPI> {
    ticks: Volatile<i32>,
    inverted: Volatile<bool>,
    absolute_offset: Volatile<u16>,
    prev_gpio_value: i32,
    a: EncoderChannelA,
    b: EncoderChannelB,
    i: EncoderChannelI,
    spi: SPI
}

impl <SPI> Encoder<SPI>
where
    SPI: FullDuplex<u16> + SpiReadWrite<u16>
{

    // look up table for encoder values
    const LUT: [i32; 16] = [0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0];

    pub fn new(
        count: i32, inverted: bool, absolute_offset: u16,
        a: EncoderChannel, b: EncoderChannel, i: EncoderChannel,
        spi: SPI
    ) -> Self {
        Encoder {
            ticks: Volatile::new(count), inverted: Volatile::new(inverted),
            absolute_offset: Volatile::new(absolute_offset), prev_gpio_value: 0,
            a, b, i, spi
        }
    }

    pub fn new_default(a: EncoderChannel, b: EncoderChannel, i: EncoderChannel, spi: SPI) -> Self {
        Self::new(0, false, 0, a, b, i, spi)
    }

    #[inline(always)]
    pub fn ticks(&self) -> i32 { self.ticks.read() }

    #[inline(always)]
    pub fn inverted(&self) -> bool { self.inverted.read() }

    #[inline(always)]
    pub fn absolute_offset(&self) -> u16 { self.absolute_offset.read() }

    #[inline(always)]
    pub fn absolute_position(self) -> u16 {
        0
    }

    #[inline(always)]
    pub fn set_inverted(&mut self, new_polarity: bool) { self.inverted.update(|val_ref| *val_ref = new_polarity); }

    #[inline(always)]
    pub fn toggle_inverted(&mut self) { self.inverted.update(|val_ref| *val_ref = !(*val_ref)); }

    #[inline(always)]
    pub fn set_ticks(&mut self, new_count: i32) { self.ticks.update(|val_ref| *val_ref = new_count); }

    #[inline(always)]
    pub fn reset_count(&mut self) { self.set_ticks(0); }

    // this will always be called from the exti interrupt
    pub fn handle_encoder_interrupt(&mut self) {
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
        let increment: i32 = Self::LUT[(self.prev_gpio_value * 4 + current_gpio_value) as usize ];

        if increment == 2 {
            // this is bad, send error message or something
        } else {
            let inverted = self.inverted.read();
            self.ticks.update(|val_ref| *val_ref += if !inverted { increment } else { -increment });
        }
        self.prev_gpio_value = current_gpio_value;

        // clear both interrupt bit (dont think this is an issue)
        self.a.clear_interrupt_pending_bit();
        self.b.clear_interrupt_pending_bit();
    }

    pub fn update_spi_values(&mut self) {
        // TODO: fill this in
    }

}


