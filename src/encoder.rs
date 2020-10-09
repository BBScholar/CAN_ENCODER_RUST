use stm32f1xx_hal::{
    spi::SpiReadWrite,
    gpio::ExtiPin,
    spi::FullDuplex
};

use embedded_hal::digital::v2::InputPin;

use volatile::Volatile;

use crate::hardware_types::{
    EncoderChannelA, EncoderChannelB, EncoderChannelI
};

use heapless::{
    Vec,
    consts::*
};

use core::convert::{Into, From};

#[allow(dead_code)]
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


pub struct Settings1 {
    iwidth: bool,
    noiseset: bool,
    dir: bool,
    uvw_abi: bool,
    daecdis: bool,
    dataselect: bool,
    pwmon: bool,
}

impl Into<u16> for &Settings1 {

    fn into(self) -> u16 {
        let mut data1 = 0u16;
        data1 |= (self.iwidth     as u16) << 0;
        data1 |= (self.noiseset   as u16) << 1;
        data1 |= (self.dir        as u16) << 2;
        data1 |= (self.uvw_abi    as u16) << 3;
        data1 |= (self.daecdis    as u16) << 4;
        data1 |= (self.dataselect as u16) << 6;
        data1 |= (self.pwmon      as u16) << 7;
        data1
    }

}

pub struct Settings2 {
    data: u16
}

impl Into<u16> for &Settings2 {

    fn into(self) -> u16 { self.data }

}

pub struct Zpos {
    offset: u16,
    error_low: bool,
    error_high: bool,
}

impl Into<(u16, u16)> for &Zpos {

    fn into(self) -> (u16, u16) {
        let mut zposm = 0u16;
        let mut zposl = 0u16;

        zposl |= (self.error_low  as u16) << 6;
        zposl |= (self.error_high as u16) << 7; 

        (zposl, zposm)
    }

}

// impl From<&Zpos> for (u16, u16) {

//     fn from(zpos: &Zpos) -> Self {
//         return zpos.into()
//     }

// }

#[allow(dead_code)]
pub struct AllSettings {
    settings1: Settings1,
    settings2: Settings2,
    zpos: Zpos
}

impl AllSettings {
    
    #[inline(always)]
    pub fn settings1(&self) -> Settings1 {
        self.settings1
    }

    #[inline(always)]
    pub fn settings2(&self) -> Settings2 {
        self.settings2
    }

    #[inline(always)]
    pub fn zpos(&self) -> Zpos {
        self.zpos
    }

}

#[allow(dead_code)]
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
        a: EncoderChannelA, b: EncoderChannelB, i: EncoderChannelI,
        spi: SPI
    ) -> Self {
        Encoder {
            ticks: Volatile::new(count), inverted: Volatile::new(inverted),
            absolute_offset: Volatile::new(absolute_offset), prev_gpio_value: 0,
            a, b, i, spi
        }
    }

    pub fn new_default(a: EncoderChannelA, b: EncoderChannelB, i: EncoderChannelI, spi: SPI) -> Self {
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
    pub fn set_ticks(&mut self, new_count: i32) { self.ticks.update(|val_ref| *val_ref = new_count); }

    #[inline(always)]
    pub fn reset_ticks(&mut self) { self.set_ticks(0); }

    #[inline(always)]
    pub fn set_inverted(&mut self, new_polarity: bool) { self.inverted.update(|val_ref| *val_ref = new_polarity); }

    #[inline(always)]
    pub fn toggle_inverted(&mut self) { self.inverted.update(|val_ref| *val_ref = !(*val_ref)); }

    #[inline(always)]
    pub fn set_absolute_offset(&mut self, offset: u16) { self.absolute_offset.update(|val_ref| *val_ref = offset); }

   

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

    fn generate_control_frame(&mut self, address: Address, read: bool) -> u16 {
        let mut frame = 0u16;
        frame |= (read as u16) << 14;
        frame |= address as u16;
        write_even_parity(&mut frame);
        frame
    }

    fn write_to_spi_register(&mut self, address: Address, data: &u16) -> Result<(), ()> {
        let mut tx_buf = Vec::<u16, U2>::new();
        let mut rx_buf = Vec::<u16, U2>::new();
        let control_frame = self.generate_control_frame(address, false);
        tx_buf.push(control_frame).unwrap();
        tx_buf.push(*data).unwrap();

        for tx in tx_buf {
            self.spi.send(tx);
            if let Some(word) = self.spi.read().ok() {
                rx_buf.push(word);
            } else {
                return Err(())
            }
        }

        // check for faults here later

        Ok(())
    }

    fn read_from_spi_register(&mut self, address: Address) -> Result<u16, ()> {
        let mut rx_buf = Vec::<u8, U2>::new();

        let control_frame = self.generate_control_frame(address, true);
        match self.spi.send(control_frame) {
            Ok(()) => {},
            Err(_) => return Err(())
        };
        if let Some(word) = self.spi.read().ok() {
            rx_buf.extend_from_slice(&word.to_ne_bytes());
        } else {
            return Err(())
        }
        
        let mut result: u16 = 0;
        result |= (rx_buf[0] as u16) << 8;
        result |= rx_buf[1] as u16;
        result %= 1 << 15;
        result %= 1 << 14;
        Ok(result)
    }

    pub fn config_all(&mut self, settings: &AllSettings) {
        self.config_settings1(&settings.settings1());
        self.config_settings2(&settings.settings2());
        self.config_zpos(&settings.zpos());
    }

    pub fn config_settings1(&mut self, settings: &Settings1) -> Result<(), ()> {
        return self.write_to_spi_register(Address::SETTINGS1, &settings.into());
    }

    pub fn config_settings2(&mut self, settings: &Settings2) -> Result<(), ()> {
        return self.write_to_spi_register(Address::SETTINGS2, &settings.into());
    }

    pub fn config_zpos(&mut self, settings: &Zpos) -> Result<(), ()>{
        let s: (u16, u16) = settings.into();
        self.write_to_spi_register(Address::ZPOSL, &s.0).and_then( |_| {
            self.write_to_spi_register(Address::ZPOSM, &s.1)
        })
    }

}

#[inline(always)]
fn write_even_parity(data: &mut u16) {
    let n = (*data).count_ones() as u16;
    let parity = n % 2 != 0;
    *data |= (parity as u16) << 15; 
}


