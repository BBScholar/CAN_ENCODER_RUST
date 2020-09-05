
use stm32f1xx_hal::{
    i2c,
    i2c::BlockingI2c
};

use num_traits::PrimInt;

use heapless::{
    consts::*,
};

use core;

// these are word addresses, since we aren't storing much data
pub enum Address {
    Ticks          = 0x00, // 32 bit int
    Polarity       = 4,    // 8 bit boolean
    AbsoluteOffset = 5     // 16 bit uint
}

pub struct EEProm<I2C, PINS> {
    i2c: i2c::BlockingI2c<I2C, PINS>
}

impl <I2C, PINS> EEProm<I2C, PINS> {

    const EEPROM_ADDRESS: u8 = 0b10101110; // last bit signals read or write
    const MIN_MEMORY_ADDRESS: u8 = 0;
    const MAX_MEMORY_ADDRESS: u8 = 127;

    pub fn clear_all_memory(&mut self) {
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.resize(9, 0u8);

        let mut i: u8 = Self::MIN_MEMORY_ADDRESS;
        while i < Self::MAX_MEMORY_ADDRESS {
            buf[0] = i;
            self.i2c.write(Self::EEPROM_ADDRESS, &[buf]);
            i += 1;
        }
    }

    pub fn write_data<T: PrimInt>(&mut self, address: Address, data: T) {
        let size = T::zero().count_zeros() as usize;
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.push(address as u8);
    

        self.i2c.write(Self::EEPROM_ADDRESS, 0);
    }

    pub fn read_data<T: PrimInt>(&mut self, address: Address) -> T {
        let size = T::zero().count_zeros() as usize;

        let mut out_buf = heapless::Vec::<u8, U16>::new();
        out_buf.resize(size, 0u8);

        // self.i2c.write_without_stop(Self::EEPROM_ADDRESS, &[address as u8]);
        for i in 0..size {
            self.i2c.read(Self::EEPROM_ADDRESS, 0);
        }
        

        // PrimInt::zero().count_zeros()
    }

}