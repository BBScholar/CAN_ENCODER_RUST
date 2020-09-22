
use stm32f1xx_hal::{
    gpio,
};

use embedded_hal::blocking::i2c::{
    Write, Read, WriteRead
};

use num_traits::{
    cast::ToPrimitive,
    cast::FromPrimitive,
    Num,
    PrimInt
};

use heapless::{
    consts::*,
};

use core::{
    ops::Deref,
    convert::TryInto
};

// these are word addresses, since we aren't storing much data
#[allow(dead_code)]
pub enum Address {
    Ticks          = 0x00, // 32 bit int
    Polarity       = 4,    // 8 bit boolean
    AbsoluteOffset = 5     // 16 bit uint
}

pub type SclPin = gpio::gpiob::PB10<gpio::Alternate<gpio::OpenDrain>>;
pub type SdaPin = gpio::gpiob::PB11<gpio::Alternate<gpio::OpenDrain>>;
pub type I2CPins = (SclPin, SdaPin);

pub struct EEProm<I2C> {
    i2c: I2C
}

impl <I2C, E> EEProm<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>
{
    
    const EEPROM_ADDRESS: u8 = 0b10101110; // last bit signals read or write
    const MIN_MEMORY_ADDRESS: u8 = 0;
    const MAX_MEMORY_ADDRESS: u8 = 127;

    pub fn new(i2c: I2C) -> Self {
        EEProm {
            i2c
        }
    }

    pub fn clear_all_memory(&mut self) {
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.resize(1 + 8, 0).unwrap();

        let mut i = Self::MIN_MEMORY_ADDRESS;
        while i < Self::MAX_MEMORY_ADDRESS {
            buf[0] = i;
            self.i2c.write(Self::EEPROM_ADDRESS, buf.as_ref()).ok();
            i += 1;
        }
    }

    // this is super jank, have to convert bools to u8
    pub fn write_data<T: Num + PrimInt + ToPrimitive>(&mut self, address: u8, data: T) {
        let mut buf = heapless::Vec::<u8, U16>::new();
        buf.push(address).ok();

        // convert every number into an i32 for ease of use
        let number = ToPrimitive::to_i32(&data).unwrap();
        buf.extend_from_slice(&number.to_ne_bytes()).unwrap();

        self.i2c.write(Self::EEPROM_ADDRESS, buf.as_ref()).ok();
    }

    pub fn read_data<T: Num + PrimInt + FromPrimitive>(&mut self, address: u8) -> T {
        let size: usize = (T::zero().count_zeros() / 8) as usize;
        let buf: [u8; 1] = [address];
        let mut out_buf = heapless::Vec::<u8, U4>::new();
        out_buf.resize(size, 0).ok();

        self.i2c.write_read(Self::EEPROM_ADDRESS, &buf, out_buf.as_mut()).ok();

        // convert back from i32
        let temp = i32::from_ne_bytes(out_buf.deref().try_into().unwrap());
        let out: T = FromPrimitive::from_i32(temp).unwrap();

        out
    }

    pub fn write_bool(&mut self, address: u8, data: bool) {
        self.write_data(address, data as u8);
    }

    pub fn read_bool(&mut self, address: u8) -> bool {
        let temp: u8 = self.read_data(address);
        return temp != 0;
    }


}