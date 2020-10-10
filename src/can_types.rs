#![allow(dead_code)]

use stm32f1xx_hal::can;

use heapless::consts::*;

use core::convert::{Into, TryFrom, TryInto};

use core::ops::Deref;

pub trait TryIntoWith<T, U> {
    fn try_into_with(self, u: U) -> Result<T, ()>;
}

#[allow(dead_code)] // TODO: remove this
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum ErrorCode {
    None = 0,
    BadCanFrame = 1,
    EncoderFault = 2,
    EepromFault = 3,
    SpiFault = 4,
}

#[derive(Clone, Copy)]
pub enum Frame {
    // output frames
    GetTicks { relative: i32, absolute: u16 },
    GetError { error_code: u8 },
    GetDebug { data: [u8; 7] },

    // input frames
    SetTicks { ticks: i32 },
    SetPolarity { inverted: bool },
    SetAbsoluteOffset { offset: u16 },

    // memory operations
    SaveToMemory,
    ClearMemory,
}

impl Into<u8> for Frame {
    fn into(self) -> u8 {
        match self {
            Self::GetError { error_code: _ } => 0,
            Self::GetTicks {
                relative: _,
                absolute: _,
            } => 1,
            Self::GetDebug { data: _ } => 2,

            Self::SetTicks { ticks: _ } => 3,
            Self::SetPolarity { inverted: _ } => 4,
            Self::SetAbsoluteOffset { offset: _ } => 5,

            Self::SaveToMemory => 6,
            Self::ClearMemory => 7,
        }
    }
}

impl TryFrom<u8> for Frame {
    type Error = ();

    fn try_from(ident: u8) -> Result<Self, Self::Error> {
        match ident {
            0 => Ok(Self::GetError { error_code: 0 }),
            1 => Ok(Self::GetTicks {
                relative: 0,
                absolute: 0,
            }),
            2 => Ok(Self::GetDebug { data: [0; 7] }),

            3 => Ok(Self::SetTicks { ticks: 0 }),
            4 => Ok(Self::SetPolarity { inverted: false }),
            5 => Ok(Self::SetAbsoluteOffset { offset: 0 }),

            6 => Ok(Self::SaveToMemory),
            7 => Ok(Self::ClearMemory),
            _ => Err(()),
        }
    }
}

impl TryIntoWith<can::Frame, can::Id> for Frame {
    fn try_into_with(self, id: can::Id) -> Result<can::Frame, ()> {
        let mut buf = heapless::Vec::<u8, U8>::new();
        // buf.push(self.try_into().unwrap());
        buf.push(self.try_into().unwrap()).unwrap();

        // only need to do output frames here
        match self {
            Self::GetTicks { relative, absolute } => {
                buf.extend_from_slice(&i32::to_ne_bytes(relative)).unwrap();
                buf.extend_from_slice(&u16::to_ne_bytes(absolute)).unwrap();
            }
            Self::GetError { error_code } => {
                buf.push(error_code).unwrap();
            }
            Self::GetDebug { data } => {
                buf.extend_from_slice(&data).unwrap();
            }
            _ => return Err(()),
        }

        can::Frame::new(id, buf.deref())
    }
}

impl TryFrom<&can::Frame> for Frame {
    type Error = ();

    fn try_from(f: &can::Frame) -> Result<Self, Self::Error> {
        if !f.is_data_frame() {
            return Err(());
        }

        let _id = f.id();
        let dlc = f.dlc();

        if dlc < 1 {
            return Err(());
        }
        let ident = f.data()[0];

        let mut buf = heapless::Vec::<u8, U7>::new();
        buf.extend_from_slice(&f.data()[1..dlc]).unwrap();

        match Self::try_from(ident) {
            Ok(frame) => match frame {
                Self::SetTicks { ticks: _ } => {
                    if buf.len() >= 4 {
                        let t = i32::from_ne_bytes(buf[0..4].try_into().unwrap());
                        Ok(Self::SetTicks { ticks: t })
                    } else {
                        Err(())
                    }
                }
                Self::SetPolarity { inverted: _ } => {
                    if buf.len() >= 1 {
                        let i = buf[0] != 0;
                        Ok(Self::SetPolarity { inverted: i })
                    } else {
                        Err(())
                    }
                }
                Self::SetAbsoluteOffset { offset: _ } => {
                    if dlc >= 2 {
                        let o = u16::from_ne_bytes(buf[0..2].try_into().unwrap());
                        Ok(Self::SetAbsoluteOffset { offset: o })
                    } else {
                        Err(())
                    }
                }
                Self::ClearMemory => Ok(Self::ClearMemory),
                Frame::SaveToMemory => Ok(Self::SaveToMemory),
                _ => Err(()),
            },
            Err(_) => Err(()),
        }
    }
}
