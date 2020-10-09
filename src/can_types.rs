#![allow(dead_code)]

use stm32f1xx_hal::can;

use heapless::{
    consts::*,
};

use core;
use core::convert::{
    Into, TryInto, TryFrom
};

pub trait TryIntoWith<T, U> { //: TryInto<T> {

    fn try_into(self, u: U) ->  Result<T, ()>;

}

pub enum FrameIdentifier {

    // output frames
    GetError = 0,
    GetTicks = 1,
    GetDebug = 2,

    // input frame
    SetTicks = 3,
    SetPolarity = 4,
    SetAbsoluteOffset = 5,

}

impl TryFrom<u8> for FrameIdentifier {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        return match value {
            0 => Ok(Self::GetError),
            1 => Ok(Self::GetTicks),
            2 => Ok(Self::GetDebug),
            3 => Ok(Self::SetTicks),
            4 => Ok(Self::SetPolarity),
            5 => Ok(Self::SetAbsoluteOffset),
            _ => Err(()) 
        }
    }

}

#[allow(dead_code)] // TODO: remove this
pub enum ErrorCode {
    None = 0,
    BadCanFrame = 1,
    EncoderFault = 2,
    EepromFault = 3,
    SpiFault = 4
}

#[derive(Clone)]
pub struct TickFrame {
    ticks: i32,
    absolute_position: u16 // check if we have room for this
}

impl TickFrame {

    fn new(ticks: i32, absolute_position: u16) -> Self {
        TickFrame {
            ticks, absolute_position
        }
    }

}

impl TryIntoWith<can::Frame, can::Id> for TickFrame {

    fn try_into(self, id: can::Id) -> Result<can::Frame, ()> {
        let mut buf = heapless::Vec::<u8, U8>::new();
        buf.push(FrameIdentifier::GetTicks as u8).unwrap();
        buf.extend_from_slice(&self.ticks.to_ne_bytes()).unwrap();
        buf.extend_from_slice(&self.absolute_position.to_ne_bytes()).unwrap();
        can::Frame::new(id, &buf)
    }

}


pub struct GetErrorFrame {
    error_code: u32
}

impl GetErrorFrame {

    pub fn new_from_u32(error_code: u32) -> Self {
        GetErrorFrame {
            error_code
        }
    }

    pub fn new(error_code: ErrorCode) -> Self {
        Self::new_from_u32(error_code as u32)
    }

}

impl TryIntoWith<can::Frame, can::Id> for GetErrorFrame {

    fn try_into(self, id: can::Id) -> Result<can::Frame, ()> {
        let mut buf = heapless::Vec::<u8, U8>::new();
        buf.push(FrameIdentifier::GetError as u8).unwrap();
        buf.extend_from_slice(&self.error_code.to_ne_bytes()).unwrap();
        can::Frame::new(id, &buf)
    }

}



pub struct GetDebugFrame {
    data: [u8; 7]
}

impl GetDebugFrame {

    pub fn new(data: [u8; 7]) -> Self {
        GetDebugFrame {
            data
        }
    }

}

impl Into<can::Frame> for GetDebugFrame {

    fn into(self) -> can::Frame {
        let mut buf = heapless::Vec::<u8, U8>::new();
        buf.push(FrameIdentifier::GetDebug as u8).unwrap();
        buf.extend_from_slice(&self.data).unwrap();
        can::Frame::new(self.id, &buf)
    }

}


pub struct SetTicksFrame {
    id: can::Id,
    ticks: i32,
}

impl SetTicksFrame {

    #[inline(always)]
    pub fn id(&self) -> can::Id { self.id }

    #[inline(always)]
    pub fn ticks(&self) -> i32  { self.ticks }

}

impl TryFrom<&can::Frame> for SetTicksFrame {
    type Error = ();

    fn try_from(frame: &can::Frame) -> Result<Self, Self::Error> {
        let dlc = frame.dlc();
        if dlc < (4 + 1) {
            return Err(());
        }

        if frame.data()[0] != (FrameIdentifier::SetTicks as u8) {
            return Err(())
        }
        let ticks = i32::from_ne_bytes(frame.data()[1..5].try_into().unwrap());
        Ok(
            SetTicksFrame {
                id: frame.id(),
                ticks
            }
        )
    }

}

pub struct SetPolarityFrame {
    id: can::Id,
    polarity: bool
}

impl SetPolarityFrame {


    #[inline(always)]
    pub fn id(&self) -> can::Id { self.id }

    #[inline(always)]
    pub fn polarity(&self) -> bool { self.polarity }

}

impl TryFrom<&can::Frame> for SetPolarityFrame {
    type Error = ();

    fn try_from(frame: &can::Frame) -> Result<Self, Self::Error> {
        let dlc = frame.dlc();
        if dlc < 2 { return Err(()) }
        if frame.data()[0] != (FrameIdentifier::SetPolarity as u8) { return Err(()) }

        Ok(SetPolarityFrame {
            id: frame.id(),
            polarity: (frame.data()[1] != 0)
        })
    }

}

pub struct SetAbsoluteOffsetFrame {
    offset: u16
}

impl SetAbsoluteOffsetFrame {

    #[inline(always)]
    pub fn offset(&self) -> u16 { self.offset }

}

impl TryFrom<&can::Frame> for SetAbsoluteOffsetFrame {
    type Error = ();

    fn try_from(frame: &can::Frame) -> Result<Self, Self::Error> {
        let dlc = frame.dlc();
        if dlc < 3 || frame.data()[0] != (FrameIdentifier::SetAbsoluteOffset as u8) { return Err(()) }
        let offset = u16::from_ne_bytes(frame.data()[1..3].try_into().unwrap());
        Ok(SetAbsoluteOffsetFrame {
            id: frame.id(),
            offset
        })
    }

}






