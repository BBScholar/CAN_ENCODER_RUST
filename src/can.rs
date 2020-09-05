
pub enum FrameType {
    Error    = 0,
    Ticks    = 1,
    Absolute = 2,
    Debug    = 3
}

pub enum ErrorType {
    None = 0,
    EncoderError = 1,
    LowPower = 2
}