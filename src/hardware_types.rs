
use stm32f1xx_hal::{
    gpio, spi, pac, i2c
};

// Status
pub type StatusLed1 = gpio::gpiob::PB5<gpio::Output<gpio::PushPull>>;
pub type StatusLed2 = gpio::gpiob::PB4<gpio::Output<gpio::PushPull>>;
pub type StatusLed3 = gpio::gpiob::PB3<gpio::Output<gpio::PushPull>>;

// Power Sense
pub type PowerSense = gpio::gpiob::PB15<gpio::Input<gpio::Floating>>;

// SPI definitions
pub type MOSIPin = gpio::gpioa::PA5<gpio::Alternate<gpio::PushPull>>;
pub type MISOPin = gpio::gpioa::PA6<gpio::Input<gpio::Floating>>;
pub type SCKPin =  gpio::gpioa::PA7<gpio::Alternate<gpio::PushPull>>;
pub type SPIPins = (MOSIPin, MISOPin, SCKPin);
pub type SPI = spi::Spi<pac::SPI1, spi::Spi1NoRemap, SPIPins, u16>;

// I2C
pub type SclPin = gpio::gpiob::PB10<gpio::Alternate<gpio::OpenDrain>>;
pub type SdaPin = gpio::gpiob::PB11<gpio::Alternate<gpio::OpenDrain>>;
pub type I2CPins = (SclPin, SdaPin);
pub type I2C = i2c::BlockingI2c<pac::I2C2, I2CPins>;

// Encoder
pub type EncoderChannelA = gpio::gpioa::PA8<gpio::Input<gpio::PullDown>>;
pub type EncoderChannelB = gpio::gpioa::PA9<gpio::Input<gpio::PullDown>>;
pub type EncoderChannelI = gpio::gpioa::PA10<gpio::Input<gpio::PullDown>>;

pub type EncoderChannel = gpio::Pxx<gpio::Input<gpio::PullDown>>;


