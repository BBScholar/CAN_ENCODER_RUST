
use stm32f1xx_hal::can;

struct CustomFrame {
    frame: can::Frame
}