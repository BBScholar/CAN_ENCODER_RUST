#![no_std]
#![no_main]

mod encoder;
mod can;
mod memory;

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::peripheral::DWT;
// use cortex_m_semihosting::hprintln;

use heapless::{
    consts::*,
    binary_heap::{BinaryHeap, Min},
    pool,
    pool::{
        Init,
        singleton::{Box, Pool}
    }
};

#[allow(unused_imports)]
use embedded_hal::digital::v2::{
    OutputPin,
    InputPin
};

use stm32f1xx_hal::{
    i2c,
    spi,
    prelude::*,
    gpio,
    gpio::ExtiPin,
    pac::{
        Interrupt
    }
};

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

#[allow(unused_imports)]
use micromath::F32Ext;

static SYSTEM_CLOCK: u32 = 8_000_000; // hz
static SECONDS_PER_CYCLE: f32 = 1.0 / SYSTEM_CLOCK as f32; // seconds

// type definitions
type StatusLed1 = gpio::gpiob::PB5<gpio::Output<gpio::PushPull>>;
type StatusLed2 = gpio::gpiob::PB4<gpio::Output<gpio::PushPull>>;
type StatusLed3 = gpio::gpiob::PB3<gpio::Output<gpio::PushPull>>;

type PowerSense = gpio::gpiob::PB15<gpio::Input<gpio::Floating>>;

// waiting for the CAN API to be released
// https://github.com/stm32-rs/stm32f1xx-hal/pull/215

#[app(device=stm32f1::stm32f103, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {

    // resources 
    struct Resources {
        // status variables
        #[init(true)]
        power_ok: bool,

        #[init(false)]
        CAN_ok: bool,

        CAN_id: u8,

        // hardware
        status1: StatusLed1,
        status2: StatusLed2,
        status3: StatusLed3,

        power_sense: PowerSense,
        encoder: encoder::Encoder<>,
        eeprom: memory::EEProm<>
    }

    #[init(schedule = [send_can_messages])]
    fn init(cx: init::Context) -> init::LateResources {
        // hardware init
        let mut peripherals: rtic::Peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut exti = device.EXTI;
        
        // TODO: configure the clocks, currently have no idea how to do this
        let clocks = rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

        // gpio structs
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // disable jtag connection to enable alternate pin use
        let (_, status3, status2) = afio.mapr.disable_jtag(
            gpioa.pa15,
            gpiob.pb3,
            gpiob.pb4
        );

        // status leds
        let status1: StatusLed1 = gpiob.pb5.into_push_pull_output_with_state(&mut gpiob.crl, gpio::State::Low);
        let status2: StatusLed2 = status2.into_push_pull_output_with_state(&mut gpiob.crl, gpio::State::Low);
        let status3: StatusLed3 = status3.into_push_pull_output_with_state(&mut gpiob.crl, gpio::State::Low);

        // power sense
        let power_sense: PowerSense = gpiob.pb15.into_floating_input(&mut gpiob.crh);
        // TODO: set interrupt for this maybe?

        // encoder pins (need to configure interrupts for this)
        let encoder_a = gpioa.pa8.into_pull_down_input(&mut gpioa.crh).downgrade();
        let encoder_b = gpioa.pa9.into_pull_down_input(&mut gpioa.crh).downgrade();
        let encoder_i = gpioa.pa10.into_pull_down_input(&mut gpioa.crh).downgrade();
        
        encoder_a.make_interrupt_source(&mut afio);
        encoder_a.trigger_on_edge(&exti, gpio::Edge::RISING_FALLING);
        encoder_a.enable_interrupt(&exti);

        encoder_b.make_interrupt_source(&mut afio);
        encoder_b.trigger_on_edge(&exti, gpio::Edge::RISING_FALLING);
        encoder_b.enable_interrupt(&exti);

        encoder_i.make_interrupt_source(&mut afio);
        encoder_i.trigger_on_edge(&exti, gpio::Edge::RISING);
        encoder_i.enable_interrupt(&exti);
        

        // memory i2c
        let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

        let i2c = i2c::BlockingI2c::i2c2(
            device.I2C2,
            (scl, sda),
            i2c::Mode::Standard {
                frequency: 100_000.hz()
            },
            clocks,
            &mut rcc.apb1,

            // dont really know what these 4 parameters should actually be
            50,   // start_timeout_us
            5,    // start_retries
            100,  // addr_timeout_us
            100   // data_timeout_us
        );

        let eeprom = memory::EEProm {
            i2c
        };

        // encoder spi
        let spi_pins = (
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa6.into_floating_input(&mut gpioa.crl),
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
        );

        let spi_mode = spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition
        };

        let spi =  spi::Spi::spi1(device.SPI1, spi_pins, &mut afio.mapr, spi_mode, 100.khz(), clocks, &mut rcc.apb2);

        let encoder = encoder::Encoder::new(
            0, false, 0,
            encoder_a, encoder_b, encoder_i,
            spi
        );

        // eventually we need to set up DMA for some spi stuff


        // CAN ID
        let mut CAN_id: u8 = 0;
        CAN_id += (gpiob.pb14.into_pull_down_input(&mut gpiob.crh).is_high().unwrap() as u8) << 0;
        CAN_id += (gpiob.pb13.into_pull_down_input(&mut gpiob.crh).is_high().unwrap() as u8) << 1;
        CAN_id += (gpiob.pb12.into_pull_down_input(&mut gpiob.crh).is_high().unwrap() as u8) << 2;
        CAN_id += (gpiob.pb2.into_pull_down_input(&mut gpiob.crl) .is_high().unwrap() as u8) << 3;
        CAN_id += (gpiob.pb1.into_pull_down_input(&mut gpiob.crl) .is_high().unwrap() as u8) << 4;
        CAN_id += (gpiob.pb0.into_pull_down_input(&mut gpiob.crl) .is_high().unwrap() as u8) << 5;
        CAN_id += (gpioa.pa4.into_pull_down_input(&mut gpioa.crl) .is_high().unwrap() as u8) << 6;
        CAN_id += (gpioa.pa3.into_pull_down_input(&mut gpioa.crl) .is_high().unwrap() as u8) << 7;

        // initalize timer
        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        let now = cx.start;
        // hprintln!("init @ {:?}", now).unwrap();

        cx.schedule.send_can_messages(now + 8_000_000.cycles()).unwrap();

        init::LateResources {
            CAN_id,
            status1,
            status2,
            status3,
            power_sense,
            encoder,
            eeprom
        }
    }

    #[idle(resources=[])]
    fn idle(mut _cx: idle::Context) -> ! {

        loop {}
    }

    #[task(binds = EXTI9_5, priority = 2, resources=[encoder])]
    fn encoder_update(cx: encoder_update::Context) {
        cx.resources.encoder.handle_encoder_interrupt();
    }

    #[task]
    fn low_power() {
        
    }

    #[task]
    fn mem_tx(cx: mem_tx::Context) {
        // Send data to eeprom memory
        // this will be called on shutdown
    }

    #[task]
    fn mem_rx(cx: mem_rx::Context) {
        // Recieve data from eeprom memory
        // this will be called on shutdown
    }


    #[task(resources=[CAN_id])]
    fn can_tx(cx: can_tx::Context) {
        // send CAN frames over the network
        // call this at 20 hz maybe?
    }

    #[task(resources=[CAN_id])]
    fn can_rx(cx: can_rx::Conext) {
        // recieve CAN frames from over the network
    }

    extern "C" {
        // delcare unused interrupts here so that RTIC can use them
        
        fn USART1();
        fn USART2();
        fn USART3();

        fn CAN1();
        fn CAN2();
    }

};