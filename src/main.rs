#![no_std]
#![no_main]

#[allow(dead_code)]

mod encoder;
mod memory;
mod status;
mod can_types;

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
    can,
    prelude::*,
    gpio,
    gpio::ExtiPin,
    pac,
    pac::{
        Interrupt
    }
};

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

#[allow(unused_imports)]
use micromath::F32Ext;

static SYSTEM_CLOCK: u32 = 72_000_000; // hz
static SECONDS_PER_CYCLE: f32 = 1.0 / SYSTEM_CLOCK as f32; // seconds

// type definitions
type StatusLed1 = gpio::gpiob::PB5<gpio::Output<gpio::PushPull>>;
type StatusLed2 = gpio::gpiob::PB4<gpio::Output<gpio::PushPull>>;
type StatusLed3 = gpio::gpiob::PB3<gpio::Output<gpio::PushPull>>;

type PowerSense = gpio::gpiob::PB15<gpio::Input<gpio::Floating>>;

// memory pools
pool!(
    #[allow(non_upper_case_globals)]
    CanTXPool: can::Frame
);

pool!(
    #[allow(non_upper_case_globals)]
    CanRXPool: can::Frame
);

pool!(
    #[allow(non_upper_case_globals)]
    StatusPool: status::LedStateTriple
);

fn alloc_frame(id: can::Id, data: &[u8]) -> Box<CanTXPool, Init> {
    let frame_box = CanTXPool::alloc().unwrap();
    frame_box.init(can::Frame::new(id, data))
}

// waiting for the CAN API to be released
// https://github.com/stm32-rs/stm32f1xx-hal/pull/215
// for now, we will use my janky hal branch

#[app(device=stm32f1::stm32f103, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {

    // resources 
    struct Resources {
        // status variables
        #[init(true)]
        power_ok: bool,

        // can variables
        #[init(false)]
        CAN_ok: bool,

        CAN_id: u8,

        can_tx: can::Tx<pac::CAN1>,
        can_tx_queue: BinaryHeap<Box<CanTXPool>, U8, Min>,
        #[init(0)]
        can_tx_count: usize,

        can_rx: can::Rx<pac::CAN1>,
        can_rx_queue: BinaryHeap<Box<CanRXPool>, U8, Min>,
        #[init(0)]
        can_rx_count: usize,

        // hardware
        status_handler: status::StatusHandler,

        power_sense: PowerSense,
        encoder: encoder::Encoder<spi::Spi<pac::SPI1, spi::Spi1NoRemap, encoder::SPIPins, u16>>,
        eeprom: memory::EEProm<i2c::BlockingI2c<pac::I2C2, memory::I2CPins>>
    }

    #[init(schedule=[can_tx, low_power])]
    fn init(cx: init::Context) -> init::LateResources {
        // memory init
        static mut CAN_TX_MEMORY: [u8; 256] = [0; 256];
        static mut CAN_RX_MEMORY: [u8; 256] = [0; 256];

        // hardware init
        let mut peripherals: rtic::Peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let exti = device.EXTI;
        
        // TODO: double check these values
        let clocks = rcc.cfgr.use_hse(8.mhz())
            .sysclk(72.mhz())
            .hclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .freeze(&mut flash.acr);

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

        let status_handler = status::StatusHandler::new(status1.downgrade(), status2.downgrade(), status3.downgrade());

        // power sense
        let power_sense: PowerSense = gpiob.pb15.into_floating_input(&mut gpiob.crh);
        // TODO: set interrupt for this maybe?

        // encoder pins (need to configure interrupts for this)
        let mut encoder_a = gpioa.pa8.into_pull_down_input(&mut gpioa.crh).downgrade();
        let mut encoder_b = gpioa.pa9.into_pull_down_input(&mut gpioa.crh).downgrade();
        let mut encoder_i = gpioa.pa10.into_pull_down_input(&mut gpioa.crh).downgrade();
        
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

        let eeprom = memory::EEProm::new(i2c);

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

        let spi1 =  spi::Spi::spi1(device.SPI1, spi_pins, &mut afio.mapr, spi_mode, 100.khz(), clocks, &mut rcc.apb2);
        let spi1 = spi1.frame_size_16bit();
        
        let encoder = encoder::Encoder::new(
            0, false, 0,
            encoder_a, encoder_b, encoder_i,
            spi1
        );

        // init can
        // the CAN and the USB periphs share SRAM, so we need to take ownership of both here to avoid errors
        let mut can = can::Can::new(device.CAN1, &mut rcc.apb1, device.USB);
        let can_pins = (gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh), gpiob.pb8.into_floating_input(&mut gpiob.crh));
        can.assign_pins(can_pins, &mut afio.mapr);
        // TODO: Configure this
        can.configure(|config| {
            // config.

        });

        let mut filters = can.split_filters().unwrap();

        // TODO: make sure filters only accept our CAN id
        filters.add(&can::Filter::new_standard(0).with_mask(0).allow_remote()).unwrap();
        filters.add(&can::Filter::new_extended(0).with_mask(0).allow_remote()).unwrap();

        let mut can_rx = can.take_rx(filters).unwrap();
        can_rx.enable_interrupts();

        let mut can_tx = can.take_tx().unwrap();
        can_tx.enable_interrupt();

        let can_tx_queue = BinaryHeap::new();
        CanTXPool::grow(CAN_TX_MEMORY);

        let can_rx_queue = BinaryHeap::new();
        CanRXPool::grow(CAN_RX_MEMORY);

        can.enable().ok();

        // eventually we should to set up DMA for some spi stuff

        // calculate CAN id
        // we don't need to hold references to the pins as we only use them on startup
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

        // schedule can tx
        cx.schedule.can_tx(now + (SYSTEM_CLOCK / 20).cycles()).unwrap();

        // schedule low power detection
        cx.schedule.low_power(now + (SYSTEM_CLOCK / 10).cycles()).unwrap();

        init::LateResources {
            CAN_id,
            can_tx,
            can_tx_queue,
            can_rx,
            can_rx_queue,
            status_handler,
            power_sense,
            encoder,
            eeprom
        }
    }

    #[idle(resources=[])]
    fn idle(mut _cx: idle::Context) -> ! {

        loop { cortex_m::asm::nop(); }
    }

    #[task(binds = EXTI9_5, priority = 2, resources=[encoder])]
    fn encoder_update(cx: encoder_update::Context) {
        cx.resources.encoder.handle_encoder_interrupt();
    }

    #[task(resources=[power_ok, power_sense], schedule=[low_power], spawn=[mem_tx])]
    fn low_power(cx: low_power::Context) {
        // run this at 200 hz? 500 hz?
        // check for shutdown condition and store data

        let power_sense = cx.resources.power_sense;

        // lambda closure bc im lazy
        let power_ok = || power_sense.is_high().unwrap();
        if !power_ok() {
            // spawn an i2c write task
            cx.spawn.mem_tx();

            // block till power is ok again
            while !power_ok() { cortex_m::asm::nop(); }
        }

        // schedule this at 500hz
        let now = Instant::now();
        cx.schedule.low_power(now + (SYSTEM_CLOCK / 500).cycles());
    }

    #[task(resources=[eeprom, encoder])]
    fn mem_tx(cx: mem_tx::Context) {
        // Send data to eeprom memory
        // this will be called on shutdown
        let eeprom = cx.resources.eeprom;
        let encoder = cx.resources.encoder;

        let ticks = encoder.count();
        let inverted = encoder.inverted();
        let absolute_offset = encoder.absolute_offset();

        eeprom.write_data(memory::Address::Ticks as u8, ticks);
        eeprom.write_bool(memory::Address::Polarity as u8, inverted);
        eeprom.write_data(memory::Address::AbsoluteOffset as u8, absolute_offset);
    }

    #[task(resources=[eeprom, encoder])]
    fn mem_rx(cx: mem_rx::Context) {
        // Recieve data from eeprom memory
        // this will be called on shutdown
        let eeprom = cx.resources.eeprom;
        let _encoder = cx.resources.encoder;

        let _ticks: i32 = eeprom.read_data(memory::Address::Ticks as u8);
        let _inverted: bool = eeprom.read_bool(memory::Address::Polarity as u8);
        let _absolute_offset: u16 = eeprom.read_data(memory::Address::AbsoluteOffset as u8);
    }


    #[task(resources=[CAN_id, can_tx, can_tx_queue, can_tx_count], schedule=[can_tx])]
    fn can_tx(cx: can_tx::Context) {
        // send CAN frames over the network
        // call this at 20 hz maybe?
        let tx = cx.resources.can_tx;
        let tx_queue = cx.resources.can_tx_queue;

        // we won't use an interrupt for now, just send all messages in queue at 20hz
        // tx.clear_interrupt_flags();

        while let Some(frame) = tx_queue.peek() {
            match tx.transmit(&frame) {
                Ok(None) => {
                    tx_queue.pop();
                    *cx.resources.can_tx_count += 1;
                }
                Ok(pending_frame) => {
                    // make sure we send the frame with the highest priority
                    tx_queue.pop();
                    if let Some(frame) = pending_frame {
                        tx_queue.push(CanTXPool::alloc().unwrap().init(frame)).unwrap();
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                _ => unreachable!(),
            }
        }

        // schedule itself at 20hz
        let now = Instant::now();
        cx.schedule.can_tx(now + (SYSTEM_CLOCK / 20).cycles());
    }

    #[task(binds = USB_LP_CAN_RX0, resources=[CAN_id, can_rx, can_tx_queue])]
    fn can_rx0(cx: can_rx0::Context) {
        loop {
            match cx.resources.can_rx.receive() {
                Ok(frame) => {
                    // handle frames here
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // ignore this
            }
        }
    }

    #[task(binds = CAN_RX1)]
    fn can_rx1(_: can_rx1::Context) {
        // Jump to the other interrupt handler which handles both RX fifos.
        rtic::pend(Interrupt::USB_LP_CAN_RX0);
    }

    extern "C" {
        // delcare unused interrupts here so that RTIC can use them
        
        fn USART1();
        fn USART2();
        fn USART3();

    }

};