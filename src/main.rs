#![no_std]
#![no_main]

#[allow(dead_code)]

mod hardware_types;
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

use core::convert::{
    Into, TryInto
};

use core::ops::{
    Deref
};

use cortex_m_rt::{entry, exception, ExceptionFrame};

extern crate static_assertions as sa;

// size of can::Frame is 16 bytes
sa::assert_eq_size!(can::Frame, [u8; 4 + 8 + 4]);


const HSE_CLOCK_MHZ: u32 = 8;
const SYSTEM_CLOCK_MHZ: u32 = 72;
const SYSTEM_CLOCK: u32 = SYSTEM_CLOCK_MHZ * 1E6 as u32; // hz

// Size of 32 element frame queue is 16 bytes * 32 elements = 512 bytes
type Queue<T> = heapless::mpmc::Q32<T>;
type FrameQueue = Queue<can::Frame>;
type StatusQueue = Queue<status::LedStateTriple>;

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

        CAN_id: can::Id,

        can_tx: can::Tx<pac::CAN1>,
        can_tx_queue: FrameQueue,
        #[init(0)]
        can_tx_count: usize,

        can_rx: can::Rx<pac::CAN1>,
        can_rx_queue: FrameQueue,
        #[init(0)]
        can_rx_count: usize,

        // status queue
        status_queue: StatusQueue,

        // hardware
        status_handler: status::StatusHandler,

        power_sense: hardware_types::PowerSense,
        encoder: encoder::Encoder<hardware_types::SPI>,
        eeprom: memory::EEProm<hardware_types::I2C>
    }

    #[init(schedule=[can_tx, low_power])]
    fn init(cx: init::Context) -> init::LateResources {
        // hardware init
        let mut peripherals: rtic::Peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let exti = device.EXTI;
        
        // TODO: double check these values
        let clocks = rcc.cfgr.use_hse((HSE_CLOCK_MHZ).mhz())
            .sysclk((SYSTEM_CLOCK_MHZ).mhz())
            .hclk((SYSTEM_CLOCK_MHZ).mhz())
            .pclk1((SYSTEM_CLOCK_MHZ / 2).mhz())
            .pclk2((SYSTEM_CLOCK_MHZ).mhz())
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
        let status1: hardware_types::StatusLed1 = gpiob.pb5.into_push_pull_output_with_state(&mut gpiob.crl, gpio::State::Low);
        let status2: hardware_types::StatusLed2 = status2.into_push_pull_output_with_state(&mut gpiob.crl, gpio::State::Low);
        let status3: hardware_types::StatusLed3 = status3.into_push_pull_output_with_state(&mut gpiob.crl, gpio::State::Low);

        let status_handler = status::StatusHandler::new(status1, status2, status3);

        // power sense
        let power_sense: hardware_types::PowerSense = gpiob.pb15.into_floating_input(&mut gpiob.crh);

        // encoder pins (need to configure interrupts for this)
        let mut encoder_a: hardware_types::EncoderChannelA = gpioa.pa8.into_pull_down_input(&mut gpioa.crh);
        let mut encoder_b: hardware_types::EncoderChannelB = gpioa.pa9.into_pull_down_input(&mut gpioa.crh);
        let mut encoder_i: hardware_types::EncoderChannelI = gpioa.pa10.into_pull_down_input(&mut gpioa.crh);
        
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
        let i2c_pins: hardware_types::I2CPins = (gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh));


        let i2c = i2c::BlockingI2c::i2c2(
            device.I2C2,
            i2c_pins,
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
        
        let encoder: encoder::Encoder<hardware_types::SPI> = encoder::Encoder::new(
            0, false, 0,
            encoder_a, encoder_b, encoder_i,
            spi1
        );

        // eventually we should to set up DMA for some spi stuff

        // calculate CAN id
        // we don't need to hold references to the pins as we only use them on startup
        let mut CAN_id: u32 = 0;
        CAN_id += (gpiob.pb14.into_pull_down_input(&mut gpiob.crh).is_high().unwrap() as u32) << 0;
        CAN_id += (gpiob.pb13.into_pull_down_input(&mut gpiob.crh).is_high().unwrap() as u32) << 1;
        CAN_id += (gpiob.pb12.into_pull_down_input(&mut gpiob.crh).is_high().unwrap() as u32) << 2;
        CAN_id += (gpiob.pb2. into_pull_down_input(&mut gpiob.crl).is_high().unwrap() as u32) << 3;
        CAN_id += (gpiob.pb1. into_pull_down_input(&mut gpiob.crl).is_high().unwrap() as u32) << 4;
        CAN_id += (gpiob.pb0. into_pull_down_input(&mut gpiob.crl).is_high().unwrap() as u32) << 5;
        CAN_id += (gpioa.pa4. into_pull_down_input(&mut gpioa.crl).is_high().unwrap() as u32) << 6;
        CAN_id += (gpioa.pa3. into_pull_down_input(&mut gpioa.crl).is_high().unwrap() as u32) << 7;

        let CAN_id = can::Id::Standard(CAN_id);

        // init can
        // the CAN and the USB periphs share SRAM, so we need to take ownership of both here to avoid errors
        let mut can = can::Can::new(device.CAN1, &mut rcc.apb1, device.USB);
        let can_pins = (gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh), gpiob.pb8.into_floating_input(&mut gpiob.crh));
        can.assign_pins(can_pins, &mut afio.mapr);
        // TODO: Configure this
        can.configure(|_config| {
            // config.

        });

        let mut filters = can.split_filters().unwrap();

        // filter any undesired CAN ids
        {
            let can::Id::Standard(value) = CAN_id;
            let extended = can::Id::Extended(value);

            filters.add(&can::Filter::new(CAN_id).with_mask(0)).unwrap();
            filters.add(&can::Filter::new(extended).with_mask(0)).unwrap();
        }
        

        let mut can_rx = can.take_rx(filters).unwrap();
        can_rx.enable_interrupts();

        let mut can_tx = can.take_tx().unwrap();
        can_tx.enable_interrupt();

        let can_tx_queue = FrameQueue::new();

        let can_rx_queue = FrameQueue::new();

        let status_queue = StatusQueue::new();
        

        can.enable().ok();

        // initalize timer
        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        let now = cx.start;

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
            status_queue,
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

    #[task(priority = 1, resources=[power_ok, power_sense], schedule=[low_power], spawn=[mem_tx])]
    fn low_power(cx: low_power::Context) {
        // run this at 200 hz? 500 hz?
        // check for shutdown condition and store data

        let power_sense = cx.resources.power_sense;

        // lambda closure bc im lazy
        let power_ok = || power_sense.is_high().unwrap();
        if !power_ok() {
            // spawn an i2c write task
            cx.spawn.mem_tx().unwrap();

            // block till power is ok again
            while !power_ok() { cortex_m::asm::nop(); }
        }

        // schedule this at 500hz
        let now = Instant::now();
        cx.schedule.low_power(now + (SYSTEM_CLOCK / 500).cycles()).unwrap();
    }

    #[task(resources=[eeprom, encoder])]
    fn mem_tx(cx: mem_tx::Context) {
        // Send data to eeprom memory
        // this will be called on shutdown
        let eeprom = cx.resources.eeprom;
        let mut encoder = cx.resources.encoder;

        // get variables
        let (ticks, inverted, absolute_offset) = encoder.lock(|encoder| {
            (
                encoder.ticks(),
                encoder.inverted(),
                encoder.absolute_offset(),
            )
        });

        eeprom.write_data(memory::Address::Ticks as u8, ticks);
        eeprom.write_bool(memory::Address::Polarity as u8, inverted);
        eeprom.write_data(memory::Address::AbsoluteOffset as u8, absolute_offset);
    }

    #[task(resources=[eeprom, encoder])]
    fn mem_rx(cx: mem_rx::Context) {
        // Recieve data from eeprom memory
        // this will be called on shutdown
        let eeprom = cx.resources.eeprom;
        let mut encoder = cx.resources.encoder;

        let ticks = eeprom.read_data(memory::Address::Ticks as u8);
        let inverted = eeprom.read_bool(memory::Address::Polarity as u8);
        let absolute_offset = eeprom.read_data(memory::Address::AbsoluteOffset as u8);


        encoder.lock(|encoder| {
            encoder.set_ticks(ticks);
            encoder.set_inverted(inverted);
            encoder.set_absolute_offset(absolute_offset);
        });

    }

    #[task(resources=[can_rx_queue, can_tx_queue, CAN_id, encoder], schedule=[handle_can_rx])]
    fn handle_can_rx(cx: handle_can_rx::Context) {
        use can_types::*;

        let rx_queue = cx.resources.can_rx_queue;
        let tx_queue = cx.resources.can_tx_queue;
        
        let mut encoder = cx.resources.encoder;

        // call this at 20hz
        // probably want to lock queue here
        while let Some(frame) = &rx_queue.dequeue() {

            // ignore frames without out CAN id (We may be able to delete this because of CAN filters)
            if frame.id() != (*cx.resources.CAN_id) { continue; }
            //  ignore frames with 0 length
            if frame.dlc() == 0 { continue; }
            // get frame ident in order to match
            let ident: FrameIdentifier = frame.data()[0].try_into().unwrap();

            let result = match ident {
                FrameIdentifier::SetTicks => {
                    let parsed_frame: Option<SetTicksFrame> = frame.deref().try_into().ok();

                    let res: Result<(), ()> =  match parsed_frame {
                        Some(f) => {
                            let ticks = f.ticks();
                            encoder.lock(|encoder| encoder.set_ticks(ticks));
                            Ok(())
                        },
                        None => {
                            Err(())
                        }
                    };

                    res   
                },
                FrameIdentifier::SetPolarity => {
                    let parsed_frame: Option<SetPolarityFrame> = frame.deref().try_into().ok();
                    
                    let res: Result<(), ()> = match parsed_frame {
                        Some(f) => {
                            encoder.lock(|encoder| encoder.set_inverted(f.polarity()));
                            Ok(())
                        },
                        None => {Err(())}
                    };


                    res
                },
                FrameIdentifier::SetAbsoluteOffset => {
                    let parsed_frame: Option<SetAbsoluteOffsetFrame> = frame.deref().try_into().ok();
                    
                    let res: Result<(), ()> = match parsed_frame {
                        Some(f) => {
                            encoder.lock(|encoder| encoder.set_absolute_offset(f.offset()));
                            Ok(())
                        },
                        None => {Err(())}
                    };


                    res
                },
                _ => { Err(()) }
            };

            match result {
                Ok(_) => {},
                Err(()) => {
                    use can_types::{
                        GetErrorFrame,
                        ErrorCode
                    };
                    let error_frame = GetErrorFrame::new(*cx.resources.CAN_id, ErrorCode::BadCanFrame);
                    tx_queue.enqueue(error_frame.try_into().unwrap()).unwrap();
                }
            }


        }

        let now = Instant::now();
        cx.schedule.handle_can_rx(now + (SYSTEM_CLOCK / 20).cycles()).unwrap();
    }


    #[task(resources=[can_tx, can_tx_queue, can_tx_count], schedule=[can_tx])]
    fn can_tx(cx: can_tx::Context) {
        // send CAN frames over the network
        // call this at 20 hz maybe?
        let tx = cx.resources.can_tx;
        let tx_queue = cx.resources.can_tx_queue;

        // we won't use an interrupt for now, just send all messages in queue at 20hz
        // tx.clear_interrupt_flags();

        while let Some(frame) = tx_queue.dequeue() {
            match tx.transmit(&frame) {
                Ok(None) => {
                    *cx.resources.can_tx_count += 1;
                }
                Err(nb::Error::WouldBlock) => break,
                _ => unreachable!(),
            }
        }

        // schedule itself at 20hz
        cx.schedule.can_tx(Instant::now() + (SYSTEM_CLOCK / 20).cycles()).unwrap();
    }

    #[task(binds = USB_LP_CAN_RX0, resources=[can_rx, can_rx_queue, can_rx_count])]
    fn can_rx0(cx: can_rx0::Context) {
        let rx = cx.resources.can_rx;
        let rx_queue = cx.resources.can_rx_queue;

        loop {
            match rx.receive() {
                Ok(frame) => {
                    rx_queue.enqueue(frame).unwrap();
                    *cx.resources.can_rx_count += 1;
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

    #[task(resources=[status_handler], schedule=[update_status])]
    fn update_status(cx: update_status::Context) {
        let _status_handler = cx.resources.status_handler;

        // call at 8 hz
        cx.schedule.update_status(Instant::now() + (SYSTEM_CLOCK / 8).cycles()).unwrap();
    }


    extern "C" {
        // delcare unused interrupts here so that RTIC can use them
        
        fn USART1();
        fn USART2();
        fn USART3();

        fn CAN2();
        
        fn SPI2();
        fn SPI3();

    }

};