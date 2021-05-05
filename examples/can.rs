//! Example of using CAN.
#![no_std]
#![no_main]

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m::asm;
use cortex_m_rt::entry;

use hal::pac;
use hal::prelude::*;
use hal::watchdog::IndependentWatchDog;

use bxcan::filter::Mask32;
use bxcan::{Frame, StandardId};
use hal::can::Can;
use nb::block;

// Each "node" needs a different ID, we set up a filter too look for messages to this ID
// Max value is 8 because we use a 3 bit mask in the filter
const ID: u16 = 0b100;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let _clocks = rcc
        .cfgr
        .use_hse(32.MHz())
        .hclk(64.MHz())
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .pclk2(64.MHz())
        .freeze(&mut flash.acr);

    // Configure CAN RX and TX pins (AF9)
    let rx = gpioa
        .pa11
        .into_af9_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let tx = gpioa
        .pa12
        .into_af9_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    // Initialize the CAN peripheral
    let can = Can::new(dp.CAN, tx, rx, &mut rcc.apb1);

    let mut can = bxcan::Can::new(can);

    // Use loopback mode: No pins need to be assigned to peripheral.
    // APB1 (PCLK1): 64MHz, Bit rate: 500kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    can.modify_config()
        .set_bit_timing(0x001c_0003)
        .set_loopback(false)
        .set_silent(false);

    let mut filters = can.modify_filters();

    filters.enable_bank(0, Mask32::accept_all());

    // Enable filters.
    drop(filters);

    // Sync to the bus and start normal operation.
    block!(can.enable()).ok();

    let mut led0 = gpiob
        .pb15
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    led0.set_high().unwrap();

    // Watchdog makes sure this gets restarted periodically if nothing happens
    let mut iwdg = IndependentWatchDog::new(dp.IWDG);
    iwdg.stop_on_debug(&dp.DBGMCU, true);
    iwdg.start(100.milliseconds());

    // Send an initial message!
    asm::delay(100_000);
    let data: [u8; 1] = [1];

    let frame = Frame::new_data(StandardId::new(ID).unwrap(), data);

    block!(can.transmit(&frame)).expect("Cannot send first CAN frame");

    loop {
        let rcv_frame = block!(can.receive()).expect("Cannot receive CAN frame");

        if let Some(d) = rcv_frame.data() {
            let counter = d[0].wrapping_add(1);

            if counter % 3 == 0 {
                led0.toggle().unwrap();
            }

            let data: [u8; 1] = [counter];
            let frame = Frame::new_data(StandardId::new(ID).unwrap(), data);

            block!(can.transmit(&frame)).expect("Cannot send CAN frame");
        }

        iwdg.feed();

        asm::delay(1_000_000);
    }
}
