//! Example of using CAN.
#![no_std]
#![no_main]

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m::asm;
use cortex_m_rt::entry;

use hal::prelude::*;
use hal::stm32;
use hal::watchdog::IndependentWatchDog;

use hal::can::{Can, CanFilter, CanFrame, CanId, Filter, Frame, Receiver, Transmitter};
use nb::block;

// Each "node" needs a different ID, we set up a filter too look for messages to this ID
// Max value is 8 because we use a 3 bit mask in the filter
const ID: u16 = 0b100;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let _clocks = rcc
        .cfgr
        .use_hse(32.mhz())
        .sysclk(32.mhz())
        .pclk1(16.mhz())
        .pclk2(16.mhz())
        .freeze(&mut flash.acr);

    // Configure CAN RX and TX pins (AF9)
    let can_rx = gpioa.pa11.into_af9(&mut gpioa.moder, &mut gpioa.afrh);
    let can_tx = gpioa.pa12.into_af9(&mut gpioa.moder, &mut gpioa.afrh);

    // Initialize the CAN peripheral
    let can = Can::new(dp.CAN, can_rx, can_tx, &mut rcc.apb1);

    // Uncomment the following line to enable CAN interrupts
    // can.listen(Event::Fifo0Fmp);

    let (mut can_tx, mut rx0, _rx1) = can.split();

    let mut led0 = gpiob
        .pb15
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    led0.set_high().unwrap();

    let filter = CanFilter::from_mask(0b100, ID as u32);
    rx0.set_filter(filter);

    // Watchdog makes sure this gets restarted periodically if nothing happens
    let mut iwdg = IndependentWatchDog::new(dp.IWDG);
    iwdg.stop_on_debug(&dp.DBGMCU, true);
    iwdg.start(100.ms());

    // Send an initial message!
    asm::delay(100_000);
    let data: [u8; 1] = [0];

    let frame = CanFrame::new_data(CanId::BaseId(ID), &data);

    block!(can_tx.transmit(&frame)).expect("Cannot send first CAN frame");

    loop {
        let rcv_frame = block!(rx0.receive()).expect("Cannot receive CAN frame");

        if let Some(d) = rcv_frame.data() {
            let counter = d[0].wrapping_add(1);

            if counter % 3 == 0 {
                led0.toggle().unwrap();
            }

            let data: [u8; 1] = [counter];
            let frame = CanFrame::new_data(CanId::BaseId(ID), &data);

            block!(can_tx.transmit(&frame)).expect("Cannot send CAN frame");
        }

        iwdg.feed();

        asm::delay(1_000_000);
    }
}
