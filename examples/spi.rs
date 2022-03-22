//! Example of configuring spi.
//! Target board: STM32F3DISCOVERY
#![no_std]
#![no_main]

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m::asm;
use cortex_m_rt::entry;

use hal::pac;
use hal::prelude::*;
use hal::spi::Spi;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    // Configure pins for SPI
    let sck = gpioc
        .pc10
        .into_alternate(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrh);
    let miso = gpioc
        .pc11
        .into_alternate(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrh);
    let mosi = gpioc
        .pc12
        .into_alternate(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrh);

    let mut spi = Spi::new(dp.SPI3, (sck, miso, mosi), 3.MHz(), clocks, &mut rcc.apb1);

    // Create an `u8` array, which can be transfered via SPI.
    let msg_send: [u8; 8] = [0xD, 0xE, 0xA, 0xD, 0xB, 0xE, 0xE, 0xF];
    // Copy the array, as it would be mutually shared in `transfer` while simultaneously would be
    // immutable shared in `assert_eq`.
    let mut msg_sending = msg_send;
    // Transfer the content of the array via SPI and receive it's output.
    // When MOSI and MISO pins are connected together, `msg_received` should receive the content.
    // from `msg_sending`
    let msg_received = spi.transfer(&mut msg_sending).unwrap();

    // Check, if msg_send and msg_received are identical.
    // This succeeds, when master and slave of the SPI are connected.
    assert_eq!(msg_send, msg_received);

    loop {
        asm::wfi();
    }
}
