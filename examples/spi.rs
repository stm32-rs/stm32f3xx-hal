//! Example of configuring spi.
//! Target board: STM32F3DISCOVERY
#![no_std]
#![no_main]

use core::convert::TryInto;

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m::asm;
use cortex_m_rt::entry;

use hal::pac;
use hal::prelude::*;
use hal::spi::{Mode, Phase, Polarity, Spi};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let clocks = rcc
        .cfgr
        .use_hse(8u32.MHz())
        .sysclk(48u32.MHz())
        .pclk1(24u32.MHz())
        .freeze(&mut flash.acr);

    // Configure pins for SPI
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let mut spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        spi_mode,
        3u32.MHz().try_into().unwrap(),
        clocks,
        &mut rcc.apb2,
    );

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
