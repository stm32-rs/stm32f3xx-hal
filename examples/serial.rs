//! Example of transmitting data over serial interface using UART.
//! For this to work, the PA9 and PA10 pins must be connected.
//! Target board: STM32F3DISCOVERY

#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f3xx_hal::{pac, prelude::*, serial::Serial};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    // Setup UART pins.
    let uart_pins = (
        gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
        gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
    );

    let serial = Serial::usart1(dp.USART1, uart_pins, 9600.bps(), clocks, &mut rcc.apb2);
    let (mut tx, mut rx) = serial.split();

    loop {
        // Send a single byte.
        let single_byte_to_send = 8;
        tx.write(single_byte_to_send).ok();

        // Send multiple bytes over the U[S]ART pins.
        let data_to_send = [2, 3, 4, 5];
        tx.bwrite_all(&data_to_send).ok();

        // Send a string
        // tx.write_str("Don't panic.");

        // Receive multiple bytes as an array.
        let _data_recieved = rx.read();
    }
}
