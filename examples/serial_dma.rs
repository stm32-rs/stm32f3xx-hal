//! Example of transmitting data over serial interface using DMA.
//! For this to work, the PA9 and PA10 pins must be connected.
//! Target board: STM32F3DISCOVERY

#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m::{asm, singleton};
use cortex_m_rt::entry;
use stm32f3xx_hal::{pac, prelude::*, serial::Serial};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let pins = (
        gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
        gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
    );
    let serial = Serial::usart1(dp.USART1, pins, 9600.bps(), clocks, &mut rcc.apb2);
    let (tx, rx) = serial.split();

    let dma1 = dp.DMA1.split(&mut rcc.ahb);

    // the data we are going to send over serial
    let tx_buf = singleton!(: [u8; 9] = *b"hello DMA").unwrap();
    // the buffer we are going to receive the transmitted data in
    let rx_buf = singleton!(: [u8; 9] = [0; 9]).unwrap();

    // DMA channel selection depends on the peripheral:
    // - USART1: TX = 4, RX = 5
    // - USART2: TX = 6, RX = 7
    // - USART3: TX = 3, RX = 2
    let (tx_channel, rx_channel) = (dma1.ch4, dma1.ch5);

    // start separate DMAs for sending and receiving the data
    let sending = tx.write_all(tx_buf, tx_channel);
    let receiving = rx.read_exact(rx_buf, rx_channel);

    // block until all data was transmitted and received
    let (tx_buf, tx_channel, tx) = sending.wait();
    let (rx_buf, rx_channel, rx) = receiving.wait();

    assert_eq!(tx_buf, rx_buf);

    // After a transfer is finished its parts can be re-used for another one.
    tx_buf.copy_from_slice(b"hi again!");

    let sending = tx.write_all(tx_buf, tx_channel);
    let receiving = rx.read_exact(rx_buf, rx_channel);

    let (tx_buf, ..) = sending.wait();
    let (rx_buf, ..) = receiving.wait();

    assert_eq!(tx_buf, rx_buf);

    loop {
        asm::wfi();
    }
}
