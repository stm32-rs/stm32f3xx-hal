/*!
# Testsuite

This testsuite module defines the test setup via the pin type setup.

Because of the ease of use, the tests are written for the STM32F3Discovery board.

To easier run any test on other hardware, some type abstractions are provided.
In most cases, many pins are pairs of GPIOs connected to each other,
so that one one pin can confirm the correct behavior of the other pin and the underlying
peripheral.
*/
#![no_std]
#![cfg_attr(test, no_main)]

use defmt_rtt as _;
use panic_probe as _;

use hal::gpio::{gpioa::*, gpiob::*, gpioc::*};
use stm32f3xx_hal as hal;

/// Pin connected to Vdd, which should be 3.3 Volts
pub type VddPin<T> = PC2<T>;
/// Pin connected to Ground / GND
pub type GroundPin<T> = PC3<T>;

/// Pin Pair directly connected to each other.
///
/// Used for basic GPIO tests
pub struct GenericPair<T1, T2>(pub PC0<T1>, pub PC1<T2>);

/// Pin Pair directly connected to each other.
///
/// This pin pair does support UART1 and SPI.
/// So this pin can be repurposed fore many tests.
pub struct SerialPair<T1, T2>(pub PA9<T1>, pub PA10<T2>);

/// Pin Pair directly connected to each other.
///
/// This is used for UART, where UART2(TX) is connected to UART3(RX)
pub struct CrossSerialPair1<T1, T2>(pub PA2<T1>, pub PB11<T2>);

/// Pin Pair directly connected to each other.
///
/// This is used for UART, where UART3(TX) is connected to UART2(RX)
pub struct CrossSerialPair2<T1, T2>(pub PB10<T1>, pub PA3<T2>);

/// Pin Pair directly connected to each other.
///
/// This is used for SPI, where SPI3(TX) is connected to SPI3(RX)
pub struct SpiPair<T>(pub PC10<T>, pub PC11<T>, pub PC12<T>);

#[export_name = "main"]
unsafe extern "C" fn __dummy_entry() -> ! {
    defmt_test::export::exit()
}
