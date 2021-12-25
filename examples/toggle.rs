//! Uses the StatefulOutputPin embedded_hal trait to toggle the pin
//! On the stm32 discovery board this is the "south" led
//! Target board: STM32F3DISCOVERY

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m_rt::entry;
use hal::pac;
use hal::prelude::*;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let mut led = gpioe.pe13.into_push_pull_output();

    led.set_low().unwrap();

    loop {
        led.toggle().unwrap();
        cortex_m::asm::delay(8_000_000);
        // Toggle by hand.
        // Uses `StatefulOutputPin` instead of `ToggleableOutputPin`.
        // Logically it is the same.
        if led.is_set_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
        cortex_m::asm::delay(8_000_000);
    }
}
