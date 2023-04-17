// TOOD Implement:
// https://github.com/dfrankland/proton-c/commit/0289f1cfa15000d6b1b2a8175420c16ffdff7451
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use hal::gpio::{self, Input};
use hal::pac;
use hal::prelude::*;
use stm32f3xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let gpioc = dp.GPIOC.split(&mut rcc.ahb);
    let gpiod = dp.GPIOD.split(&mut rcc.ahb);

    let mut pin_array: [gpio::PXx<Input>; 4] = [
        gpiob.pb11.into_floating_input().downgrade().downgrade(),
        gpioc.pc4.into_floating_input().downgrade().downgrade(),
        gpiod.pd3.into_floating_input().downgrade().downgrade(),
        gpiod.pd2.into_floating_input().downgrade().downgrade(),
    ];

    hprintln!("Start scanning pin array");
    loop {
        for pin in pin_array.iter_mut() {
            hprintln!("Value is {}", pin.is_high().unwrap());
            asm::delay(1_000_000);
        }
    }
}
