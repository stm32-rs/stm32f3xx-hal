#![no_std]
#![no_main]

//! VCU - Vehicle Control Unit on the stm32f3discovery (STM32 F303 VCT6)

extern crate panic_halt;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use stm32f3xx_hal::{adc, prelude::*, stm32};

#[entry]
/// Main Thread
fn main() -> ! {
    let mut peripherals = stm32::Peripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut peripherals.FLASH.constrain().acr);
    let mut gpio_a = peripherals.GPIOA.split(&mut rcc.ahb);
    let mut adc1 = adc::Adc::adc1(
        peripherals.ADC1,
        &mut peripherals.ADC1_2,
        &mut rcc.ahb,
        clocks,
    );

    // set up pin pa0, pa1 as analog pin
    let mut adc1_in1_pin = gpio_a.pa0.into_analog(&mut gpio_a.moder, &mut gpio_a.pupdr);

    loop {
        let data: u16 = adc1.read(&mut adc1_in1_pin).unwrap();
        hprintln!("pa0 reads {}", data).expect("error in read");
    }
}
