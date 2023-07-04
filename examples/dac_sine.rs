#![no_std]
#![no_main]

//! Example usage for DAC on STM32F303
// Based on example in stm32hal by David-OConnor

use panic_semihosting as _;

use cortex_m_rt::entry;

use stm32f3xx_hal::{dac::Dac, pac, prelude::*};

#[entry]
/// Main Thread
fn main() -> ! {
    // Get peripherals, clocks and freeze them
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    // Set up pin PA4 as analog pin.
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let _dac1_out1 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // set up led for blinking loop
    let mut ok_led = gpioa
        .pa15
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    // set up dac1, data is twelve bits, aligned right
    let mut dac1 = Dac::new(dp.DAC1, &mut rcc.apb1);

    let mut led = true;

    loop {
        for value in (0..256).chain((0..255).rev()) {
            dac1.write_data((4095 / 255) * value);
            cortex_m::asm::delay(8_000);
        }
        if led {
            ok_led.set_low();
            led = false;
        } else {
            ok_led.set_high();
            led = true;
        }
    }
}
