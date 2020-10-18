#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f3xx_hal::{
    dac::{Dac, DacBits},
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    // Set up microcontroller peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Set up the DAC output pin.
    // The output pin you choose selects the respective DAC channel:
    //  - PA4: channel 1
    //  - PA5: channel 2
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let dac_pin = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // Set up the DAC
    let mut dac = Dac::new(dp.DAC, dac_pin, DacBits::EightR, 3.3);
    dac.enable(&mut rcc.apb1);

    dac.set_voltage(1.5);
    // dac.set_value(128);  // or equivalently

    loop {
        continue;
    }
}
