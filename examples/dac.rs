#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m::asm;

use cortex_m_rt::entry;
use stm32f3xx_hal::{
    dac::{Dac, DacBits, Trigger},
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
    let reference_voltage = 2.915;
    let mut dac = Dac::new(dp.DAC, dac_pin, DacBits::EightR, reference_voltage);
    dac.enable(&mut rcc.apb1);

    // primitively wait about one second on a default stm32f3discovery
    let wait = 8_000_000;
    asm::delay(wait);

    // directly set the output voltage to 1.5 V (valid values: 0..=reference_voltage)
    dac.set_voltage(1.5);
    asm::delay(3 * wait);

    // equivalently:
    dac.set_value(256 / 2);

    // set the output off for 3 seconds
    dac.set_value(0);
    asm::delay(3 * wait);

    // setup a trigger. From now on, setting a value or voltage will be stored but not converted
    dac.set_trigger(Trigger::SoftwareTrigger);

    // this is never converted:
    dac.set_voltage(1.5);
    asm::delay(wait);

    // toggle the dac value every 2 seconds
    loop {
        dac.set_value(0);
        asm::delay(wait);

        dac.trigger_software_trigger();
        asm::delay(wait);

        dac.set_value(255);
        asm::delay(wait);

        dac.trigger_software_trigger();
        asm::delay(wait);
    }
}
