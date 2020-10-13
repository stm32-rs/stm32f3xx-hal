#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use stm32f3xx_hal::{
    dac::{Channel, Dac, DacBits},
    delay::Delay,
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up microcontroller peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Set up clocks
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    // Set up gpio pins if required
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    // Set up the DAC
    let dac_pin = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let mut dac = Dac::new(dp.DAC, dac_pin, Channel::One, DacBits::EightR, 3.3);
    dac.enable(&mut rcc.apb1);

    dac.set_voltage(1.5);
    // dac.set_value(128);  // or equivalently

    loop {
        delay.delay_ms(1_000_u16);
    }
}
