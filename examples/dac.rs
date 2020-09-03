#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use hal::{
    dac::{Dac, DacBits, DacId, DacTrait},
    delay::Delay,
    pac,
    prelude::*,
};
use stm32f3xx_hal as hal;

#[entry]
fn main() -> ! {
    // Set up CPU peripherals
    let mut cp = cortex_m::Peripherals::take().unwrap();

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
    let mut _dac_pin = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let mut dac = Dac::new(dp.DAC, DacId::One, DacBits::EightR, 3.3);
    dac.try_enable(&mut rcc.apb1).ok();

    dac.try_set_voltage(1.5).ok();
    // dac.set_value(128);  // or equivalently

    loop {
        delay.delay_ms(1_000_u16);
    }
}
