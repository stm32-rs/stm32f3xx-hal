#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use hal::{
    delay::Delay,
    pac,
    prelude::*,
    timer::{Alignment, Channel, Timer, OutputCompare}
};
use stm32f3xx_hal as hal;

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

    let mut timer = Timer::tim2(dp.TIM2, 94.hz(), &clocks);

    timer.set_resolution(1_000);
    
    timer.set_preload(Channel::One, true);

    // An example where we set Asymmetric PWM mode, that serves
    // more generally to highlight features and API.
    
    timer.set_output_compare(Channel::One, OutputCompare::Pwm1);
    timer.set_output_compare(Channel::Two, OutputCompare::Pwm1);
    timer.set_output_compare(Channel::Three, OutputCompare::AsymmetricPwm1);
    timer.set_output_compare(Channel::Four, OutputCompare::AsymmetricPwm1);
    
    timer.set_duty(Channel::One, timer.get_max_duty() / 2);
    timer.set_duty(Channel::Two, timer.get_max_duty() / 5);
    // Set a phase offset for Channel 3/4, using asymmetric mode.
    timer.set_duty(Channel::Three, timer.get_max_duty() * 3 / 5);
    timer.set_duty(Channel::Four, timer.get_max_duty() * 4 / 5);
    
    timer.set_alignment(Alignment::Center1);
    
    timer.enable(Channel::One);
    timer.enable(Channel::Two);
    timer.enable(Channel::Three);
    timer.enable(Channel::Four);

    loop {
        delay.delay_ms(1_000_u16);
    }
}
