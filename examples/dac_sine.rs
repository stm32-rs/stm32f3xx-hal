#![no_std]
#![no_main]

//! Example usage for DAC on STM32F303
// Based on example in stm32hal by David-OConnor

use panic_semihosting as _;

use cortex_m_rt::entry;

use stm32f3xx_hal::{
    dac::{Dac, DacChannel},
    pac,
    prelude::*,
};

const LUT_LEN: usize = 256;

// A lookup table for sin(x), over one period, using values from 0 - 4095; ie the full range of
// the STM32 12-bit onboard DAC. Compared to computation, this is faster, at the expense of memory use.
pub static SIN_X: [u16; crate::LUT_LEN] = [
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
    2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
    3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
    3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
    2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697, 1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311,
    1264, 1218, 1172, 1127, 1082, 1038, 995, 952, 910, 869, 828, 788, 749, 710, 672, 636, 600, 565,
    530, 497, 465, 433, 403, 373, 345, 318, 291, 266, 242, 219, 197, 176, 156, 137, 120, 103, 88,
    74, 61, 50, 39, 30, 22, 15, 10, 6, 2, 1, 0, 1, 2, 6, 10, 15, 22, 30, 39, 50, 61, 74, 88, 103,
    120, 137, 156, 176, 197, 219, 242, 266, 291, 318, 345, 373, 403, 433, 465, 497, 530, 565, 600,
    636, 672, 710, 749, 788, 828, 869, 910, 952, 995, 1038, 1082, 1127, 1172, 1218, 1264, 1311,
    1358, 1405, 1453, 1501, 1550, 1599, 1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997,
];

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
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut ok_led = gpioe
        .pe15
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    // set up dac1, data is twelve bits, alighned right
    let mut dac1 = Dac::new(dp.DAC1, &mut rcc.apb1);
    // enable channel one for single channel mode
    dac1.enable_channel(DacChannel::One);

    let mut led = true;

    loop {
        // lookup values for sine wave and write in buffer
        for value in SIN_X {
            dac1.write_data(value);
            cortex_m::asm::delay(8_000);
        }
        if led {
            ok_led.set_low().unwrap();
            led = false;
        } else {
            ok_led.set_high().unwrap();
            led = true;
        }
    }
}
