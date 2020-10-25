//! Example of using a number of timer channels in PWM mode.
//! Target board: STM32F3DISCOVERY
#![no_std]
#![no_main]

use panic_semihosting as _;

use stm32f3xx_hal::{clocks, pac};

use cortex_m_rt::entry;

use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    // Get our peripherals
    let mut dp = pac::Peripherals::take().unwrap();

    // Configure our clocks

    // Start using a default
    let mut clocks_ = clocks::Clocks::default();

    // The lines below show examples of setting different configs.

    // Setting HSE bypass, to save power and free up a pin, if using a compatible oscillator.
    clocks_.hse_bypass = true;

    // Set a PLL multiplier of 6, resulting in a system clock speed of 48 Mhz.
    clocks_.pll_mul = clocks::PllMul::Mul6;

    // If you wish to use the internal (HSI) oscillator:
    clocks_.input_src = clocks::InputSrc::Hsi;

    // Setting the HCLK (AHB) prescaler:
    clocks_.hclk_prescaler = clocks::HclkPrescaler::Div2;

    // Setting the APB1 peripheral clock prescaler:
    clocks_.apb1_prescaler = clocks::ApbPrescaler::Div1;

    // The `setup` method validates our clock speeds, and if validated, writes
    // to the clock registesr.
    if clocks_.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        hprintln!("Clock speeds out of range").ok();
    }

    // Display the calculated speeds.
    hprintln!("Speeds: {:#?}", clocks_.calc_speeds()).ok();

    loop {}
}
