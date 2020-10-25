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

    // Initialize to 48Mhz sys clock, and 24Mhz peripheral clocks. Settings don't take
    // effect until we run the `setup` method below.
    let mut clocks = clocks::Clocks::default();

    // Setting HSE bypass, to save power and free up a pin, if using a compatible oscillator.
    clocks.hse_bypass = true;

    // We'll set the system clock frequency to 72 Mhz, while keeping a 48Mhz USB
    // frequency. We do this by increasing the PLL multiplier from it's default of
    // 6×, to 9×, and increasing the amount we divide by for the USB clock.
    clocks.pll_mul = clocks::PllMul::Mul9;
    clocks.usb_pre = clocks::UsbPrescaler::Div1_5;

    // Some further functionality examples:

    // If you wish to use the internal (HSI) oscillator:
    // clocks.input_src = clocks::InputSrc::Hsi;

    // Setting the HCLK (AHB) prescaler:
    // clocks.hclk_prescaler = clocks::HclkPrescaler::Div2;

    // Setting the APB1 peripheral clock prescaler:
    // clocks.apb1_prescaler = clocks::ApbPrescaler::Div1;

    // The `setup` method validates our clock speeds, and if validated, writes
    // to the clock registers.
    if clocks.setup(&mut dp.RCC, &mut dp.FLASH).is_err() {
        hprintln!("Unable to configure clocks due to a speed error.").ok()
    };

    // Display the calculated speeds.
    hprintln!("Speeds: {:#?}", clocks.calc_speeds()).ok();

    // Existing modules expect an `rcc::Clocks` struct to be passed, to use
    // speeds during configuration. We can make one using the `make_rcc_clocks` method.
    // `rcc.rs` clock config we pass to other fns that need the speeds.
    let rcc_clocks = clocks.make_rcc_clocks();

    // Eg, we could use it in the commented-out line below, in a timer, spi etc.
    // let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), rcc_clocks, &mut rcc.apb1);

    loop {}
}
