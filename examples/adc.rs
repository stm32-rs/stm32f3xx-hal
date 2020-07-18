#![no_std]
#![no_main]

//! Example usage for ADC on STM32F303

extern crate panic_semihosting;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use stm32f3xx_hal::{adc, pac, prelude::*};

#[entry]
/// Main Thread
fn main() -> ! {
    // Get peripherals, clocks and freeze them
    let mut dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    // set up adc1
    #[rustfmt::skip]
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1, // The ADC we are going to control
        // The following is only needed to make sure the clock signal for the ADC is set up
        // correctly.
        &mut dp.ADC1_2,
        &mut rcc.ahb,
        adc::CKMODE::default(),
        clocks,
        // If everything is set up correctly, we'll get `Some(adc1)`. to access it, we need to `unwrap`
        // it. If there was an error, we'll get `None` and this unwrap will `panic!`.
    ).unwrap();

    // Set up pin PA0 as analog pin.
    // This pin is connected to the user button on the stm32f3discovery board.
    let mut gpio_a = dp.GPIOA.split(&mut rcc.ahb);
    let mut adc1_in1_pin = gpio_a.pa0.into_analog(&mut gpio_a.moder, &mut gpio_a.pupdr);

    // Be aware that the values in the table below depend on the input of VREF
    // Also know that and that the ADC hardware unit always rounds down.
    //
    // To have a stable VREF input, put a condensator and a volt limiting diode in front of it.
    //
    // To compensate for the unit and integer math rounding down, use the following formula:
    // ```rust
    // // [setup as above]
    // let mut adc: u3216 = adc1.read(&mut adc1_in1_pin).expect('Error reading adc1.');
    // // add '0.5' to the adc value, compensating the ADC hardware rounding down
    // adc <<= 1;
    // adc += 1;
    // // multiply it by '300 mV', converting to your voltage level
    // let mut volt_mv = adc * 300;
    // // make up for the coming integer division
    // volt_mv += 1 << 12;
    // // undo the first bitshift above and the 12bit ADC size
    // volt_mv >>= 13;
    // println!("Was reading {} mV", volt_mv);
    // ```
    hprintln!("
    The ADC has a 12 bit resolution, i.e. if your reference Value is 3V:
        approx. ADC value | approx. volt value
        ==================+===================
                        0 |       0 mV
                     2048 |     150 mV
                     4095 |     300 mV

    If you are using a STM32F3Discovery, PA0 is connected to the User Button.
    Pressing it should connect the user Button to to HIGH and the value should change from 0 to 4095.
    ").expect("Error using hprintln.");

    loop {
        let adc1_in1_data: u16 = adc1.read(&mut adc1_in1_pin).expect("Error reading adc1.");
        hprintln!("PA0 reads {}", adc1_in1_data).unwrap_or(());
    }
}
