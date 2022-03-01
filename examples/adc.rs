#![no_std]
#![no_main]

//! Example usage for ADC on STM32F303

use defmt_rtt as _;
use panic_probe as _;

use defmt;

use cortex_m::asm;
use cortex_m_rt::entry;

use embedded_hal::adc::OneShot;
use stm32f3xx_hal::{adc, pac, prelude::*};

#[entry]
fn main() -> ! {
    // Get peripherals, clocks and freeze them
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    let mut adc_common = adc::CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);

    let mut tuple = (dp.ADC1, dp.ADC2);
    let mut ts = adc::TemperatureSensor::new(&mut adc_common, &mut tuple);

    // set up adc1
    let mut adc = adc::Adc::new(
        tuple.0, // The ADC we are going to control
        adc::config::Config::default(),
        // The following is only needed to make sure the clock signal for the ADC is set up
        // correctly.
        &clocks,
        &adc_common,
    )
    // Convert the ADC into `OneShot` mode.
    //
    // This is not necessary, as the normal ADC does also implement the `OneShot` trait, which can
    // call `read`. But this conversion implies less overhead for the trait implementation. The
    // consequence is, that in this mode nothing can be configured manually.
    .into_oneshot();

    // You can also create an ADC, which is initially disabled and uncalibrated and calibrate
    // it later:
    let mut adc2 = adc::Adc::new_disabled(tuple.1);
    adc2.calibrate(&clocks, &adc_common);
    adc2.set_config(adc::config::Config::default());
    let _ = adc2.into_enabled();

    // Set up pin PA0 as analog pin.
    // This pin is connected to the user button on the stm32f3discovery board.
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut analog_pin = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // Be aware that the values in the table below depend on the input of VREF.
    // To have a stable VREF input, put a condensator and a volt limiting diode in front of it.
    //
    // Also know that integer division and the ADC hardware unit always round down.
    // To make up for those errors, see this forum entry:
    // [https://forum.allaboutcircuits.com/threads/why-adc-1024-is-correct-and-adc-1023-is-just-plain-wrong.80018/]
    defmt::info!("
    The ADC has a 12 bit resolution, i.e. if your reference Value is 3V:
        approx. ADC value | approx. volt value
        ==================+===================
                        0 |        0 mV
                     2048 |     1500 mV
                     4095 |     3000 mV

    If you are using a STM32F3Discovery, PA0 is connected to the User Button.
    Pressing it should connect the user Button to to HIGH and the value should change from 0 to 4095.
    ");

    loop {
        let adc_data: u16 = adc.read(&mut analog_pin).unwrap();
        defmt::trace!("PA0 reads {}", adc_data);
        let adc_data: u16 = adc.read(&mut ts).unwrap();
        defmt::trace!("TemperatureSensor reads {}", adc_data);
        asm::delay(2_000_000);
    }
}

// TODO: Add adc example, which uses Continuous or Discontinous and interrupts.
