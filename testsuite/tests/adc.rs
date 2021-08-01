#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::adc;
use hal::gpio::{Analog, Output, PushPull};
use hal::pac;
use hal::prelude::*;
use hal::{
    gpio::gpioc::{PC0, PC1},
    pac::ADC1_2,
    rcc::{Clocks, AHB},
};

struct State {
    adc: Option<adc::Adc<pac::ADC1>>,
    analog: PC0<Analog>,
    output: PC1<Output<PushPull>>,
    ahb: AHB,
    clocks: Clocks,
    adc1_2: ADC1_2,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert, unwrap};
    use testsuite::GenericPair;

    #[init]
    fn init() -> State {
        let mut dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let pair = GenericPair {
            0: gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr),
            1: gpioc
                .pc1
                .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper),
        };

        State {
            adc: Some(adc::Adc::adc1(
                dp.ADC1,
                &mut dp.ADC1_2,
                &mut rcc.ahb,
                adc::ClockMode::default(),
                clocks,
            )),
            analog: pair.0,
            output: pair.1,
            ahb: rcc.ahb,
            clocks,
            adc1_2: dp.ADC1_2,
        }
    }

    #[test]
    fn measure_pin_high_low(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());
        for _ in 0..10 {
            defmt::unwrap!(state.output.set_high());
            let adc_level: u16 = defmt::unwrap!(adc.read(&mut state.analog).ok());
            defmt::debug!("{}", adc_level);
            defmt::unwrap!(state.output.set_low());
            // Vref is 3V so output should reach the maximum.
            assert!(adc_level >= 3900 && adc_level <= 4100);
            let adc_level: u16 = defmt::unwrap!(adc.read(&mut state.analog).ok());
            defmt::debug!("{}", adc_level);
            // nearly zero (always zero can not be guaranteed)
            assert!(adc_level <= 100);
        }

        // put adc back in place
        state.adc.replace(adc);
    }

    #[test]
    fn free_and_reconfigure(state: &mut State) {
        let adc = defmt::unwrap!(state.adc.take());
        defmt::debug!("Free");
        let adc1 = adc.free();

        defmt::debug!("Reconfigure");
        let new_adc = adc::Adc::adc1(
            adc1,
            &mut state.adc1_2,
            &mut state.ahb,
            adc::ClockMode::default(),
            state.clocks,
        );

        defmt::debug!("Replace");
        // put adc back in place
        state.adc.replace(new_adc);
    }
}
