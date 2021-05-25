#![no_std]
#![no_main]

// TODO: Get pa9 and pa10 because these also implement spi and uart
use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::adc;
use hal::gpio::{Analog, Output, PushPull};
use hal::pac;
use hal::prelude::*;
use hal::serial::{Rx, Serial, Tx};
use hal::{
    gpio::{
        gpioa::{PA10, PA2, PA3, PA9},
        gpiob::{PB10, PB11},
        gpioc::{PC0, PC1},
    },
    pac::ADC1_2,
    rcc::{Clocks, Rcc, AHB},
};

use core::array::IntoIter;

use hal::serial::Error;

struct State {
    adc: Option<adc::Adc<pac::ADC1>>,
    analog: PC0<Analog>,
    output: PC1<Output<PushPull>>,
    ahb: AHB,
    flash: hal::flash::Parts,
    clocks: Clocks,
    adc1_2: ADC1_2,
}

const TEST_MSG: [u8; 8] = [0xD, 0xE, 0xA, 0xD, 0xB, 0xE, 0xE, 0xF];

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert, assert_eq, unwrap};

    #[init]
    fn init() -> State {
        let mut dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        State {
            adc: Some(adc::Adc::adc1(
                dp.ADC1, // The ADC we are going to control
                // The following is only needed to make sure the clock signal for the ADC is set up
                // correctly.
                &mut dp.ADC1_2,
                &mut rcc.ahb,
                adc::CkMode::default(),
                clocks,
            )),
            analog: gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr),
            output: gpioc
                .pc1
                .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper),
            ahb: rcc.ahb,
            flash,
            clocks,
            adc1_2: dp.ADC1_2,
        }
    }

    #[test]
    fn measure_pin_high_low(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());
        for _ in 1..10 {
            defmt::unwrap!(state.output.set_high());
            let adc_level: u16 = defmt::unwrap!(adc.read(&mut state.analog).ok());
            defmt::info!("{}", adc_level);
            defmt::unwrap!(state.output.set_low());
            // Vref is 3V so output should reach the maximum.
            defmt::assert!(adc_level >= 4070 && adc_level <= 4100);
            let adc_level: u16 = defmt::unwrap!(adc.read(&mut state.analog).ok());
            defmt::info!("{}", adc_level);
            defmt::assert_eq!(adc_level, 0);
        }

        // put adc back in place
        state.adc.replace(adc);
    }

    // #[test]
    // fn free_and_reconfigure(state: &mut State) {
    //     let mut adc = defmt::unwrap!(state.adc.take());
    //     defmt::debug!("Free");
    //     let adc1 = adc.free();

    //     // FIXME: This is not working (stuck on this function)
    //     defmt::debug!("Reconfigure");
    //     let new_adc = adc::Adc::adc1(
    //         adc1,
    //         &mut state.adc1_2,
    //         &mut state.ahb,
    //         adc::CkMode::default(),
    //         state.clocks,
    //     );

    //     defmt::debug!("Replace");
    //     // put adc back in place
    //     state.adc.replace(new_adc);
    // }
}
