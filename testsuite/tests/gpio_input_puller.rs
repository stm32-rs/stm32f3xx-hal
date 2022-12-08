#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::gpioc;
use hal::gpio::Resistor;
use hal::gpio::{Input, PXx};
use hal::{pac, prelude::*};

struct State {
    observer: PXx<Input>,
    puller: gpioc::PC1<Input>,
    pupdr: gpioc::PUPDR,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{assert, unwrap};
    use testsuite::GenericPair;

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let pair = GenericPair {
            0: gpioc
                .pc0
                .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr),
            1: gpioc
                .pc1
                .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr),
        };
        let mut observer = pair.0;
        let puller = pair.1;

        observer.set_internal_resistor(&mut gpioc.pupdr, Resistor::Floating);
        let pupdr = gpioc.pupdr;

        super::State {
            observer: observer.downgrade().downgrade(),
            puller,
            pupdr,
        }
    }

    #[test]
    fn pulldown_is_low(state: &mut super::State) {
        state
            .puller
            .set_internal_resistor(&mut state.pupdr, Resistor::PullDown);
        cortex_m::asm::delay(10);
        assert!(unwrap!(state.puller.is_low()));
        assert!(unwrap!(state.observer.is_low()));
    }

    #[test]
    fn set_high_is_high(state: &mut super::State) {
        state
            .puller
            .set_internal_resistor(&mut state.pupdr, Resistor::PullUp);
        cortex_m::asm::delay(10);
        assert!(unwrap!(state.puller.is_high()));
        assert!(unwrap!(state.observer.is_high()));
    }
}
