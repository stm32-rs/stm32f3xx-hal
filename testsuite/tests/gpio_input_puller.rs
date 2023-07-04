#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::{self, ErasedPin, Input, Pull};
use hal::{pac, prelude::*};

struct State {
    observer: ErasedPin<Input>,
    puller: gpio::PC1<Input>,
    pupdr: gpio::PUPDR<'C'>,
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
        let observer = pair.0.internal_resistor(&mut gpioc.pupdr, Pull::None);
        let puller = pair.1;

        let pupdr = gpioc.pupdr;

        super::State {
            observer: observer.erase(),
            puller,
            pupdr,
        }
    }

    #[test]
    fn pulldown_is_low(state: &mut super::State) {
        state
            .puller
            .set_internal_resistor(&mut state.pupdr, Pull::None);
        cortex_m::asm::delay(10);
        assert!(state.puller.is_low());
        assert!(state.observer.is_low());
    }

    #[test]
    fn set_high_is_high(state: &mut super::State) {
        state
            .puller
            .set_internal_resistor(&mut state.pupdr, Pull::Up);
        cortex_m::asm::delay(10);
        assert!(state.puller.is_high());
        assert!(state.observer.is_high());
    }
}
