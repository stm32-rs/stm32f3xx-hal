#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use hal::prelude::*;
use stm32f3xx_hal as hal;

use hal::gpio::{EPin, Input, Output, PushPull};

struct State {
    input_pin: EPin<Input>,
    output_pin: EPin<Output<PushPull>>,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{assert, unwrap};
    use testsuite::GenericPair;

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(hal::pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let pair = GenericPair {
            0: gpioc
                .pc0
                .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr),
            1: gpioc
                .pc1
                .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper),
        };

        super::State {
            input_pin: pair.0.erase(),
            output_pin: pair.1.erase(),
        }
    }

    #[test]
    fn set_low_is_low(state: &mut super::State) {
        state.output_pin.set_low();
        assert!(state.output_pin.is_set_low());
        assert!(state.input_pin.is_low());
    }

    #[test]
    fn set_high_is_high(state: &mut super::State) {
        state.output_pin.set_high();
        assert!(state.output_pin.is_set_high());
        assert!(state.input_pin.is_high());
    }
}
