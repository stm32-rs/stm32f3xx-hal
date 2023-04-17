#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::{Input, OpenDrain, Output, PXx};

struct State {
    input_pin: PXx<Input>,
    output_pin: PXx<Output<OpenDrain>>,
}

#[defmt_test::tests]
mod tests {
    use defmt::{assert, unwrap};
    use stm32f3xx_hal::{pac, prelude::*};
    use testsuite::GenericPair;

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let pair = GenericPair {
            0: gpioc.pc0.into_floating_input(),
            1: gpioc.pc1.into_open_drain_output(),
        };
        let mut input_pin = pair.0;
        input_pin.internal_pull_up(true);
        let output_pin = pair.1;

        super::State {
            input_pin: input_pin.downgrade().downgrade(),
            output_pin: output_pin.downgrade().downgrade(),
        }
    }

    #[test]
    fn set_low_is_low(state: &mut super::State) {
        unwrap!(state.output_pin.set_low());
        assert!(unwrap!(state.output_pin.is_set_low()));
        assert!(unwrap!(state.input_pin.is_low()));
    }

    #[test]
    fn set_high_is_high(state: &mut super::State) {
        unwrap!(state.output_pin.set_high());
        assert!(unwrap!(state.output_pin.is_set_high()));
        assert!(unwrap!(state.input_pin.is_high()));
    }
}
