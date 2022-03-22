#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::{EPin, Input, OpenDrain, Output};

struct State {
    input_pin: EPin<Input>,
    output_pin: EPin<Output<OpenDrain>>,
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
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let pair = GenericPair {
            0: gpioc
                .pc0
                .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr),
            1: gpioc
                .pc1
                .into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper),
        };
        let input_pin = pair.0.internal_pull_up(&mut gpioc.pupdr, true);
        let output_pin = pair.1;

        super::State {
            input_pin: input_pin.erase(),
            output_pin: output_pin.erase(),
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
