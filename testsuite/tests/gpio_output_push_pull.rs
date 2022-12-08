#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use hal::prelude::*;
use stm32f3xx_hal as hal;

use hal::gpio::{Input, Output, PXx, PushPull};

struct State {
    input_pin: PXx<Input>,
    output_pin: PXx<Output<PushPull>>,
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
            input_pin: pair.0.downgrade().downgrade(),
            output_pin: pair.1.downgrade().downgrade(),
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
