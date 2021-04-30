#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::gpio::{Input, Output, PXx, PushPull};

struct State {
    input_pin: PXx<Input>,
    output_pin: PXx<Output<PushPull>>,
}

#[defmt_test::tests]
mod tests {
    use defmt::{assert, unwrap};
    use stm32f3xx_hal::{pac, prelude::*};

    // Test the defaults with no configuration
    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let input_pin = gpioc
            .pc0
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr)
            .downgrade()
            .downgrade();
        let output_pin = gpioc
            .pc1
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper)
            .downgrade()
            .downgrade();

        super::State {
            input_pin,
            output_pin,
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
