#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::gpio::{Input, OpenDrain, Output, PXx};

struct State {
    input_pin: PXx<Input>,
    output_pin: PXx<Output<OpenDrain>>,
}

#[defmt_test::tests]
mod tests {
    use cortex_m::asm;
    use defmt::{assert, unwrap};
    use stm32f3xx_hal::{pac, prelude::*};

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let mut input_pin = gpioc
            .pc0
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
        input_pin.internal_pull_up(&mut gpioc.pupdr, true);
        let input_pin = input_pin.downgrade().downgrade();
        let output_pin = gpioc
            .pc1
            .into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper)
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
