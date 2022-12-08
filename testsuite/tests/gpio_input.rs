#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::{Input, PXx};

struct State {
    input_ground: PXx<Input>,
    input_vdd: PXx<Input>,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{assert, unwrap};
    use stm32f3xx_hal::{pac, prelude::*};
    use testsuite::{GroundPin, VddPin};

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
        let input_ground: GroundPin<Input> = gpioc
            .pc3
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
        let input_vdd: VddPin<Input> = gpioc
            .pc2
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);

        super::State {
            input_ground: input_ground.downgrade().downgrade(),
            input_vdd: input_vdd.downgrade().downgrade(),
        }
    }

    #[test]
    fn ground_is_low(state: &mut super::State) {
        assert!(unwrap!(state.input_ground.is_low()));
    }

    #[test]
    fn vdd_is_high(state: &mut super::State) {
        assert!(unwrap!(state.input_vdd.is_high()));
    }
}
