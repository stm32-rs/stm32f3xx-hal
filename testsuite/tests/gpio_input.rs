#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::gpio::{Input, PXx};

struct State {
    input_ground: PXx<Input>,
    input_vdd: PXx<Input>,
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
        let input_ground = gpioc
            .pc3
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr)
            .downgrade()
            .downgrade();
        let input_vdd = gpioc
            .pc2
            .into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr)
            .downgrade()
            .downgrade();

        super::State {
            input_ground,
            input_vdd,
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
