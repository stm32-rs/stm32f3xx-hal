#![no_std]
#![no_main]

use hal::rcc::{Clocks, APB1};
use testsuite as _;

use stm32f3xx_hal as hal;

use hal::prelude::*;

use hal::gpio::{PushPull, AF6};
use hal::gpio::{PC10, PC11, PC12};
use hal::hal::spi::MODE_0;
use hal::pac;
use hal::pac::SPI3;
use hal::spi::{config::Config, Mode, Phase, Polarity, Spi};
use hal::time::rate::{self, Extensions};

const TEST_MSG: [u8; 8] = [0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0xaa, 0xbb];

// TODO: PC10 (SckPin) is not really needed but is forced to be set,
// because of the current implement types.
type Spi3<T> = Spi<SPI3, (PC10<T>, PC11<T>, PC12<T>), u8>;

struct State {
    spi: Option<Spi3<AF6<PushPull>>>,
    clocks: Clocks,
    apb1: APB1,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert_eq, unwrap};
    use hal::gpio::GpioExt;
    use testsuite::SpiPair;

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        let gpioc = dp.GPIOC.split(&mut rcc.ahb);

        let spi_pins = SpiPair {
            0: gpioc.pc10.into_af_push_pull(),
            1: gpioc.pc11.into_af_push_pull(),
            2: gpioc.pc12.into_af_push_pull(),
        };

        let spi = Spi::new(
            dp.SPI3,
            (spi_pins.0, spi_pins.1, spi_pins.2),
            10.MHz(),
            clocks,
            &mut rcc.apb1,
        );

        super::State {
            spi: Some(spi),
            clocks,
            apb1: rcc.apb1,
        }
    }

    #[test]
    fn test_transfer(state: &mut super::State) {
        let rates: [rate::Generic<u32>; 4] = [
            100.Hz().into(),
            100.kHz().into(),
            10.MHz().into(),
            32.MHz().into(),
        ];
        for freq in rates {
            let (spi, pins) = unwrap!(state.spi.take()).free();

            let mut spi = Spi::new(spi, pins, freq, state.clocks, &mut state.apb1);
            let mut transfer_buffer = TEST_MSG;

            unwrap!(spi.transfer(&mut transfer_buffer));
            assert_eq!(transfer_buffer, TEST_MSG);

            state.spi = Some(spi);
        }
    }

    #[test]
    fn config_builder() {
        let config = Config::default();

        assert!(config.frequency == 1.MHz().into());
        assert!(config.mode == MODE_0);

        let config = config.frequency(1.MHz()).mode(Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        });

        assert!(config == Config::default());
    }
}
