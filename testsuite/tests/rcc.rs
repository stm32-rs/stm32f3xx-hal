#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use stm32f3xx_hal::{pac, prelude::*};

    // Test the defaults with no configuration
    #[test]
    fn hsi_default() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc.cfgr.freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 8.MHz());
        defmt::assert!(clock.hclk() == 8.MHz());
        defmt::assert!(clock.pclk2() == 8.MHz());
        defmt::assert!(clock.pclk1() == 8.MHz());
        defmt::assert!(!clock.usbclk_valid());
    }

    // Test configurations where a finer division
    // of the crystal clock is needed
    // In this case div = 8 and mul = 15
    #[test]
    fn hse_sysclk_15mhz() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(15.MHz())
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 15.MHz());
        defmt::assert!(clock.hclk() == 15.MHz());
        defmt::assert!(clock.pclk2() == 15.MHz());
        defmt::assert!(clock.pclk1() == 15.MHz());
        defmt::assert!(!clock.usbclk_valid());
    }

    // Test highest possible frequency reachable via HSI
    #[test]
    fn hsi_highest_sysclk() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        cfg_if::cfg_if! {
            if #[cfg(any(
                    feature = "stm32f302xd",
                    feature = "stm32f302xe",
                    feature = "stm32f303xd",
                    feature = "stm32f303xe",
                    feature = "stm32f398",
            ))] {
                let clock = rcc.cfgr.sysclk(72.MHz()).freeze(&mut flash.acr);

                defmt::assert!(clock.sysclk() == 72.MHz());
                defmt::assert!(clock.hclk() == 72.MHz());
                defmt::assert!(clock.pclk2() == 72.MHz());
                defmt::assert!(clock.pclk1() == 36.MHz());
            } else {
                // Notice the strange part about 67 being reduced to 64?
                //
                // This is the case, because for these devices,
                // with HSI as source, the clock divisor can not be used
                // and the resolution is therefor lower.
                // Because of the implementation the clock is then approximated to
                // the highest possible value (64 Mhz).
                let clock = rcc.cfgr.sysclk(67.MHz()).freeze(&mut flash.acr);

                defmt::assert!(clock.sysclk() == 64.MHz());
                defmt::assert!(clock.hclk() == 64.MHz());
                defmt::assert!(clock.pclk2() == 64.MHz());
                defmt::assert!(clock.pclk1() == 32.MHz());
            }
        }

        // With HSI USB clock should never be valid
        defmt::assert!(!clock.usbclk_valid());
    }

    #[test]
    fn hse_sysclk_32mhz() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(32.MHz())
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 32.MHz());
        defmt::assert!(clock.hclk() == 32.MHz());
        defmt::assert!(clock.pclk2() == 32.MHz());
        defmt::assert!(clock.pclk1() == 32.MHz());
        defmt::assert!(!clock.usbclk_valid());
    }

    // One of the possible frequencies, which is
    // results into a valid USB frequency
    #[test]
    fn hse_sysclk_48mhz() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .freeze(&mut flash.acr); // works

        defmt::assert!(clock.sysclk() == 48.MHz());
        defmt::assert!(clock.hclk() == 48.MHz());
        defmt::assert!(clock.pclk2() == 48.MHz());
        defmt::assert!(clock.pclk1() == 24.MHz());
        defmt::assert!(clock.usbclk_valid());
    }

    #[test]
    fn rcc_doc_example() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .hclk(48.MHz())
            .sysclk(48.MHz())
            .pclk1(12.MHz())
            .pclk2(12.MHz())
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 48.MHz());
        defmt::assert!(clock.hclk() == 48.MHz());
        defmt::assert!(clock.pclk2() == 12.MHz());
        defmt::assert!(clock.pclk1() == 12.MHz());
        defmt::assert!(clock.usbclk_valid());
    }

    // Another multiple of 8.MHz() external crystal
    #[test]
    fn hse_sysclk_64mhz() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .pclk1(16.MHz())
            .sysclk(64.MHz())
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 64.MHz());
        defmt::assert!(clock.hclk() == 64.MHz());
        defmt::assert!(clock.pclk2() == 64.MHz());
        defmt::assert!(clock.pclk1() == 16.MHz());
        defmt::assert!(!clock.usbclk_valid());
    }

    // The highest possible frequency currently allowed
    // Also a valid USB frequency
    #[test]
    fn hse_sysclk_72mhz() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 72.MHz());
        defmt::assert!(clock.hclk() == 72.MHz());
        defmt::assert!(clock.pclk2() == 72.MHz());
        defmt::assert!(clock.pclk1() == 36.MHz());
        defmt::assert!(clock.usbclk_valid());
    }
}
