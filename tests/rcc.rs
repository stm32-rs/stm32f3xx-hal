#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use stm32f3xx_hal::{pac, prelude::*, time::rate::*};

    // Test the defaults with no configuration
    #[test]
    fn hsi_default() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc.cfgr.freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 8u32.MHz());
        defmt::assert!(clock.hclk() == 8u32.MHz());
        defmt::assert!(clock.pclk2() == 8u32.MHz());
        defmt::assert!(clock.pclk1() == 8u32.MHz());
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
            .use_hse(Hertz(8_000_000))
            .sysclk(Hertz(15_000_000))
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 15u32.MHz());
        defmt::assert!(clock.hclk() == 15u32.MHz());
        defmt::assert!(clock.pclk2() == 15u32.MHz());
        defmt::assert!(clock.pclk1() == 15u32.MHz());
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
                let clock = rcc.cfgr.sysclk(Hertz(72_000_000)).freeze(&mut flash.acr);

                defmt::assert!(clock.sysclk() == 72u32.MHz());
                defmt::assert!(clock.hclk() == 72u32.MHz());
                defmt::assert!(clock.pclk2() == 72u32.MHz());
                defmt::assert!(clock.pclk1() == 36u32.MHz());
            } else {
                // Notice the strange part about 67 being reduced to 64?
                //
                // This is the case, because for these devices,
                // with HSI as source, the clock divisor can not be used
                // and the resolution is therefor lower.
                // Because of the implementation the clock is then approximated to
                // the highest possible value (64 Mhz).
                let clock = rcc.cfgr.sysclk(Hertz(67_000_000)).freeze(&mut flash.acr);

                defmt::assert!(clock.sysclk() == 64u32.MHz());
                defmt::assert!(clock.hclk() == 64u32.MHz());
                defmt::assert!(clock.pclk2() == 64u32.MHz());
                defmt::assert!(clock.pclk1() == 32u32.MHz());
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
            .use_hse(Hertz(8_000_000))
            .sysclk(Hertz(32_000_000))
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 32u32.MHz());
        defmt::assert!(clock.hclk() == 32u32.MHz());
        defmt::assert!(clock.pclk2() == 32u32.MHz());
        defmt::assert!(clock.pclk1() == 32u32.MHz());
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
            .use_hse(Hertz(8_000_000))
            .sysclk(Hertz(48_000_000))
            .freeze(&mut flash.acr); // works

        defmt::assert!(clock.sysclk() == 48u32.MHz());
        defmt::assert!(clock.hclk() == 48u32.MHz());
        defmt::assert!(clock.pclk2() == 48u32.MHz());
        defmt::assert!(clock.pclk1() == 24u32.MHz());
        defmt::assert!(clock.usbclk_valid());
    }

    #[test]
    fn rcc_doc_example() {
        let dp = unsafe { pac::Peripherals::steal() };

        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        let clock = rcc
            .cfgr
            .use_hse(Hertz(8_000_000))
            .hclk(Hertz(48_000_000))
            .sysclk(Hertz(48_000_000))
            .pclk1(Hertz(12_000_000))
            .pclk2(Hertz(12_000_000))
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 48u32.MHz());
        defmt::assert!(clock.hclk() == 48u32.MHz());
        defmt::assert!(clock.pclk2() == 12u32.MHz());
        defmt::assert!(clock.pclk1() == 12u32.MHz());
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
            .use_hse(Hertz(8_000_000))
            .pclk1(Hertz(16_000_000))
            .sysclk(Hertz(64_000_000))
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 64u32.MHz());
        defmt::assert!(clock.hclk() == 64u32.MHz());
        defmt::assert!(clock.pclk2() == 64u32.MHz());
        defmt::assert!(clock.pclk1() == 16u32.MHz());
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
            .use_hse(Hertz(8_000_000))
            .sysclk(Hertz(72_000_000))
            .freeze(&mut flash.acr);

        defmt::assert!(clock.sysclk() == 72u32.MHz());
        defmt::assert!(clock.hclk() == 72u32.MHz());
        defmt::assert!(clock.pclk2() == 72u32.MHz());
        defmt::assert!(clock.pclk1() == 36u32.MHz());
        defmt::assert!(clock.usbclk_valid());
    }
}
