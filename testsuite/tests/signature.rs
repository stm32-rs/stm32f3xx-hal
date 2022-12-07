#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::signature::{FlashSize, Uid};

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert_eq};

    #[test]
    fn test_uid() {
        let uid = Uid::get();
        defmt::debug!("{}", uid);
        defmt::debug!("{}", defmt::Debug2Format(&uid));
        defmt::debug!("{}", uid.x());
        defmt::debug!("{}", uid.y());
    }

    #[test]
    fn test_flash_size() {
        let flash = FlashSize::get();
        defmt::debug!("Flash Size: {} kB", flash.kilo_bytes());

        assert_eq!(flash.bytes(), usize::from(flash.kilo_bytes()) * 1024);
    }
}
