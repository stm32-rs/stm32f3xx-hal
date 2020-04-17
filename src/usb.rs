//! USB peripheral
//!
//! Requires the `stm32-usbd` feature and one of the `stm32f303x*` features.
//!
//! See <https://github.com/stm32-rs/stm32f3xx-hal/tree/master/examples>
//! for usage examples.

use crate::stm32::{RCC, USB};
use stm32_usbd::UsbPeripheral;

use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::AF14;
pub use stm32_usbd::UsbBus;

pub struct Peripheral {
    pub usb: USB,
    pub pin_dm: PA11<AF14>,
    pub pin_dp: PA12<AF14>,
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr() as *const ();
    const DP_PULL_UP_FEATURE: bool = false;
    const EP_MEMORY: *const () = 0x4000_6000 as _;
    #[cfg(any(feature = "stm32f303xb", feature = "stm32f303xc"))]
    const EP_MEMORY_SIZE: usize = 512;
    #[cfg(any(feature = "stm32f303xd", feature = "stm32f303xe"))]
    const EP_MEMORY_SIZE: usize = 1024;

    fn enable() {
        let rcc = unsafe { &*RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.apb1enr.modify(|_, w| w.usben().enabled());

            // Reset USB peripheral
            rcc.apb1rstr.modify(|_, w| w.usbrst().reset());
            rcc.apb1rstr.modify(|_, w| w.usbrst().clear_bit());
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
        // at least that long.
        cortex_m::asm::delay(72);
    }
}

pub type UsbBusType = UsbBus<Peripheral>;
