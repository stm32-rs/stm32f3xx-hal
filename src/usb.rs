//! USB peripheral
//!
//! Requires the `stm32-usbd` feature and one of the `stm32f303x*` features.
//!
//! See [examples/usb_serial.rs] for a usage example.
//!
//! [examples/usb_serial.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.1/examples/usb_serial.rs

use crate::pac::{RCC, USB};
use stm32_usbd::UsbPeripheral;

use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::{PushPull, AF14};
pub use stm32_usbd::UsbBus;

/// USB Peripheral
///
/// Constructs the peripheral, which
/// than gets passed to the [`UsbBus`].
pub struct Peripheral {
    /// USB Register Block
    pub usb: USB,
    /// Data Negativ Pin
    pub pin_dm: PA11<AF14<PushPull>>,
    /// Data Positiv Pin
    pub pin_dp: PA12<AF14<PushPull>>,
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
    #[cfg(any(feature = "stm32f303xb", feature = "stm32f303xc"))]
    const EP_MEMORY_ACCESS_2X16: bool = false;
    #[cfg(any(feature = "stm32f303xd", feature = "stm32f303xe"))]
    const EP_MEMORY_ACCESS_2X16: bool = true;

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
        // 72 Mhz is the highest frequency, so this ensures a minimum of 1µs wait time.
        cortex_m::asm::delay(72);
    }
}

/// Type of the UsbBus
///
/// As this MCU family has only USB peripheral,
/// this is the only possible concrete type construction.
pub type UsbBusType = UsbBus<Peripheral>;
