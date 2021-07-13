//! # USB peripheral.
//!
//! Mostly builds upon the [`stm32_usbd`] crate.
//!
//! ## Examples
//!
//! See [examples/usb_serial.rs] for a usage example.
//!
//! [examples/usb_serial.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.7.0/examples/usb_serial.rs

use crate::pac::{RCC, USB};
use stm32_usbd::UsbPeripheral;

use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::{PushPull, AF14};
pub use stm32_usbd::UsbBus;

/// Trait implemented by all pins that can be the "D-" pin for the USB peripheral
pub trait DMPin: crate::private::Sealed {}

/// Trait implemented by all pins that can be the "D+" pin for the USB peripheral
pub trait DPPin: crate::private::Sealed {}

#[cfg(any(feature = "stm32f303xb", feature = "stm32f303xc"))]
impl DMPin for PA11<AF14<PushPull>> {}

#[cfg(any(feature = "stm32f303xb", feature = "stm32f303xc"))]
impl DPPin for PA12<AF14<PushPull>> {}

#[cfg(any(feature = "stm32f303xd", feature = "stm32f303xe"))]
impl<Mode> DMPin for PA11<Mode> {}

#[cfg(any(feature = "stm32f303xd", feature = "stm32f303xe"))]
impl<Mode> DPPin for PA12<Mode> {}

/// USB Peripheral
///
/// Constructs the peripheral, which
/// than gets passed to the [`UsbBus`].
pub struct Peripheral<DM: DMPin, DP: DPPin> {
    /// USB Register Block
    pub usb: USB,
    /// Data Negativ Pin
    pub pin_dm: DM,
    /// Data Positiv Pin
    pub pin_dp: DP,
}

unsafe impl<DM: DMPin, DP: DPPin> Sync for Peripheral<DM, DP> {}

unsafe impl<DM: DMPin + Send, DP: DPPin + Send> UsbPeripheral for Peripheral<DM, DP> {
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
pub type UsbBusType<DP = PA11<AF14<PushPull>>, DM = PA12<AF14<PushPull>>> =
    UsbBus<Peripheral<DM, DP>>;
