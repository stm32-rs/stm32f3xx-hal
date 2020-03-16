#![no_std]
#![allow(non_camel_case_types)]

#[cfg(all(not(feature = "device-selected"), not(feature = "needs-subvariant")))]
compile_error!(
    "This crate requires you to specify your target device as a feature.

    See README -> Selecting the right feature gate."
);

#[cfg(all(not(feature = "device-selected"), feature = "needs-subvariant"))]
compile_error!(
    "This crate requires you to specify the subvariant of your chip.

    e.g. The STM32F3Discovery board has a STM32F303VCT6 chip.
    If you only specified `--features stm32f303` you will get this error.
    Expand it to `--features stm32f303xc` to get all the functionality of your board.

    See README -> Selecting the right feature gate for more."
);

pub use embedded_hal as hal;

pub use nb;
pub use nb::block;

#[cfg(feature = "stm32f301")]
pub use stm32f3::stm32f301 as stm32;

#[cfg(feature = "stm32f302")]
pub use stm32f3::stm32f302 as stm32;

#[cfg(feature = "stm32f303")]
pub use stm32f3::stm32f303 as stm32;

#[cfg(feature = "stm32f373")]
pub use stm32f3::stm32f373 as stm32;

#[cfg(feature = "stm32f334")]
pub use stm32f3::stm32f3x4 as stm32;

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f378",
    feature = "stm32f398"
))]
pub use stm32f3::stm32f3x8 as stm32;

// Enable use of interrupt macro
#[cfg(feature = "rt")]
pub use crate::stm32::interrupt;

#[cfg(feature = "device-selected")]
pub mod delay;
#[cfg(feature = "device-selected")]
pub mod flash;
#[cfg(feature = "device-selected")]
pub mod gpio;
#[cfg(feature = "device-selected")]
pub mod i2c;
#[cfg(feature = "device-selected")]
pub mod prelude;
#[cfg(feature = "device-selected")]
pub mod pwm;
#[cfg(feature = "device-selected")]
pub mod rcc;
#[cfg(feature = "device-selected")]
pub mod serial;
#[cfg(feature = "device-selected")]
pub mod spi;
#[cfg(feature = "device-selected")]
pub mod time;
#[cfg(feature = "device-selected")]
pub mod timer;
#[cfg(all(
    feature = "stm32-usbd",
    any(
        feature = "stm32f303xb",
        feature = "stm32f303xc",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
    )
))]
pub mod usb;
