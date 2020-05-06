/*!
 # Selecting the right chip

   This crate requires you to specify your target chip as a feature.

   Please select one of the following

   (Note: `x` denotes any character in [a-z])
   *   stm32f301xb
   *   stm32f301xc
   *   stm32f301xd
   *   stm32f301xe
   *   stm32f318
   *   stm32f302xb
   *   stm32f302xc
   *   stm32f302xd
   *   stm32f302xe
   *   stm32f303xb
   *   stm32f303xc
   *   stm32f303xd
   *   stm32f303xe
   *   stm32f303x6
   *   stm32f303x8
   *   stm32f373
   *   stm32f378
   *   stm32f334
   *   stm32f328
   *   stm32f358
   *   stm32f398

   Example: The STM32F3Discovery board has a STM32F303VCT6 chip.
   So you want to expand your call to `cargo` with `--features stm32f303xc`.

   For more information, see the [README](https://github.com/stm32-rs/stm32f3xx-hal/blob/master/README.md#selecting-the-right-chip)
*/
#![no_std]
#![allow(non_camel_case_types)]

#[cfg(all(not(feature = "device-selected"), not(feature = "needs-subvariant")))]
compile_error!(
    "This crate requires you to specify your target chip as a feature.

    Please select one of the following

    (Note: `x` denotes any character in [a-z])
    *   stm32f301xb
    *   stm32f301xc
    *   stm32f301xd
    *   stm32f301xe
    *   stm32f318
    *   stm32f302xb
    *   stm32f302xc
    *   stm32f302xd
    *   stm32f302xe
    *   stm32f303xb
    *   stm32f303xc
    *   stm32f303xd
    *   stm32f303xe
    *   stm32f303x6
    *   stm32f303x8
    *   stm32f373
    *   stm32f378
    *   stm32f334
    *   stm32f328
    *   stm32f358
    *   stm32f398

    Example: The STM32F3Discovery board has a STM32F303VCT6 chip.
    So you want to expand your call to `cargo` with `--features stm32f303xc`.

    For more information, see README -> Selecting the right chip.
    "
);

#[cfg(all(not(feature = "device-selected"), feature = "direct-call-deprecated",))]
compile_error!(
    "The feature you selected is deprecated, because it was split up into sub-devices.

    Example: The STM32F3Discovery board has a STM32F303VCT6 chip.
    You probably used to use `--features stm32f303` but now functionalities for the sub-device were added.
    In this case replace it with `--features stm32f303xc` to make your code build again.

    Please select one of the chip features stated above."
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

#[cfg(feature = "device-selected")]
pub mod watchdog;
