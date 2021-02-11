/*!
 # stm32f3xx-hal

 `stm32f3xx-hal` contains a multi device hardware abstraction on top of the
 peripheral access API for the STMicro [STM32F3][stm] series microcontrollers. The
 selection of the MCU is done by [feature][f] gates

 [f]: #selecting-the-right-chip
 [stm]: https://www.st.com/en/microcontrollers-microprocessors/stm32f3-series.html

 # Selecting the right chip

   This crate requires you to specify your target chip as a feature.

   Please select one of the following

   (Note: `x` denotes any character in [a-z])
   *   stm32f301
   *   stm32f318
   *   stm32f302xb
   *   stm32f302xc
   *   stm32f302xd
   *   stm32f302xe
   *   stm32f302x6
   *   stm32f302x8
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
#![allow(clippy::upper_case_acronyms)]
#![warn(missing_docs)]
#![deny(macro_use_extern_crate)]

#[cfg(all(feature = "direct-call-deprecated", not(feature = "device-selected")))]
compile_error!(
    "The feature you selected is deprecated, because it was split up into sub-devices.

    Example: The STM32F3Discovery board has a STM32F303VCT6 chip.
    You probably used to use `--features stm32f303` but now functionalities for the sub-device were added.
    In this case replace it with `--features stm32f303xc` to make your code build again.

    Please select one of the chip features stated above."
);

// TODO Remove because, as of stm32f3 v0.12, this will be caught by it's build.rs?
#[cfg(all(
    not(feature = "direct-call-deprecated"),
    not(feature = "device-selected")
))]
compile_error!(
    "This crate requires you to specify your target chip as a feature.

    Please select one of the following

    (Note: `x` denotes any character in [a-z])
    *   stm32f301
    *   stm32f318
    *   stm32f302xb
    *   stm32f302xc
    *   stm32f302xd
    *   stm32f302xe
    *   stm32f302x6
    *   stm32f302x8
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

pub use embedded_hal as hal;

pub use nb;
pub use nb::block;

pub use embedded_time as time;

#[cfg(feature = "defmt")]
pub(crate) use defmt::{assert, panic, unreachable, unwrap};
#[cfg(feature = "defmt")]
mod macros {
    /// Wrapper function for `.exepct()`
    ///
    /// Uses [`defmt::unwrap!`] instead, because
    /// it has the same functionality as `expect()`
    #[macro_export]
    macro_rules! expect {
        ($l:expr, $s:tt) => {
            defmt::unwrap!($l, $s)
        };
    }
}

#[cfg(not(feature = "defmt"))]
pub(crate) use core::{assert, panic, unreachable};
#[cfg(not(feature = "defmt"))]
mod macros {
    /// Wrapper macro for `.unwrap()`
    ///
    /// Uses core function, when defmt is not active
    #[macro_export]
    macro_rules! unwrap {
        ($l:expr) => {
            $l.unwrap()
        };
    }

    /// Wrapper macro for `.expect()`
    ///
    /// Uses core function, when defmt is not active
    #[macro_export]
    macro_rules! expect {
        ($l:expr, $s:tt) => {
            $l.expect($s)
        };
    }
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
/// Peripheral access
pub use stm32f3::stm32f301 as pac;

#[cfg(feature = "stm32f302")]
/// Peripheral access
pub use stm32f3::stm32f302 as pac;

#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
/// Peripheral access
pub use stm32f3::stm32f303 as pac;

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
/// Peripheral access
pub use stm32f3::stm32f373 as pac;

#[cfg(feature = "stm32f334")]
/// Peripheral access
pub use stm32f3::stm32f3x4 as pac;

#[cfg(feature = "device-selected")]
#[deprecated(since = "0.5.0", note = "please use `pac` instead")]
/// Peripheral access
pub use crate::pac as stm32;

// Enable use of interrupt macro
#[cfg(feature = "rt")]
pub use crate::pac::interrupt;

cfg_if::cfg_if! {
    if #[cfg(feature = "device-selected")] {
        pub mod delay;
        pub mod flash;
        pub mod gpio;
        pub mod i2c;
        pub mod prelude;
        pub mod pwm;
        pub mod rcc;
        pub mod rtc;
        pub mod serial;
        pub mod spi;
        pub mod syscfg;
        pub mod timer;
    }
}
#[cfg(feature = "stm32f303")]
pub mod adc;
#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
pub mod dma;
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

#[cfg(all(feature = "device-selected", feature = "can"))]
pub mod can;
