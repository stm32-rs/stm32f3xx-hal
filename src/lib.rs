/*!
 # `stm32f3xx-hal`

 `stm32f3xx-hal` contains a multi device hardware abstraction on top of the
 peripheral access API for the STMicro [STM32F3][stm] series microcontrollers.

 ## Philosophie

 HAL (meaning **H**ardware **A**bstraction **L**ayer) is a generic term used in many contexts,
 but in the specific context of this crate, it is meant to abstract away the control exposed
 by the devices "[peripheral access crate](`crate::pac`)" to simplify initialization routines,
 with a robust interface avoiding miss-configurations while still not abstracting away too much.

 Also, this crate's goal is to integrate well with the rest of the rust embedded ecosystem,
 for example by implementing the [`embedded_hal`] traits or using crates like [`embedded_time`],
 or [`rtcc`].

 [stm]: https://www.st.com/en/microcontrollers-microprocessors/stm32f3-series.html

 ## Basic Usage

 ```rust
 #![no_std]
 #![no_main]

 use cortex_m::asm;
 use cortex_m_rt::entry;
 use panic_halt as _;
 use stm32f3xx_hal::{self as hal, pac, prelude::*};

 #[entry]
 fn main() -> ! {
       let dp = pac::Peripherals::take().unwrap();

       let mut rcc = dp.RCC.constrain();
       let gpioe = dp.GPIOE.split(&mut rcc.ahb);

       let mut led = gpioe.pe13.into_push_pull_output();

       loop {
             led.toggle().unwrap();
             asm::delay(8_000_000);
       }
 }
 ```

 ## Cargo features

 ### Target chip selection

 This crate requires you to specify your target chip as a feature.

 Please select one of the following (`x` denotes any character in [a-z]):

 * stm32f301x6, stm32f301x8, stm32f318x8
 * stm32f302x6, stm32f302x8, stm32f302xb, stm32f302xc, stm32f302xd, stm32f302xe
 * stm32f303x6, stm32f303x8, stm32f303xb, stm32f303xc, stm32f303xd, stm32f303xe
 * stm32f328x8
 * stm32f358xc
 * stm32f398xe
 * stm32f373x8, stm32f373xb, stm32f373xc, stm32f378xc
 * stm32f334x4, stm32f334x6, stm32f334x8

 Example: The STM32F3Discovery board has a STM32F303VCT6 chip.
 So you need to specify `stm32f303xc` in your `Cargo.toml` (note that VC → xc).

 For more information, see the [README][].

 [README]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.1/README.md#selecting-the-right-chip

 ### `ld`

 When this feature is enabled the `memory.x` linker script for target chip is automatically
 provided by this crate. See [`cortex-m-rt` document][memoryx] for more info.

 [memoryx]: https://docs.rs/cortex-m-rt/0.6.13/cortex_m_rt/#memoryx

 ### `rt`

 This feature enables [`stm32f3`][]'s `rt` feature. See [`cortex-m-rt` document][device] for more info.

 [`stm32f3`]: https://crates.io/crates/stm32f3
 [device]: https://docs.rs/cortex-m-rt/0.6.13/cortex_m_rt/#device

 ### `can`

 Enable CAN peripherals on supported targets.
 The can implementation of the interface is backed by  [`bxcan`](https://crates.io/crates/bxcan)

 ### `usb`

 Enable USB peripherals on supported targets via the [`stm32-usbd`](https://crates.io/crates/stm32-usbd) crate.

 ### `rtc`

 Enables RTC support, build upon [`rtcc`](https://crates.io/crates/rtcc) crate.

 ### `enumset`

 Enable functions, which leverage [`enumset`](https://crates.io/crates/enumset).
 This is especially usefull to get all set status events at once,
 see for example [`serial::Serial::triggered_events()`]

 ### `defmt`

 Currently these are only used for panicking calls, like
 `assert!` `panic!` or `unwrap()`. These are enabled using the [defmt][]
 [filter][].
 For now [defmt][] is mostly intended for internal development and testing
 to further reduce panicking calls in this crate.
 The support of this feature is _subject to change_ as the development
 of [defmt][] is advancing.

 To use this feature follow the [Application Setup][] of the `defmt-book`.

 [Application Setup]: https://defmt.ferrous-systems.com/setup-app.html
 [defmt]: https://github.com/knurling-rs/defmt
 [filter]: https://defmt.ferrous-systems.com/filtering.html
 [`serial::Serial::triggered_events`]: `crate::serial::Serial::triggered_events`
*/
#![no_std]
#![allow(clippy::upper_case_acronyms)]
#![warn(missing_docs)]
#![deny(macro_use_extern_crate)]
#![cfg_attr(nightly, deny(rustdoc::broken_intra_doc_links))]
#![cfg_attr(docsrs, feature(doc_cfg))]

use cfg_if::cfg_if;

pub use embedded_hal as hal;

pub use nb;
pub use nb::block;

pub use embedded_time as time;

/// Peripheral access
#[cfg(feature = "svd-f301")]
pub use stm32f3::stm32f301 as pac;

/// Peripheral access
#[cfg(feature = "svd-f302")]
pub use stm32f3::stm32f302 as pac;

/// Peripheral access
#[cfg(feature = "svd-f303")]
pub use stm32f3::stm32f303 as pac;

/// Peripheral access
#[cfg(feature = "svd-f373")]
pub use stm32f3::stm32f373 as pac;

/// Peripheral access
#[cfg(feature = "svd-f3x4")]
pub use stm32f3::stm32f3x4 as pac;

/// Enable use of interrupt macro.
#[cfg(feature = "rt")]
#[cfg_attr(docsrs, doc(cfg(feature = "rt")))]
pub use crate::pac::interrupt;

// TODO: Not yet supported for stm32f373 as this does not share a concept of a "shared ADC"
#[cfg(not(feature = "svd-f373"))]
pub mod adc;
#[cfg(all(feature = "can", not(feature = "svd-f301")))]
#[cfg_attr(docsrs, doc(cfg(feature = "can")))]
pub mod can;
pub mod dac;
pub mod delay;
pub mod dma;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod interrupts;
pub mod prelude;
pub mod pwm;
pub mod rcc;
#[cfg(feature = "rtc")]
#[cfg_attr(docsrs, doc(cfg(feature = "rtc")))]
pub mod rtc;
pub mod serial;
pub mod signature;
pub mod spi;
pub mod syscfg;
pub mod timer;
#[cfg(all(
    feature = "usb",
    any(
        feature = "stm32f303xb",
        feature = "stm32f303xc",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
    ),
))]
#[cfg_attr(docsrs, doc(cfg(feature = "usb")))]
pub mod usb;
pub mod watchdog;

pub(crate) mod reg;

mod private {
    /// Private sealed trait to seal all GPIO implementations
    /// which do implement peripheral functionalities.
    pub trait Sealed {}
}

pub(crate) use private::Sealed;

cfg_if! {
    if #[cfg(feature = "defmt")] {
        #[allow(unused_imports)]
        pub(crate) use defmt::{assert, panic, unreachable, unwrap};
        #[allow(unused_imports)]
        pub(crate) use macros::expect;

        mod macros {
            /// Wrapper function for `.expect()`
            ///
            /// Uses [`defmt::unwrap!`] instead, because
            /// it has the same functionality as `expect()`
            macro_rules! expect_wrapper {
                ($l:expr, $s:tt) => {
                    defmt::unwrap!($l, $s)
                };
            }
            pub(crate) use expect_wrapper as expect;
        }
    } else {
        #[allow(unused_imports)]
        pub(crate) use core::{assert, panic, unreachable};
        #[allow(unused_imports)]
        pub(crate) use macros::{unwrap, expect};

        mod macros {
            /// Wrapper macro for `.unwrap()`
            ///
            /// Uses core function, when defmt feature is not active
            macro_rules! unwrap_wrapper {
                ($l:expr) => {
                    $l.unwrap()
                };
            }
            pub(crate) use unwrap_wrapper as unwrap;

            /// Wrapper macro for `.expect()`
            ///
            /// Uses core function, when defmt feature is not active
            macro_rules! expect_wrapper {
                ($l:expr, $s:tt) => {
                    $l.expect($s)
                };
            }
            pub(crate) use expect_wrapper as expect;
        }
    }
}

/// Toggle something on or off.
///
/// Convenience enum and wrapper around a bool, which more explicit about the intention to enable
/// or disable something, in comparison to `true` or `false`.
// TODO: Maybe move to some mod like "util"?
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[deprecated(since = "0.10.0", note = "Use Switch instead")]
#[allow(deprecated)]
pub enum Toggle {
    /// Toggle something on / enable a thing.
    On,
    /// Toggle something off / disable a thing.
    Off,
}

/// Switch something on or off.
///
/// Convenience enum and wrapper around a bool, which more explicit about the intention to enable
/// or disable something, in comparison to `true` or `false`.
// TODO: Maybe move to some mod like "util"?
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Switch {
    /// Switch something on / enable a thing.
    On,
    /// Switch something off / disable a thing.
    Off,
}

#[allow(deprecated)]
impl From<Toggle> for Switch {
    fn from(toggle: Toggle) -> Self {
        match toggle {
            Toggle::On => Switch::On,
            Toggle::Off => Switch::Off,
        }
    }
}

impl From<Switch> for bool {
    fn from(switch: Switch) -> Self {
        matches!(switch, Switch::On)
    }
}

impl From<bool> for Switch {
    fn from(b: bool) -> Self {
        match b {
            true => Switch::On,
            false => Switch::Off,
        }
    }
}

/// A generic Error type for failable integer to enum conversions used
/// in multiple occasions inside this crate.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TryFromIntError;
