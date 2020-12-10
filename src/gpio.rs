//! General Purpose Input / Output
//!
//! To use the GPIO pins, you first need to configure the GPIO bank (GPIOA, GPIOB, ...) that you
//! are interested in. This is done using the [`GpioExt::split`] function.
//!
//! ```
//! let dp = pac::Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//!
//! let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//! ```
//!
//! The resulting [Parts](gpioa::Parts) struct contains one field for each
//! pin, as well as some shared registers.
//!
//! To use a pin, first use the relevant `into_...` method of the [pin](gpioa::PA0).
//!
//! ```rust
//! let pa0 = gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
//! ```
//!
//! And finally, you can use the functions from the [InputPin] or [OutputPin] traits in
//! `embedded_hal`
//!
//! For a complete example, see [examples/toggle.rs]
//!
//! [InputPin]: embedded_hal::digital::v2::InputPin
//! [OutputPin]: embedded_hal::digital::v2::OutputPin
//! [examples/toggle.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.0/examples/toggle.rs

use core::convert::Infallible;
use core::marker::PhantomData;

#[cfg(feature = "unproven")]
use crate::hal::digital::v2::toggleable;
#[cfg(feature = "unproven")]
use crate::hal::digital::v2::InputPin;
#[cfg(feature = "unproven")]
use crate::hal::digital::v2::OutputPin;
#[cfg(feature = "unproven")]
use crate::hal::digital::v2::StatefulOutputPin;
use crate::rcc::AHB;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The Parts to split the GPIO peripheral into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB) -> Self::Parts;
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

/// Alternate function 0 (type state)
pub struct AF0;

/// Alternate function 1 (type state)
pub struct AF1;

/// Alternate function 2 (type state)
pub struct AF2;

/// Alternate function 3 (type state)
pub struct AF3;

/// Alternate function 4 (type state)
pub struct AF4;

/// Alternate function 5 (type state)
pub struct AF5;

/// Alternate function 6 (type state)
pub struct AF6;

/// Alternate function 7 (type state)
pub struct AF7;

/// Alternate function 8 (type state)
pub struct AF8;

/// Alternate function 9 (type state)
pub struct AF9;

/// Alternate function 10 (type state)
pub struct AF10;

/// Alternate function 11 (type state)
pub struct AF11;

/// Alternate function 12 (type state)
pub struct AF12;

/// Alternate function 13 (type state)
pub struct AF13;

/// Alternate function 14 (type state)
pub struct AF14;

/// Alternate function 15 (type state)
pub struct AF15;

macro_rules! gpio {
    ([
        $({
            GPIO: $GPIOX:ident,
            gpio: $gpiox:ident,
            gpio_mapped: $gpioy:ident,
            gpio_mapped_ioenr: $iopxenr:ident,
            gpio_mapped_iorst: $iopxrst:ident,
            partially_erased_pin: $PXx:ident,
            pins: [
                $($PXi:ident: (
                    $pxi:ident, $i:expr, $MODE:ty, $moderi:ident, $AFR:ident, $afri:ident,
                    $bsi:ident, $bri:ident, $odri:ident, $idri:ident, $pupdri:ident, $oti:ident,
                    { $( $AFi:ty: ($into_afi:ident, $afi:ident), )* },
                ),)+
            ],
        },)+
    ]) => {
        $( use crate::pac::$GPIOX; )+


        /// GPIO discriminator enum.
        ///
        /// Use to store the gpio bank, when using
        /// fully erased pins [`PXx`]
        pub enum Gpio {
            $(
                /// GPIO Bank
                $GPIOX,
            )+
        }

        /// Fully erased pin
        ///
        /// This moves the pin type information to be known
        /// at runtime, and erases the specific compile time type of the GPIO.
        /// It does only matter, that it is a GPIO pin with a specific MODE.
        ///
        /// See [examples/gpio_erased.rs] as an example.
        ///
        /// [examples/gpio_erased.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.0/examples/gpio_erased.rs
        pub struct PXx<MODE> {
            i: u8,
            gpio: Gpio,
            _mode: PhantomData<MODE>,
        }

        impl<MODE> OutputPin for PXx<Output<MODE>> {
            type Error = Infallible;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe, write) atomic write to a stateless register
                unsafe {
                    match &self.gpio {
                        $(
                            Gpio::$GPIOX => (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)),
                        )+
                    }
                }
                Ok(())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe, write) atomic write to a stateless register
                unsafe {
                    match &self.gpio {
                        $(
                            Gpio::$GPIOX => (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))),
                        )+
                    }
                }
                Ok(())
            }
        }

        #[cfg(feature = "unproven")]
        impl<MODE> InputPin for PXx<Input<MODE>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_low()?)
            }

             fn is_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe {
                    match &self.gpio {
                        $(
                            Gpio::$GPIOX => (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0,
                        )+
                    }
                })
            }
        }

        #[cfg(feature = "unproven")]
        impl InputPin for PXx<Output<OpenDrain>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_low()?)
            }

             fn is_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe {
                    match &self.gpio {
                        $(
                            Gpio::$GPIOX => (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0,
                        )+
                    }
                })
            }
        }

        #[cfg(feature = "unproven")]
        impl <MODE> StatefulOutputPin for PXx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|b| !b)
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe {
                    match &self.gpio {
                        $(
                            Gpio::$GPIOX => (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0,
                        )+
                    }
                })
            }
        }

        #[cfg(feature = "unproven")]
        impl <MODE> toggleable::Default for PXx<Output<MODE>> {}

        $(
            doc_comment::doc_comment!{
                concat!("All Pins and associated functions for GPIO Bank: ", stringify!($GPIOX)),
                pub mod $gpiox {
                    use core::marker::PhantomData;
                    use core::convert::Infallible;

                    use crate::hal::digital::v2::OutputPin;
                    #[cfg(feature = "unproven")]
                    use crate::hal::digital::v2::InputPin;
                    #[cfg(feature = "unproven")]
                    use crate::hal::digital::v2::StatefulOutputPin;
                    #[cfg(feature = "unproven")]
                    use crate::hal::digital::v2::toggleable;
                    use crate::pac::{$gpioy, $GPIOX};

                    use crate::rcc::AHB;
                    #[allow(unused_imports)]
                    use super::{AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15};
                    use super::{
                        Floating, GpioExt, Input, OpenDrain, Output, Analog,
                        PullDown, PullUp, PushPull,
                        PXx, Gpio,
                    };

                    /// GPIO parts
                    pub struct Parts {
                        /// Opaque AFRH register
                        pub afrh: AFRH,
                        /// Opaque AFRL register
                        pub afrl: AFRL,
                        /// Opaque MODER register
                        pub moder: MODER,
                        /// Opaque OTYPER register
                        pub otyper: OTYPER,
                        /// Opaque PUPDR register
                        pub pupdr: PUPDR,
                        $(
                            /// Pin
                            pub $pxi: $PXi<$MODE>,
                        )+
                    }

                    impl GpioExt for $GPIOX {
                        type Parts = Parts;

                        fn split(self, ahb: &mut AHB) -> Parts {
                            ahb.enr().modify(|_, w| w.$iopxenr().set_bit());
                            ahb.rstr().modify(|_, w| w.$iopxrst().set_bit());
                            ahb.rstr().modify(|_, w| w.$iopxrst().clear_bit());

                            Parts {
                                afrh: AFRH { _0: () },
                                afrl: AFRL { _0: () },
                                moder: MODER { _0: () },
                                otyper: OTYPER { _0: () },
                                pupdr: PUPDR { _0: () },
                                $(
                                    $pxi: $PXi { _mode: PhantomData },
                                )+
                            }
                        }
                    }

                    /// Opaque AFRL register
                    pub struct AFRL {
                        _0: (),
                    }

                    impl AFRL {
                        // A couple device/port combos have no valid alternate functions:
                        //   - stm32f303 GPIOG and GPIOH
                        //   - stm32f318 GPIOC, GPIOD, and GPIOE
                        //   - stm32f328 GPIOE
                        #[allow(dead_code)]
                        pub(crate) fn afr(&mut self) -> &$gpioy::AFRL {
                            unsafe { &(*$GPIOX::ptr()).afrl }
                        }
                    }

                    /// Opaque AFRH register
                    pub struct AFRH {
                        _0: (),
                    }

                    impl AFRH {
                        // stm32f301 and stm32f318 don't have any high pins for GPIOF
                        #[allow(dead_code)]
                        pub(crate) fn afr(&mut self) -> &$gpioy::AFRH {
                            unsafe { &(*$GPIOX::ptr()).afrh }
                        }
                    }

                    /// Opaque MODER register
                    pub struct MODER {
                        _0: (),
                    }

                    impl MODER {
                        pub(crate) fn moder(&mut self) -> &$gpioy::MODER {
                            unsafe { &(*$GPIOX::ptr()).moder }
                        }
                    }

                    /// Opaque OTYPER register
                    pub struct OTYPER {
                        _0: (),
                    }

                    impl OTYPER {
                        pub(crate) fn otyper(&mut self) -> &$gpioy::OTYPER {
                            unsafe { &(*$GPIOX::ptr()).otyper }
                        }
                    }

                    /// Opaque PUPDR register
                    pub struct PUPDR {
                        _0: (),
                    }

                    impl PUPDR {
                        pub(crate) fn pupdr(&mut self) -> &$gpioy::PUPDR {
                            unsafe { &(*$GPIOX::ptr()).pupdr }
                        }
                    }

                    /// Partially erased pin
                    pub struct $PXx<MODE> {
                        i: u8,
                        _mode: PhantomData<MODE>,
                    }

                    impl<MODE> $PXx<MODE> {
                        /// Erases the port letter from the type
                        ///
                        /// This is useful when you want to collect the pins into an array where you
                        /// need all the elements to have the same type
                        pub fn downgrade(self) -> PXx<MODE> {
                            PXx {
                                i: self.i,
                                gpio: Gpio::$GPIOX,
                                _mode: self._mode,
                            }
                        }
                    }

                    impl<MODE> OutputPin for $PXx<Output<MODE>> {
                        type Error = Infallible;

                        fn set_high(&mut self) -> Result<(), Self::Error> {
                            // NOTE(unsafe, write) atomic write to a stateless register
                            unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
                            Ok(())
                        }

                        fn set_low(&mut self) -> Result<(), Self::Error> {
                            // NOTE(unsafe, write) atomic write to a stateless register
                            unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) }
                            Ok(())
                        }
                    }

                    #[cfg(feature = "unproven")]
                    impl<MODE> InputPin for $PXx<Input<MODE>> {
                        type Error = Infallible;

                        fn is_high(&self) -> Result<bool, Self::Error> {
                            Ok(!self.is_low()?)
                        }

                        fn is_low(&self) -> Result<bool, Self::Error> {
                            // NOTE(unsafe) atomic read with no side effects
                            Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                        }
                    }

                    #[cfg(feature = "unproven")]
                    impl InputPin for $PXx<Output<OpenDrain>> {
                        type Error = Infallible;

                        fn is_high(&self) -> Result<bool, Self::Error> {
                            Ok(!self.is_low()?)
                        }

                        fn is_low(&self) -> Result<bool, Self::Error> {
                            // NOTE(unsafe) atomic read with no side effects
                            Ok(unsafe { (*$GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
                        }
                    }

                    #[cfg(feature = "unproven")]
                    impl<MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                        fn is_set_high(&self) -> Result<bool, Self::Error> {
                            self.is_set_low().map(|b| !b)
                        }

                        fn is_set_low(&self) -> Result<bool, Self::Error> {
                            // NOTE(unsafe) atomic read with no side effects
                            Ok(unsafe { (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 })
                        }
                    }

                    #[cfg(feature = "unproven")]
                    impl<MODE> toggleable::Default for $PXx<Output<MODE>> {}

                    $(
                        doc_comment::doc_comment! {
                            concat!("Pin ", stringify!($PXi)),
                            pub struct $PXi<MODE> {
                                _mode: PhantomData<MODE>,
                            }
                        }

                        impl<MODE> $PXi<MODE> {
                            $(
                                // /// Configures the pin to serve as a specific alternate function
                                doc_comment::doc_comment!{
                                    concat!("Configures ", stringify!($PXi), " to serve as alternate function: ", stringify!($AFi)),
                                    pub fn $into_afi(
                                        self,
                                        moder: &mut MODER,
                                        afr: &mut $AFR,
                                    ) -> $PXi<$AFi> {
                                        moder.moder().modify(|_, w| w.$moderi().alternate());
                                        afr.afr().modify(|_, w| w.$afri().$afi());
                                        $PXi { _mode: PhantomData }
                                    }
                                }
                            )*

                            /// Configures the pin to operate as a floating input pin
                            pub fn into_floating_input(
                                self,
                                moder: &mut MODER,
                                pupdr: &mut PUPDR,
                            ) -> $PXi<Input<Floating>> {
                                moder.moder().modify(|_, w| w.$moderi().input());
                                pupdr.pupdr().modify(|_,w| w.$pupdri().floating());
                                $PXi { _mode: PhantomData }
                            }

                            /// Configures the pin to operate as a pulled down input pin
                            pub fn into_pull_down_input(
                                self,
                                moder: &mut MODER,
                                pupdr: &mut PUPDR,
                            ) -> $PXi<Input<PullDown>> {
                                moder.moder().modify(|_, w| w.$moderi().input());
                                pupdr.pupdr().modify(|_,w| w.$pupdri().pull_down());
                                $PXi { _mode: PhantomData }
                            }

                            /// Configures the pin to operate as a pulled up input pin
                            pub fn into_pull_up_input(
                                self,
                                moder: &mut MODER,
                                pupdr: &mut PUPDR,
                            ) -> $PXi<Input<PullUp>> {
                                moder.moder().modify(|_, w| w.$moderi().input());
                                pupdr.pupdr().modify(|_,w| w.$pupdri().pull_up());
                                $PXi { _mode: PhantomData }
                            }

                            /// Configures the pin to operate as an open drain output pin
                            pub fn into_open_drain_output(
                                self,
                                moder: &mut MODER,
                                otyper: &mut OTYPER,
                            ) -> $PXi<Output<OpenDrain>> {
                                moder.moder().modify(|_, w| w.$moderi().output());
                                otyper.otyper().modify(|_, w| w.$oti().open_drain());
                                $PXi { _mode: PhantomData }
                            }

                            /// Configures the pin to operate as an push pull output pin
                            pub fn into_push_pull_output(
                                self,
                                moder: &mut MODER,
                                otyper: &mut OTYPER,
                            ) -> $PXi<Output<PushPull>> {
                                moder.moder().modify(|_, w| w.$moderi().output());
                                otyper.otyper().modify(|_, w| w.$oti().push_pull());
                                $PXi { _mode: PhantomData }
                            }

                            /// Configures the pin to operate as analog, with disabled schmitt trigger.
                            /// This mode is suitable when the pin is connected to the DAC or ADC.
                            pub fn into_analog(
                                self,
                                moder: &mut MODER,
                                pupdr: &mut PUPDR,
                            ) -> $PXi<Analog> {
                                moder.moder().modify(|_, w| w.$moderi().analog());
                                pupdr.pupdr().modify(|_,w| w.$pupdri().floating());
                                $PXi { _mode: PhantomData }
                            }
                        }

                        impl $PXi<Output<OpenDrain>> {
                            /// Enables / disables the internal pull up
                            pub fn internal_pull_up(&mut self, pupdr: &mut PUPDR, on: bool) {
                                pupdr.pupdr().modify(|_, w| if on {
                                    w.$pupdri().pull_up()
                                } else {
                                    w.$pupdri().floating()
                                });
                            }
                        }

                        impl<MODE> $PXi<Output<MODE>> {
                            /// Erases the pin number from the type
                            ///
                            /// This is useful when you want to collect the pins into an array where you
                            /// need all the elements to have the same type
                            pub fn downgrade(self) -> $PXx<Output<MODE>> {
                                $PXx {
                                    i: $i,
                                    _mode: self._mode,
                                }
                            }
                        }

                        impl<MODE> $PXi<Input<MODE>> {
                            /// Erases the pin number from the type
                            ///
                            /// This is useful when you want to collect the pins into an array where you
                            /// need all the elements to have the same type
                            pub fn downgrade(self) -> $PXx<Input<MODE>> {
                                $PXx {
                                    i: $i,
                                    _mode: self._mode,
                                }
                            }
                        }

                        impl<MODE> OutputPin for $PXi<Output<MODE>> {
                            type Error = Infallible;

                            fn set_high(&mut self) -> Result<(), Self::Error> {
                                // NOTE(unsafe, write) atomic write to a stateless register
                                unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.$bsi().set()) }
                                Ok(())
                            }

                            fn set_low(&mut self) -> Result<(), Self::Error> {
                                // NOTE(unsafe, write) atomic write to a stateless register
                                unsafe { (*$GPIOX::ptr()).bsrr.write(|w| w.$bri().reset()) }
                                Ok(())
                            }
                        }

                        #[cfg(feature = "unproven")]
                        impl<MODE> InputPin for $PXi<Input<MODE>> {
                            type Error = Infallible;

                            fn is_high(&self) -> Result<bool, Self::Error> {
                                Ok(!self.is_low()?)
                            }

                            fn is_low(&self) -> Result<bool, Self::Error> {
                                // NOTE(unsafe) atomic read with no side effects
                                Ok(unsafe { (*$GPIOX::ptr()).idr.read().$idri().is_low()})
                            }
                        }

                        #[cfg(feature = "unproven")]
                        impl InputPin for $PXi<Output<OpenDrain>> {
                            type Error = Infallible;

                            fn is_high(&self) -> Result<bool, Self::Error> {
                                Ok(!self.is_low()?)
                            }

                            fn is_low(&self) -> Result<bool, Self::Error> {
                                // NOTE(unsafe) atomic read with no side effects
                                Ok(unsafe { (*$GPIOX::ptr()).idr.read().$idri().is_low()})
                            }
                        }

                        #[cfg(feature = "unproven")]
                        impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                            fn is_set_high(&self) -> Result<bool, Self::Error> {
                                self.is_set_low().map(|b| !b)
                            }

                            fn is_set_low(&self) -> Result<bool, Self::Error> {
                                // NOTE(unsafe) atomic read with no side effects
                                Ok(unsafe { (*$GPIOX::ptr()).odr.read().$odri().is_low()})
                            }
                        }

                        #[cfg(feature = "unproven")]
                        impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}
                    )+
                }
            }
        )+
    };

    ([
        $({
            port: ($X:ident/$x:ident, pac: $gpioy:ident),
            pins: [
                $( $i:expr => {
                    reset: $mode:ty,
                    afr: $LH:ident/$lh:ident,
                    af: [$( $af:expr ),*]
                }, )+
            ],
        },)+
    ]) => {
        paste::item! {
            gpio!([
                $({
                    GPIO: [<GPIO $X>],
                    gpio: [<gpio $x>],
                    gpio_mapped: $gpioy,
                    gpio_mapped_ioenr: [<iop $x en>],
                    gpio_mapped_iorst: [<iop $x rst>],
                    partially_erased_pin: [<P $X x>],
                    pins: [
                        $([<P $X $i>]: (
                            [<p $x $i>], $i, $mode, [<moder $i>], [<AFR $LH>], [<afr $lh $i>],
                            [<bs $i>], [<br $i>], [<odr $i>], [<idr $i>], [<pupdr $i>], [<ot $i>],
                            { $( [<AF $af>]: ([<into_af $af>], [<af $af>]), )* },
                        ),)+
                    ],
                },)+
            ]);
        }
    };
}
// auto-generated using codegen
// STM32CubeMX DB release: DB.6.0.0

#[cfg(feature = "gpio-f302")]
gpio!([
    {
        port: (A/a, pac: gpioa),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3, 7, 9, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 8, 9, 15] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 9, 15] },
            4 => { reset: Input<Floating>, afr: L/l, af: [3, 6, 7, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 6, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 6, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [0, 3, 4, 5, 6, 7, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [2, 3, 4, 5, 6, 7, 9, 10, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 4, 5, 6, 7, 8, 10, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [5, 6, 7, 9, 11, 12, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 5, 6, 7, 8, 9, 11, 15] },
            13 => { reset: AF0, afr: H/h, af: [0, 1, 3, 5, 7, 15] },
            14 => { reset: AF0, afr: H/h, af: [0, 3, 4, 6, 7, 15] },
            15 => { reset: AF0, afr: H/h, af: [0, 1, 3, 4, 6, 7, 9, 15] },
        ],
    },
    {
        port: (B/b, pac: gpiob),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [3, 6, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [3, 6, 8, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [3, 15] },
            3 => { reset: AF0, afr: L/l, af: [0, 1, 3, 6, 7, 15] },
            4 => { reset: AF0, afr: L/l, af: [0, 1, 3, 6, 7, 10, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 6, 7, 8, 10, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 4, 7, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 4, 7, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 4, 7, 9, 12, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 6, 7, 8, 9, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [3, 4, 5, 6, 7, 15] },
            13 => { reset: Input<Floating>, afr: H/h, af: [3, 5, 6, 7, 15] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5, 6, 7, 15] },
            15 => { reset: Input<Floating>, afr: H/h, af: [0, 1, 2, 4, 5, 15] },
        ],
    },
    {
        port: (C/c, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 6] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 6, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 6] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 6, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 6, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 6, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [4] },
            14 => { reset: Input<Floating>, afr: H/h, af: [] },
            15 => { reset: Input<Floating>, afr: H/h, af: [] },
        ],
    },
    {
        port: (D/d, pac: gpioc),
        pins: [
            2 => { reset: Input<Floating>, afr: L/l, af: [1] },
        ],
    },
    {
        port: (F/f, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [4, 5, 6] },
            1 => { reset: Input<Floating>, afr: L/l, af: [4, 5] },
        ],
    },
]);

#[cfg(feature = "gpio-f303e")]
gpio!([
    {
        port: (A/a, pac: gpioa),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 8, 9, 10, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3, 7, 9, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 8, 9, 15] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 9, 15] },
            4 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 5, 6, 7, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 5, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 8, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [0, 3, 4, 5, 6, 7, 8, 10, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [2, 3, 4, 5, 6, 7, 8, 9, 10, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 4, 5, 6, 7, 8, 10, 11, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [5, 6, 7, 8, 9, 10, 11, 12, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 5, 6, 7, 8, 9, 10, 11, 15] },
            13 => { reset: AF0, afr: H/h, af: [0, 1, 3, 5, 7, 10, 15] },
            14 => { reset: AF0, afr: H/h, af: [0, 3, 4, 5, 6, 7, 15] },
            15 => { reset: AF0, afr: H/h, af: [0, 1, 2, 3, 4, 5, 6, 7, 9, 15] },
        ],
    },
    {
        port: (B/b, pac: gpiob),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 4, 6, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 4, 6, 8, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [3, 15] },
            3 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
            4 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 7, 8, 10, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 7, 10, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 7, 10, 12, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 4, 7, 8, 9, 10, 12, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 6, 7, 8, 9, 10, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [3, 4, 5, 6, 7, 15] },
            13 => { reset: Input<Floating>, afr: H/h, af: [3, 5, 6, 7, 15] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5, 6, 7, 15] },
            15 => { reset: Input<Floating>, afr: H/h, af: [0, 1, 2, 4, 5, 15] },
        ],
    },
    {
        port: (C/c, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 6] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 6, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 6, 7] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 7] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 4, 5, 6] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 5, 6, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 5, 6, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 5, 6, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 4] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1] },
        ],
    },
    {
        port: (D/d, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 7, 12] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 6, 7, 12] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 5] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7, 12] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7, 12] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 7, 12] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7, 12] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7, 12] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 7, 12] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 7, 12] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 7, 12] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 7, 12] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 7, 12] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 12] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 12] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 6, 12] },
        ],
    },
    {
        port: (E/e, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 6, 7, 12] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 6, 7, 12] },
            2 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3, 5, 6, 12] },
            3 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3, 5, 6, 12] },
            4 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3, 5, 6, 12] },
            5 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3, 5, 6, 12] },
            6 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 5, 6, 12] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 5, 12] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 5, 12] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 5, 12] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 5, 6, 12] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 7, 12] },
        ],
    },
    {
        port: (F/f, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 5, 6] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 5] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 12] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 7, 12] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 5, 12] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 5, 12] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 12] },
        ],
    },
    {
        port: (G/g, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 12] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 12] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 12] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 12] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 12] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 12] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 12] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 12] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1] },
        ],
    },
    {
        port: (H/h, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 12] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1] },
        ],
    },
]);

#[cfg(feature = "gpio-f303")]
gpio!([
    {
        port: (A/a, pac: gpioa),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 8, 9, 10, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3, 7, 9, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 8, 9, 15] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 9, 15] },
            4 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 5, 6, 7, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 5, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 8, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 8, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [0, 4, 5, 6, 7, 8, 10, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [3, 4, 5, 6, 7, 8, 9, 10, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 4, 6, 7, 8, 10, 11, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [6, 7, 8, 9, 10, 11, 12, 14, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 6, 7, 8, 9, 10, 11, 14, 15] },
            13 => { reset: AF0, afr: H/h, af: [0, 1, 3, 5, 7, 10, 15] },
            14 => { reset: AF0, afr: H/h, af: [0, 3, 4, 5, 6, 7, 15] },
            15 => { reset: AF0, afr: H/h, af: [0, 1, 2, 4, 5, 6, 7, 9, 15] },
        ],
    },
    {
        port: (B/b, pac: gpiob),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 4, 6, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 4, 6, 8, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [3, 15] },
            3 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
            4 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 7, 10, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 6, 7, 10, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 5, 7, 10, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 4, 8, 9, 10, 12, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 6, 8, 9, 10, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [3, 4, 5, 6, 7, 15] },
            13 => { reset: Input<Floating>, afr: H/h, af: [3, 5, 6, 7, 15] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5, 6, 7, 15] },
            15 => { reset: Input<Floating>, afr: H/h, af: [0, 1, 2, 4, 5, 15] },
        ],
    },
    {
        port: (C/c, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 3] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 6] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 6, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 6, 7] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 7] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 5, 6] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 5, 6, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 5, 6, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 5, 6, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [4] },
            14 => { reset: Input<Floating>, afr: H/h, af: [] },
            15 => { reset: Input<Floating>, afr: H/h, af: [] },
        ],
    },
    {
        port: (D/d, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 7] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 6, 7] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 5] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 6] },
        ],
    },
    {
        port: (E/e, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 7] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 7] },
            2 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3] },
            3 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3] },
            4 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3] },
            5 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3] },
            6 => { reset: Input<Floating>, afr: L/l, af: [0, 1] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 6] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 7] },
        ],
    },
    {
        port: (F/f, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [4, 6] },
            1 => { reset: Input<Floating>, afr: L/l, af: [4] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 7] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5] },
        ],
    },
]);

#[cfg(feature = "gpio-f333")]
gpio!([
    {
        port: (A/a, pac: gpioa),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 9, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 8, 9, 15] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7, 9, 15] },
            4 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 5, 7, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 5, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 5, 6, 13, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 5, 6, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [0, 6, 7, 13, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [3, 6, 7, 9, 10, 13, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 6, 7, 8, 10, 13, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [6, 7, 9, 11, 12, 13, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 6, 7, 8, 9, 11, 13, 15] },
            13 => { reset: AF0, afr: H/h, af: [0, 1, 3, 5, 7, 15] },
            14 => { reset: AF0, afr: H/h, af: [0, 3, 4, 6, 7, 15] },
            15 => { reset: AF0, afr: H/h, af: [0, 1, 3, 4, 5, 7, 9, 13, 15] },
        ],
    },
    {
        port: (B/b, pac: gpiob),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 6, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 6, 8, 13, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [3, 13, 15] },
            3 => { reset: AF0, afr: L/l, af: [0, 1, 3, 5, 7, 10, 12, 13, 15] },
            4 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 5, 7, 10, 13, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 5, 7, 10, 13, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 4, 7, 12, 13, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 4, 7, 10, 13, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 4, 7, 9, 12, 13, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 4, 6, 7, 8, 9, 13, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 13, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7, 13, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [3, 6, 7, 13, 15] },
            13 => { reset: Input<Floating>, afr: H/h, af: [3, 6, 7, 13, 15] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 6, 7, 13, 15] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 13, 15] },
        ],
    },
    {
        port: (C/c, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 6] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [4] },
            14 => { reset: Input<Floating>, afr: H/h, af: [] },
            15 => { reset: Input<Floating>, afr: H/h, af: [] },
        ],
    },
    {
        port: (D/d, pac: gpioc),
        pins: [
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
        ],
    },
    {
        port: (F/f, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [6] },
            1 => { reset: Input<Floating>, afr: L/l, af: [] },
        ],
    },
]);

#[cfg(feature = "gpio-f373")]
gpio!([
    {
        port: (A/a, pac: gpioa),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 7, 8, 11, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 2, 3, 6, 7, 9, 11, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 6, 7, 8, 9, 11, 15] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 6, 7, 9, 11, 15] },
            4 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 5, 6, 7, 10, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 5, 7, 9, 10, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 5, 8, 9, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 5, 8, 9, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [0, 2, 4, 5, 7, 10, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [2, 3, 4, 5, 7, 9, 10, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 4, 5, 7, 9, 10, 15] },
            11 => { reset: Input<Floating>, afr: H/h, af: [2, 5, 6, 7, 8, 9, 10, 14, 15] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 6, 7, 8, 9, 10, 14, 15] },
            13 => { reset: AF0, afr: H/h, af: [0, 1, 2, 3, 5, 6, 7, 10, 15] },
            14 => { reset: AF0, afr: H/h, af: [0, 3, 4, 10, 15] },
            15 => { reset: AF0, afr: H/h, af: [0, 1, 3, 4, 5, 6, 10, 15] },
        ],
    },
    {
        port: (B/b, pac: gpiob),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 5, 10, 15] },
            1 => { reset: Input<Floating>, afr: L/l, af: [2, 3, 15] },
            2 => { reset: Input<Floating>, afr: L/l, af: [15] },
            3 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 5, 6, 7, 9, 10, 15] },
            4 => { reset: AF0, afr: L/l, af: [0, 1, 2, 3, 5, 6, 7, 9, 10, 15] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 5, 6, 7, 10, 11, 15] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 7, 9, 10, 11, 15] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 4, 7, 9, 10, 11, 15] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 15] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 4, 5, 6, 7, 8, 9, 11, 15] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5, 6, 7, 15] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5, 7, 9, 15] },
            15 => { reset: Input<Floating>, afr: H/h, af: [0, 1, 2, 3, 5, 9, 15] },
        ],
    },
    {
        port: (C/c, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 5] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 5] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 3, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 3, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 5] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 5] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 5] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 5] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 6, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 6, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 6, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [] },
            14 => { reset: Input<Floating>, afr: H/h, af: [] },
            15 => { reset: Input<Floating>, afr: H/h, af: [] },
        ],
    },
    {
        port: (D/d, pac: gpiod),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 2] },
            3 => { reset: Input<Floating>, afr: L/l, af: [1, 5, 7] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1, 5, 7] },
            5 => { reset: Input<Floating>, afr: L/l, af: [1, 7] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 5, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 5, 7] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 5, 7] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 3, 7] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3, 7] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 2, 3] },
        ],
    },
    {
        port: (E/e, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 7] },
            1 => { reset: Input<Floating>, afr: L/l, af: [1, 7] },
            2 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3] },
            3 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3] },
            4 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3] },
            5 => { reset: Input<Floating>, afr: L/l, af: [0, 1, 3] },
            6 => { reset: Input<Floating>, afr: L/l, af: [0, 1] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1] },
            8 => { reset: Input<Floating>, afr: H/h, af: [1] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1] },
            11 => { reset: Input<Floating>, afr: H/h, af: [1] },
            12 => { reset: Input<Floating>, afr: H/h, af: [1] },
            13 => { reset: Input<Floating>, afr: H/h, af: [1] },
            14 => { reset: Input<Floating>, afr: H/h, af: [1] },
            15 => { reset: Input<Floating>, afr: H/h, af: [1, 7] },
        ],
    },
    {
        port: (F/f, pac: gpioc),
        pins: [
            0 => { reset: Input<Floating>, afr: L/l, af: [4] },
            1 => { reset: Input<Floating>, afr: L/l, af: [4] },
            2 => { reset: Input<Floating>, afr: L/l, af: [1, 4] },
            4 => { reset: Input<Floating>, afr: L/l, af: [1] },
            6 => { reset: Input<Floating>, afr: L/l, af: [1, 2, 4, 5, 7] },
            7 => { reset: Input<Floating>, afr: L/l, af: [1, 4, 7] },
            9 => { reset: Input<Floating>, afr: H/h, af: [1, 2] },
            10 => { reset: Input<Floating>, afr: H/h, af: [1] },
        ],
    },
]);
