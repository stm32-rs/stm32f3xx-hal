//! General Purpose Input / Output
//!
//! To use the GPIO pins, you first need to configure the GPIO bank (GPIOA, GPIOB, ...) that you
//! are interested in. This is done using the [GpioExt::split](trait.GpioExt.html#tymethod.split) function.
//!
//! ```
//! let dp = pac::Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//!
//! let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//! ```
//!
//! The resulting [Parts](gpioa/struct.Parts.html) struct contains one field for each
//! pin, as well as some shared registers.
//!
//! To use a pin, first use the relevant `into_...` function in the [pin](gpioa/struct.PA0.html).
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
//! [InputPin]: ../prelude/trait._embedded_hal_digital_InputPin.html
//! [OutputPin]: ../prelude/trait._embedded_hal_digital_OutputPin.html
//! [examples/toggle.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.4.3/examples/toggle.rs

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
    /// The to split the GPIO into
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
            devices: [$($device:expr,)+],
            devices_except: [$($device_except:expr,)*],
            GPIO: $GPIOX:ident,
            gpio: $gpiox:ident,
            gpio_mapped: $gpioy:ident,
            gpio_mapped_ioenr: $iopxenr:ident,
            gpio_mapped_iorst: $iopxrst:ident,
            partially_erased_pin: $PXx:ident,
            pins: [
                $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $moderi:ident, $AFR:ident, $afri:ident,
                  $bsi:ident, $bri:ident, $odri:ident, $idri:ident, $pupdri:ident, $oti:ident, [
                    $($AFi_common:ty: ($into_afi_common:ident, $afi_common:ident,),)*
                ], [
                    $($AFi:ty: ($into_afi:ident, $afi:ident, [$($afi_devices:expr,)*],),)*
                ]),)+
            ],
        },)+
    ]) => {
        $(
            #[cfg(all(any(
                $(feature = $device,)+
            ), not(any(
                $(feature = $device_except,)*
            ))))]
            use crate::pac::$GPIOX;
        )+

        pub enum Gpio {
            $(
                #[cfg(all(any(
                    $(feature = $device,)+
                ), not(any(
                    $(feature = $device_except,)*
                ))))]
                $GPIOX,
            )+
        }

        /// Fully erased pin
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
                            #[cfg(all(any(
                                $(feature = $device,)+
                            ), not(any(
                                $(feature = $device_except,)*
                            ))))]
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
                            #[cfg(all(any(
                                $(feature = $device,)+
                            ), not(any(
                                $(feature = $device_except,)*
                            ))))]
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
                            #[cfg(all(any(
                                $(feature = $device,)+
                            ), not(any(
                                $(feature = $device_except,)*
                            ))))]
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
                            #[cfg(all(any(
                                $(feature = $device,)+
                            ), not(any(
                                $(feature = $device_except,)*
                            ))))]
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
                            #[cfg(all(any(
                                $(feature = $device,)+
                            ), not(any(
                                $(feature = $device_except,)*
                            ))))]
                            Gpio::$GPIOX => (*$GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0,
                        )+
                    }
                })
            }
        }

        #[cfg(feature = "unproven")]
        impl <MODE> toggleable::Default for PXx<Output<MODE>> {}

        $(
            /// GPIO
            #[cfg(all(any(
                $(feature = $device,)+
            ), not(any(
                $(feature = $device_except,)*
            ))))]
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
                    /// Pin
                    pub struct $PXi<MODE> {
                        _mode: PhantomData<MODE>,
                    }

                    impl<MODE> $PXi<MODE> {
                        $(
                            /// Configures the pin to serve as a specific alternate function
                            pub fn $into_afi_common(
                                self,
                                moder: &mut MODER,
                                afr: &mut $AFR,
                            ) -> $PXi<$AFi_common> {
                                moder.moder().modify(|_, w| w.$moderi().alternate());
                                afr.afr().modify(|_, w| w.$afri().$afi_common());
                                $PXi { _mode: PhantomData }
                            }
                        )*

                        $(
                            /// Configures the pin to serve as a specific alternate function
                            #[cfg(any(
                                $(feature = $afi_devices,)*
                            ))]
                            pub fn $into_afi(
                                self,
                                moder: &mut MODER,
                                afr: &mut $AFR,
                            ) -> $PXi<$AFi> {
                                moder.moder().modify(|_, w| w.$moderi().alternate());
                                afr.afr().modify(|_, w| w.$afri().$afi());
                                $PXi { _mode: PhantomData }
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
        )+
    }
}

gpio!([
    {
        devices: [
            "stm32f301",
            "stm32f318",
            "stm32f302",
            "stm32f303",
            "stm32f373",
            "stm32f378",
            "stm32f334",
            "stm32f328",
            "stm32f358",
            "stm32f398",
        ],
        devices_except: [],
        GPIO: GPIOA,
        gpio: gpioa,
        gpio_mapped: gpioa,
        gpio_mapped_ioenr: iopaen,
        gpio_mapped_iorst: ioparst,
        partially_erased_pin: PAx,
        pins: [
            PA0: (pa0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF8: (into_af8, af8, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f373", "stm32f378",],),
            ]),
            PA1: (pa1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF15: (into_af15, af15,),
            ], [
                AF0: (into_af0, af0, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f373", "stm32f378",],),
                AF11: (into_af11, af11, ["stm32f373", "stm32f378",],),
            ]),
            PA2: (pa2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF8: (into_af8, af8,),
                AF9: (into_af9, af9,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f373", "stm32f378",],),
                AF11: (into_af11, af11, ["stm32f373", "stm32f378",],),
            ]),
            PA3: (pa3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f373", "stm32f378",],),
                AF11: (into_af11, af11, ["stm32f373", "stm32f378",],),
            ]),
            PA4: (pa4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe","stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f373", "stm32f378",],),
            ]),
            PA5: (pa5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF15: (into_af15, af15,),
            ], [
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f373", "stm32f378",],),
                AF9: (into_af9, af9, ["stm32f373", "stm32f378",],),
                AF10: (into_af10, af10, ["stm32f373", "stm32f378",],),
            ]),
            PA6: (pa6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f373", "stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334", "stm32f328",],),
            ]),
            PA7: (pa7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f373", "stm32f378", "stm32f358",],),
                AF9: (into_af9, af9, ["stm32f373", "stm32f378",],),
            ]),
            PA8: (pa8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [
                AF0: (into_af0, af0,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF3: (into_af3, af3, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f303", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PA9: (pa9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f373", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PA10: (pa10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], [
                AF4: (into_af4, af4, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f373", "stm32f378", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f373", "stm32f378",],),
                AF11: (into_af11, af11, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PA11: (pa11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f373", "stm32f378", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
                AF14: (into_af14, af14, ["stm32f302", "stm32f303xb", "stm32f303xc",],),
            ]),
            PA12: (pa12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [
                AF1: (into_af1, af1,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF8: (into_af8, af8,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
                AF14: (into_af14, af14, ["stm32f302", "stm32f303xb", "stm32f303xc",],),
            ]),
            PA13: (pa13, 13, AF0, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f373", "stm32f378", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
            ]),
            PA14: (pa14, 14, AF0, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [
                AF0: (into_af0, af0,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF15: (into_af15, af15,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f373", "stm32f378",],),
            ]),
            PA15: (pa15, 15, AF0, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF4: (into_af4, af4,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f303x6", "stm32f303x8", "stm32f318", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f373", "stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f301",
            "stm32f318",
            "stm32f302",
            "stm32f303",
            "stm32f334",
            "stm32f328",
            "stm32f358",
            "stm32f378",
            "stm32f398",
        ],
        devices_except: [],
        GPIO: GPIOB,
        gpio: gpiob,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopben,
        gpio_mapped_iorst: iopbrst,
        partially_erased_pin: PBx,
        pins: [
            PB0: (pb0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF3: (into_af3, af3,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f378",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f378",],),
            ]),
            PB1: (pb1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF3: (into_af3, af3,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB2: (pb2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [], [
                AF3: (into_af3, af3, ["stm32f301", "stm32f302", "stm32f303", "stm32f334",],),
                AF13: (into_af13, af13, ["stm32f334",],),
                AF15: (into_af15, af15, ["stm32f301", "stm32f302", "stm32f303", "stm32f334",],),
            ]),
            PB3: (pb3, 3, AF0, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f378",],),
                AF10: (into_af10, af10, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f334",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB4: (pb4, 4, AF0, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB5: (pb5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [
                AF1: (into_af1, af1,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB6: (pb6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f378",],),
                AF10: (into_af10, af10, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f378",],),
                AF12: (into_af12, af12, ["stm32f334",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB7: (pb7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f378",],),
                AF10: (into_af10, af10, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f378",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB8: (pb8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f378",],),
                AF6: (into_af6, af6, ["stm32f378",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f334", "stm32f328", "stm32f378", "stm32f398",],),
                AF8: (into_af8, af8, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",  "stm32f358", "stm32f378", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f378",],),
                AF12: (into_af12, af12, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB9: (pb9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [
                AF1: (into_af1, af1,),
                AF4: (into_af4, af4,),
                AF6: (into_af6, af6,),
                AF8: (into_af8, af8,),
                AF15: (into_af15, af15,),
            ], [
                AF2: (into_af2, af2, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f378",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f303xd", "stm32f303xe", "stm32f318", "stm32f334", "stm32f328", "stm32f378", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f378", "stm32f398",],),
                AF10: (into_af10, af10, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF11: (into_af11, af11, ["stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB10: (pb10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF5: (into_af5, af5, ["stm32f378",],),
                AF6: (into_af6, af6, ["stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB11: (pb11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
                AF15: (into_af15, af15, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
            ]),
            PB12: (pb12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], [
                AF3: (into_af3, af3, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
                AF15: (into_af15, af15, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
            ]),
            PB13: (pb13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], [
                AF3: (into_af3, af3, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF13: (into_af13, af13, ["stm32f334",],),
                AF15: (into_af15, af15, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
            ]),
            PB14: (pb14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], [
                AF5: (into_af5, af5, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
            PB15: (pb15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF15: (into_af15, af15,),
            ], [
                AF0: (into_af0, af0, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f378",],),
                AF4: (into_af4, af4, ["stm32f301", "stm32f318", "stm32f302", "stm32f303", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f318", "stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", "stm32f358", "stm32f378", "stm32f398",],),
                AF9: (into_af9, af9, ["stm32f378",],),
                AF13: (into_af13, af13, ["stm32f334",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f373",
        ],
        devices_except: [],
        GPIO: GPIOB,
        gpio: gpiob,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopben,
        gpio_mapped_iorst: iopbrst,
        partially_erased_pin: PBx,
        pins: [
            PB0: (pb0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], []),
            PB1: (pb1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF15: (into_af15, af15,),
            ], []),
            PB2: (pb2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF15: (into_af15, af15,),
            ], []),
            PB3: (pb3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], []),
            PB4: (pb4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF10: (into_af10, af10,),
                AF15: (into_af15, af15,),
            ], []),
            PB5: (pb5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF4: (into_af4, af4,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF10: (into_af10, af10,),
                AF11: (into_af11, af11,),
                AF15: (into_af15, af15,),
            ], []),
            PB6: (pb6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF10: (into_af10, af10,),
                AF11: (into_af11, af11,),
                AF15: (into_af15, af15,),
            ], []),
            PB7: (pb7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF10: (into_af10, af10,),
                AF11: (into_af11, af11,),
                AF15: (into_af15, af15,),
            ], []),
            PB8: (pb8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF4: (into_af4, af4,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF8: (into_af8, af8,),
                AF9: (into_af9, af9,),
                AF11: (into_af11, af11,),
                AF15: (into_af15, af15,),
            ], []),
            PB9: (pb9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF4: (into_af4, af4,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF8: (into_af8, af8,),
                AF9: (into_af9, af9,),
                AF11: (into_af11, af11,),
                AF15: (into_af15, af15,),
            ], []),
            PB10: (pb10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], []),
            PB11: (pb11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
                AF7: (into_af7, af7,),
                AF15: (into_af15, af15,),
            ], []),
            PB14: (pb14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF7: (into_af7, af7,),
                AF9: (into_af9, af9,),
                AF15: (into_af15, af15,),
            ], []),
            PB15: (pb15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
                AF9: (into_af9, af9,),
                AF15: (into_af15, af15,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f301",
            "stm32f318",
            "stm32f373",
            "stm32f378",
            "stm32f334",
            "stm32f328",
            "stm32f358",
            "stm32f398",
        ],
        devices_except: [],
        GPIO: GPIOC,
        gpio: gpioc,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopcen,
        gpio_mapped_iorst: iopcrst,
        partially_erased_pin: PCx,
        pins: [
            PC0: (pc0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f398",],),
            ]),
            PC1: (pc1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f398",],),
            ]),
            PC2: (pc2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
            ]),
            PC3: (pc3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC4: (pc4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC5: (pc5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f301", "stm32f334", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC6: (pc6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f334",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC7: (pc7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f334",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f358", "stm32f398",],),
            ]),
            PC8: (pc8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f334",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f358", "stm32f398",],),
            ]),
            PC9: (pc9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f301", "stm32f334", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f301", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f358", "stm32f398",],),
            ]),
            PC10: (pc10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC11: (pc11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF3: (into_af3, af3, ["stm32f334",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC12: (pc12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], [
                AF1: (into_af1, af1, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF3: (into_af3, af3, ["stm32f334",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f301", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f301", "stm32f373", "stm32f378", "stm32f334", "stm32f358", "stm32f398",],),
            ]),
            PC13: (pc13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], [
                AF1: (into_af1, af1, ["stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f301", "stm32f318", "stm32f334", "stm32f328", "stm32f358", "stm32f398",],),
            ]),
            PC14: (pc14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], [
                AF1: (into_af1, af1, ["stm32f378", "stm32f398",],),
            ]),
            PC15: (pc15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], [
                AF1: (into_af1, af1, ["stm32f378", "stm32f398",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f302",
            "stm32f303",
        ],
        devices_except: [],
        GPIO: GPIOC,
        gpio: gpioc,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopcen,
        gpio_mapped_iorst: iopcrst,
        partially_erased_pin: PCx,
        pins: [
            PC0: (pc0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF1: (into_af1, af1,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PC1: (pc1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
            ], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PC2: (pc2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe", "stm32f303x6", "stm32f303x8",],),
                AF3: (into_af3, af3, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC3: (pc3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [
                AF1: (into_af1, af1,),
                AF6: (into_af6, af6,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe", "stm32f303x6", "stm32f303x8",],),
            ]),
            PC4: (pc4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF1: (into_af1, af1,),
                AF7: (into_af7, af7,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe", "stm32f303x6", "stm32f303x8",],),
            ]),
            PC5: (pc5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF7: (into_af7, af7,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe", "stm32f303x6", "stm32f303x8",],),
            ]),
            PC6: (pc6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF7: (into_af7, af7,),
            ], [
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC7: (pc7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF7: (into_af7, af7, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC8: (pc8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF7: (into_af7, af7, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC9: (pc9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF3: (into_af3, af3, ["stm32f303xd", "stm32f303xe",],),
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe", ],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC10: (pc10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF7: (into_af7, af7,),
            ], [
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC11: (pc11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [
                AF1: (into_af1, af1,),
                AF7: (into_af7, af7,),
            ], [
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC12: (pc12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [
                AF1: (into_af1, af1,),
                AF7: (into_af7, af7,),
            ], [
                AF4: (into_af4, af4, ["stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f302", "stm32f303xb", "stm32f303xc", "stm32f303xd", "stm32f303xe",],),
            ]),
            PC13: (pc13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [
                AF4: (into_af4, af4,),
            ], [
                AF1: (into_af1, af1, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PC14: (pc14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], [
                AF1: (into_af1, af1, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PC15: (pc15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], [
                AF1: (into_af1, af1, ["stm32f303xd", "stm32f303xe",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f301",
        ],
        devices_except: [],
        GPIO: GPIOD,
        gpio: gpiod,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopden,
        gpio_mapped_iorst: iopdrst,
        partially_erased_pin: PDx,
        pins: [
            PD2: (pd2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f303",
        ],
        devices_except: [
            "stm32f303xb",
            "stm32f303xc",
            "stm32f303xd",
            "stm32f303xe",
        ],
        GPIO: GPIOD,
        gpio: gpiod,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopden,
        gpio_mapped_iorst: iopdrst,
        partially_erased_pin: PDx,
        pins: [
            PD2: (pd2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f334",
        ],
        devices_except: [],
        GPIO: GPIOD,
        gpio: gpiod,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopden,
        gpio_mapped_iorst: iopdrst,
        partially_erased_pin: PDx,
        pins: [
            PD0: (pd0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [], []),
            PD1: (pd1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], []),
            PD2: (pd2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], []),
            PD3: (pd3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [], []),
            PD4: (pd4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [], []),
            PD5: (pd5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], []),
            PD6: (pd6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [], []),
            PD7: (pd7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [], []),
            PD8: (pd8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [], []),
            PD9: (pd9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], []),
            PD10: (pd10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [], []),
            PD11: (pd11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], []),
            PD12: (pd12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], []),
            PD13: (pd13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], []),
            PD14: (pd14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], []),
            PD15: (pd15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], []),
        ],
    },
    {
        devices: [
            "stm32f302",
            "stm32f303xb",
            "stm32f303xc",
            "stm32f303xd",
            "stm32f303xe",
            "stm32f318",
            "stm32f373",
            "stm32f378",
            "stm32f328",
            "stm32f358",
            "stm32f398",
        ],
        devices_except: [],
        GPIO: GPIOD,
        gpio: gpiod,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopden,
        gpio_mapped_iorst: iopdrst,
        partially_erased_pin: PDx,
        pins: [
            PD0: (pd0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD1: (pd1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f373", "stm32f378",],),
                AF4: (into_af4, af4, ["stm32f303", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f303", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD2: (pd2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f328", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f328", "stm32f358", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f303", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f302", "stm32f303", "stm32f358", "stm32f398",],),
            ]),
            PD3: (pd3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD4: (pd4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD5: (pd5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD6: (pd6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD7: (pd7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD8: (pd8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f373", "stm32f378",],),
                AF5: (into_af5, af5, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD9: (pd9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f373", "stm32f378",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD10: (pd10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD11: (pd11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD12: (pd12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD13: (pd13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD14: (pd14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
            PD15: (pd15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], [
                AF1: (into_af1, af1, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f302", "stm32f303", "stm32f373", "stm32f378", "stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f302", "stm32f303", "stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe", "stm32f398",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f302",
            "stm32f303xb",
            "stm32f303xc",
            "stm32f303xd",
            "stm32f303xe",
        ],
        devices_except: [],
        GPIO: GPIOE,
        gpio: gpioe,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopeen,
        gpio_mapped_iorst: ioperst,
        partially_erased_pin: PEx,
        pins: [
            PE0: (pe0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
            ], [
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE1: (pe1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF1: (into_af1, af1,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
            ], [
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE2: (pe2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE3: (pe3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE4: (pe4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE5: (pe5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF3: (into_af3, af3,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE6: (pe6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF0: (into_af0, af0,),
                AF1: (into_af1, af1,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF6: (into_af6, af6, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE7: (pe7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE8: (pe8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE9: (pe9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE10: (pe10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE11: (pe11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE12: (pe12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE13: (pe13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE14: (pe14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF6: (into_af6, af6,),
            ], [
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PE15: (pe15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF7: (into_af7, af7,),
            ], [
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f318",
            "stm32f328",
            "stm32f358",
            "stm32f373",
            "stm32f378",
            "stm32f398",
        ],
        devices_except: [],
        GPIO: GPIOE,
        gpio: gpioe,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopeen,
        gpio_mapped_iorst: ioperst,
        partially_erased_pin: PEx,
        pins: [
            PE0: (pe0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF7: (into_af7, af7, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE1: (pe1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF7: (into_af7, af7, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE2: (pe2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [], [
                AF0: (into_af0, af0, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE3: (pe3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [], [
                AF0: (into_af0, af0, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE4: (pe4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [], [
                AF0: (into_af0, af0, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE5: (pe5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], [
                AF0: (into_af0, af0, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE6: (pe6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [], [
                AF0: (into_af0, af0, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF6: (into_af6, af6, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE7: (pe7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE8: (pe8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE9: (pe9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE10: (pe10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE11: (pe11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE12: (pe12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE13: (pe13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE14: (pe14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF6: (into_af6, af6, ["stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PE15: (pe15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF7: (into_af7, af7, ["stm32f358", "stm32f373", "stm32f378", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f301",
        ],
        devices_except: [],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF4: (into_af4, af4,),
                AF5: (into_af5, af5,),
                AF6: (into_af6, af6,),
            ], []),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF4: (into_af4, af4,),
                AF5: (into_af5, af5,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f303",
        ],
        devices_except: [
            "stm32f303xb",
            "stm32f303xc",
            "stm32f303xd",
            "stm32f303xe",
        ],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF6: (into_af6, af6,),
            ], []),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], []),
        ],
    },
    {
        devices: [
            "stm32f302",
        ],
        devices_except: [],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF4: (into_af4, af4,),
                AF6: (into_af6, af6,),
            ], []),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF4: (into_af4, af4,),
            ], []),
            PF2: (pf2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
            ], []),
            PF4: (pf4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], []),
            PF5: (pf5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], []),
            PF6: (pf6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
            ], []),
            PF9: (pf9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], []),
            PF10: (pf10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f334",
        ],
        devices_except: [],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF6: (into_af6, af6,),
            ], []),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], []),
            PF2: (pf2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [], []),
            PF4: (pf4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [], []),
            PF5: (pf5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], []),
            PF6: (pf6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [], []),
            PF9: (pf9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], []),
            PF10: (pf10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [], []),
        ],
    },
    {
        devices: [
            "stm32f303xb",
            "stm32f303xc",
            "stm32f303xd",
            "stm32f303xe",
        ],
        devices_except: [],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF4: (into_af4, af4,),
                AF6: (into_af6, af6,),
            ], [
                AF1: (into_af1, af1, ["stm32f303xd", "stm32f303xe",],),
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF4: (into_af4, af4,),
            ], [
                AF1: (into_af1, af1, ["stm32f303xd", "stm32f303xe",],),
                AF5: (into_af5, af5, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF2: (pf2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF3: (pf3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [], []),
            PF4: (pf4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
            ], [
                AF3: (into_af3, af3, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF5: (pf5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], []),
            PF6: (pf6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
            ], [
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF7: (pf7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [], []),
            PF8: (pf8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [], []),
            PF9: (pf9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF10: (pf10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF3: (into_af3, af3,),
                AF5: (into_af5, af5,),
            ], [
                AF2: (into_af2, af2, ["stm32f303xd", "stm32f303xe",],),
                AF12: (into_af12, af12, ["stm32f303xd", "stm32f303xe",],),
            ]),
            PF11: (pf11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], []),
            PF12: (pf12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], []),
            PF13: (pf13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], []),
            PF14: (pf14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], []),
            PF15: (pf15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], []),
        ],
    },
    {
        devices: [
            "stm32f373",
        ],
        devices_except: [],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF4: (into_af4, af4,),
            ], []),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF4: (into_af4, af4,),
            ], []),
            PF2: (pf2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
                AF4: (into_af4, af4,),
            ], []),
            PF4: (pf4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF1: (into_af1, af1,),
            ], []),
            PF6: (pf6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF4: (into_af4, af4,),
                AF5: (into_af5, af5,),
                AF7: (into_af7, af7,),
            ], []),
            PF7: (pf7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF4: (into_af4, af4,),
                AF7: (into_af7, af7,),
            ], []),
            PF9: (pf9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], []),
            PF10: (pf10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f318",
            "stm32f328",
            "stm32f358",
            "stm32f378",
            "stm32f398",
        ],
        devices_except: [],
        GPIO: GPIOF,
        gpio: gpiof,
        gpio_mapped: gpioc,
        gpio_mapped_ioenr: iopfen,
        gpio_mapped_iorst: iopfrst,
        partially_erased_pin: PFx,
        pins: [
            PF0: (pf0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF4: (into_af4, af4, ["stm32f318", "stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f318", "stm32f398",],),
                AF6: (into_af6, af6, ["stm32f318", "stm32f328", "stm32f358", "stm32f398",],),
            ]),
            PF1: (pf1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF4: (into_af4, af4, ["stm32f318", "stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f318", "stm32f398",],),
            ]),
            PF2: (pf2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF4: (into_af4, af4, ["stm32f378",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF3: (pf3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF4: (pf4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f398",],),
                AF3: (into_af3, af3, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF5: (pf5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF6: (pf6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f358", "stm32f378", "stm32f398",],),
                AF4: (into_af4, af4, ["stm32f358", "stm32f378", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f378",],),
                AF7: (into_af7, af7, ["stm32f358", "stm32f378", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF7: (pf7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [], [
                AF1: (into_af1, af1, ["stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF4: (into_af4, af4, ["stm32f378",],),
                AF7: (into_af7, af7, ["stm32f378",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF8: (pf8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF9: (pf9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF3: (into_af3, af3, ["stm32f398",],),
                AF5: (into_af5, af5, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF10: (pf10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [], [
                AF1: (into_af1, af1, ["stm32f358", "stm32f378", "stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF3: (into_af3, af3, ["stm32f358", "stm32f398",],),
                AF5: (into_af5, af5, ["stm32f358", "stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF11: (pf11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
            ]),
            PF12: (pf12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF13: (pf13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF14: (pf14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
            PF15: (pf15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [], [
                AF1: (into_af1, af1, ["stm32f398",],),
                AF2: (into_af2, af2, ["stm32f398",],),
                AF12: (into_af12, af12, ["stm32f398",],),
            ]),
        ],
    },
    {
        devices: [
            "stm32f303xd",
            "stm32f303xe",
        ],
        devices_except: [],
        GPIO: GPIOG,
        gpio: gpiog,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iopgen,
        gpio_mapped_iorst: iopgrst,
        partially_erased_pin: PGx,
        pins: [
            PG0: (pg0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PG1: (pg1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PG2: (pg2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PG3: (pg3, 3, Input<Floating>, moder3, AFRL, afrl3, bs3, br3, odr3, idr3, pupdr3, ot3, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PG4: (pg4, 4, Input<Floating>, moder4, AFRL, afrl4, bs4, br4, odr4, idr4, pupdr4, ot4, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PG5: (pg5, 5, Input<Floating>, moder5, AFRL, afrl5, bs5, br5, odr5, idr5, pupdr5, ot5, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PG6: (pg6, 6, Input<Floating>, moder6, AFRL, afrl6, bs6, br6, odr6, idr6, pupdr6, ot6, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG7: (pg7, 7, Input<Floating>, moder7, AFRL, afrl7, bs7, br7, odr7, idr7, pupdr7, ot7, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG8: (pg8, 8, Input<Floating>, moder8, AFRH, afrh8, bs8, br8, odr8, idr8, pupdr8, ot8, [
                AF1: (into_af1, af1,),
            ], []),
            PG9: (pg9, 9, Input<Floating>, moder9, AFRH, afrh9, bs9, br9, odr9, idr9, pupdr9, ot9, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG10: (pg10, 10, Input<Floating>, moder10, AFRH, afrh10, bs10, br10, odr10, idr10, pupdr10, ot10, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG11: (pg11, 11, Input<Floating>, moder11, AFRH, afrh11, bs11, br11, odr11, idr11, pupdr11, ot11, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG12: (pg12, 12, Input<Floating>, moder12, AFRH, afrh12, bs12, br12, odr12, idr12, pupdr12, ot12, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG13: (pg13, 13, Input<Floating>, moder13, AFRH, afrh13, bs13, br13, odr13, idr13, pupdr13, ot13, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG14: (pg14, 14, Input<Floating>, moder14, AFRH, afrh14, bs14, br14, odr14, idr14, pupdr14, ot14, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
            PG15: (pg15, 15, Input<Floating>, moder15, AFRH, afrh15, bs15, br15, odr15, idr15, pupdr15, ot15, [
                AF1: (into_af1, af1,),
            ], []),
        ],
    },
    {
        devices: [
            "stm32f303xd",
            "stm32f303xe",
        ],
        devices_except: [],
        GPIO: GPIOH,
        gpio: gpioh,
        gpio_mapped: gpiob,
        gpio_mapped_ioenr: iophen,
        gpio_mapped_iorst: iophrst,
        partially_erased_pin: PHx,
        pins: [
            PH0: (ph0, 0, Input<Floating>, moder0, AFRL, afrl0, bs0, br0, odr0, idr0, pupdr0, ot0, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PH1: (ph1, 1, Input<Floating>, moder1, AFRL, afrl1, bs1, br1, odr1, idr1, pupdr1, ot1, [
                AF1: (into_af1, af1,),
                AF2: (into_af2, af2,),
                AF12: (into_af12, af12,),
            ], []),
            PH2: (ph2, 2, Input<Floating>, moder2, AFRL, afrl2, bs2, br2, odr2, idr2, pupdr2, ot2, [
                AF1: (into_af1, af1,),
                AF12: (into_af12, af12,),
            ], []),
        ],
    },
]);
