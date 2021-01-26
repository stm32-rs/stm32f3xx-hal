use proc_macro;
use proc_macro2::Span;
use quote::{format_ident, quote};
use regex::Regex;
use syn::parse::{Error, Parse, ParseStream, Result};
use syn::{parse_macro_input, Ident, Visibility};

// fully_erased
// Should not be a macro. It should just be implemented once.
// Should GPIO also be a trait that is implemented across all crates?
//     More or less I think it already is. Just in smaller form factors such as

struct GpioPartiallyErasedParsedInfo {
    visibility: Visibility,
    port_upper_case: Ident,
    port_lower_case: Ident,
}

impl Parse for GpioPartiallyErasedParsedInfo {
    fn parse(input: ParseStream) -> Result<Self> {
        eprintln!("input: {:?}", input);
        let visibility: Visibility = input.parse()?;
        let pxx: Ident = input.parse()?;
        let pxx_string = pxx.to_string();
        let expected_pattern = Regex::new(r"^P[A-Z]x").unwrap();
        if !expected_pattern.is_match(&pxx_string) {
            return Err(Error::new(
                pxx.span(),
                format!(
                    "An identifier with the {:?} pattern was expected but {:?} was supplied",
                    expected_pattern, pxx_string
                ),
            ));
        }

        let mut pxx_iter = pxx_string.chars();
        pxx_iter.next(); // The P char
        let (port_upper_case, port_lower_case) = match pxx_iter.next() {
            Some(port_char) => (port_char.to_uppercase().to_string(), port_char.to_lowercase().to_string()),
            None => return Err(Error::new(
                // Should unreachable go here instead?
                pxx.span(),
                "An identifier was supplied with a length < 2. Was expecting an identifier with a length = 3"
            ))
        };
        let (port_upper_case, port_lower_case) = (
            Ident::new(&port_upper_case, Span::call_site()),
            Ident::new(&port_lower_case, Span::call_site()),
        );

        Ok(GpioPartiallyErasedParsedInfo {
            visibility,
            port_upper_case,
            port_lower_case,
        })
    }
}

/// gpio_partially_erased!
/// Generates a partially erased gpio pin.
/// # Example
/// ```rust
/// use gpio_macros::gpio_partially_erased;
/// gpio_partially_erased!(PAx);
/// gpio_partially_erased!(PBx);
/// //--snip--
/// gpio_partially_erased!(PFx);
/// ```
#[proc_macro]
pub fn gpio_partially_erased(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let GpioPartiallyErasedParsedInfo {
        visibility,
        port_upper_case,
        port_lower_case,
    } = parse_macro_input!(input as GpioPartiallyErasedParsedInfo);

    let gpiox = format_ident!("gpio{}", port_lower_case.clone());
    let GPIOX = format_ident!("GPIO{}", port_upper_case);

    // Point out bazaar exception for stm32f373 port d mapping.
    let gpioy = match port_lower_case.to_string() {
        "a" => Ident::new("gpioa", Span::call_site()),
        "b" => Ident::new("gpiob", Span::call_site()),
        some_letter => format_ident!("gpio{}", some_letter),
    };

    let expanded = quote! {
        #[doc = "All Pins and associated functions for GPIO Bank: `" #GPIOX "`"]
                pub mod #gpiox {
                    use core::marker::PhantomData;
                    use core::convert::Infallible;

                    use crate::hal::digital::v2::OutputPin;
                    #[cfg(feature = "unproven")]
                    use crate::hal::digital::v2::InputPin;
                    #[cfg(feature = "unproven")]
                    use crate::hal::digital::v2::StatefulOutputPin;
                    #[cfg(feature = "unproven")]
                    use crate::hal::digital::v2::toggleable;
                    use crate::pac::{#gpioy, #GPIOX};
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
                        /// Pin
                        seq!(N in 

                        )
                         pub $pxi: $PXi<$MODE>,
                        
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
                }
    };

    proc_macro::TokenStream::from(expanded)
}

struct GpioPinParsedInfo {
    visibility: Visibility,
    port: Ident,
    num: Ident,
}

// gpio_pin!
// Generates the code for a gpio with a pin number
// # Example
// ```rust
// gpio!(PA1);
// gpio!(PA2);
// // --snip--
// gpio!(PF10)
// ```
