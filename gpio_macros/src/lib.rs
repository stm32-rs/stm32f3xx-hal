#![allow(non_snake_case)]
use proc_macro;
use proc_macro2::Span;
use quote::{format_ident, quote};
use regex::Regex;
use syn::parse::{Error, Parse, ParseStream, Result};
use syn::punctuated::Punctuated;
use syn::{bracketed, parse_macro_input, Ident, LitInt, Token, Visibility};

// Point out bazaar exception for stm32f373 port d mapping.
// gpio!

fn test_input_against_regex(input: &str, pattern: &Regex) -> Result<()> {
    if !pattern.is_match(&input) {
        return Err(Error::new(
            Span::call_site(),
            format!(
                "An identifier with the {:?} pattern was expected but {:?} was supplied",
                pattern, input
            ),
        ));
    }

    Ok(())
}

/// Parsed info from invoking the `gpio_partially_erased` macro.
struct GpioPartiallyErasedParsedInfo {
    port: Ident,
}

impl Parse for GpioPartiallyErasedParsedInfo {
    fn parse(input: ParseStream) -> Result<Self> {
        let _visibility: Visibility = input.parse()?;
        // uncomment to see what you are trying to parse
        eprintln!("{:?}", _visibility);

        let PIx: Ident = input.parse()?;
        let PIx_string = PIx.to_string();
        let expected_pattern = Regex::new(r"^P[A-H]x$").unwrap();
        test_input_against_regex(&PIx_string, &expected_pattern)?;
        let mut PIx_iter = PIx_string.chars();
        PIx_iter.next(); // The P char
        let port = PIx_iter.next().expect("A regex check failed").to_string();
        let port = Ident::new(&port, Span::call_site());

        Ok(Self { port })
    }
}

/// gpio_partially_erased!
/// Generates a partially erased gpio pin.
/// # Usage
/// ```ignore
/// use gpio_macros::gpio_partially_erased;
/// gpio_partially_erased!(PAx);
/// gpio_partially_erased!(PBx);
/// //--snip--
/// gpio_partially_erased!(PFx);
/// ```
#[proc_macro]
pub fn gpio_partially_erased(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let GpioPartiallyErasedParsedInfo { port } =
        parse_macro_input!(input as GpioPartiallyErasedParsedInfo);

    let PIx = format_ident!("P{}x", port);
    let GPIOX = format_ident!("GPIO{}", port);

    let expanded = quote! {
        /// Partially erased pin
        pub struct #PIx<MODE> {
            i: u8,
            _mode: PhantomData<MODE>,
        }

        impl<MODE> #PIx<MODE> {
            /// Erases the port letter from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> PXx<MODE> {
                PXx {
                    i: self.i,
                    gpio: Gpio::#GPIOX,
                    _mode: self._mode,
                }
            }
        }

        impl<MODE> OutputPin for #PIx<Output<MODE>> {
            type Error = Infallible;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe, write) atomic write to a stateless register
                unsafe { (*#GPIOX::ptr()).bsrr.write(|w| w.bits(1 << self.i)) }
                Ok(())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe, write) atomic write to a stateless register
                unsafe { (*#GPIOX::ptr()).bsrr.write(|w| w.bits(1 << (16 + self.i))) }
                Ok(())
            }
        }

        #[cfg(feature = "unproven")]
        impl<MODE> InputPin for #PIx<Input<MODE>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_low()?)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe { (*#GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
            }
        }

        #[cfg(feature = "unproven")]
        impl InputPin for #PIx<Output<OpenDrain>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_low()?)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe { (*#GPIOX::ptr()).idr.read().bits() & (1 << self.i) == 0 })
            }
        }

        #[cfg(feature = "unproven")]
        impl<MODE> StatefulOutputPin for #PIx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|b| !b)
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read with no side effects
                Ok(unsafe { (*#GPIOX::ptr()).odr.read().bits() & (1 << self.i) == 0 })
            }
        }

        #[cfg(feature = "unproven")]
        impl<MODE> toggleable::Default for #PIx<Output<MODE>> {}
    };

    proc_macro::TokenStream::from(expanded)
}

/// Holds relevant information that is parsed from the
/// invocation of the gpio pin macro
struct GpioPinParsedInfo {
    port: Ident,
    pin: u8,
    alternate_functions: Punctuated<LitInt, Token![,]>,
}

impl Parse for GpioPinParsedInfo {
    fn parse(input: ParseStream) -> Result<Self> {
        let _visibility: Visibility = input.parse()?;
        // uncomment to debug what you are parsing
        eprintln!("{:?}", _visibility);
        let PIi: Ident = input.parse()?;
        let PIi = PIi.to_string();
        let expected_pattern = Regex::new(r"^P[A-H][0-9]{1,2}$").unwrap();
        test_input_against_regex(&PIi, &expected_pattern)?;
        let mut PIi_iter = PIi.chars();
        PIi_iter.next(); // P

        let port = PIi_iter
            .next()
            .expect("A regex pattern check failed")
            .to_string();
        let port = Ident::new(&port, Span::call_site());

        let pin = PIi_iter
            .collect::<String>()
            .parse::<u8>()
            .expect("A regex pattern check failed");

        let _comma: Token![,] = input.parse()?;
        let _af: Ident = input.parse()?;
        let _collon: Token![:] = input.parse()?;

        let alternate_functions;
        bracketed!(alternate_functions in input);
        let alternate_functions: Punctuated<LitInt, Token![,]> =
            Punctuated::parse_terminated(&alternate_functions)?;

        Ok(Self {
            port,
            pin,
            alternate_functions,
        })
    }
}

/// Generates the code for a gpio with a pin number
/// # Usage
/// ```ignore
/// gpio_pin!(PA1, af: [1, 3, 7, 8, 9, 10, 15]);
/// gpio_pin!(PA2, af: [0, 1, 3, 7, 9, 15]);
/// // --snip--
/// gpio_pin!(PF10)
/// ```
#[proc_macro]
pub fn gpio_pin(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let GpioPinParsedInfo {
        port,
        pin,
        alternate_functions,
    } = parse_macro_input!(input as GpioPinParsedInfo);

    let GPIOI = format_ident!("GPIO{}", port);
    let PIi = format_ident!("P{}{}", port, pin);
    let PIx = format_ident!("P{}x", port);
    let moderi = format_ident!("moder{}", pin);
    let pupdri = format_ident!("pupdr{}", pin);
    let oti = format_ident!("ot{}", pin);
    let afrli = format_ident!("afrl{}", pin);
    let bri = format_ident!("br{}", pin);
    let bsi = format_ident!("bs{}", pin);
    let odri = format_ident!("odr{}", pin);
    let idri = format_ident!("idr{}", pin);

    let afis = alternate_functions
        .iter()
        .map(|alternate_function| format_ident!("af{}", alternate_function.base10_digits()));

    let into_afis = afis.clone().map(|afi| format_ident!("into_{}", afi));

    let AFis = alternate_functions
        .iter()
        .map(|alternate_function| format_ident!("AF{}", alternate_function.base10_digits()));

    let PIi_AFis_doc = AFis.clone().map(|AFi| {
        format!(
            "Configures `{}` to serve as alternate function: `{}`",
            PIi.clone(),
            AFi
        )
    });

    let PIi_doc = format!("Pin `{}`", PIi.clone());

    let expanded = quote! {
        #[doc = #PIi_doc]
        pub struct #PIi<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> #PIi<MODE> {
            #(#[doc = #PIi_AFis_doc]
            pub fn #into_afis(self, moder: &mut MODER, afr: &mut AFRL) -> #PIi<#AFis> {
                moder.moder().modify(|_, w| w.#moderi().alternate());
                afr.afr().modify(|_, w| w.#afrli().#afis());
                #PIi { _mode: PhantomData }
            })*

            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(
                self,
                moder: &mut MODER,
                pupdr: &mut PUPDR,
            ) -> #PIi<Input<Floating>> {
                moder.moder().modify(|_, w| w.#moderi().input());
                pupdr.pupdr().modify(|_, w| w.#pupdri().floating());
                #PIi { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(
                self,
                moder: &mut MODER,
                pupdr: &mut PUPDR,
            ) -> #PIi<Input<PullDown>> {
                moder.moder().modify(|_, w| w.#moderi().input());
                pupdr.pupdr().modify(|_, w| w.#pupdri().pull_down());
                #PIi { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(
                self,
                moder: &mut MODER,
                pupdr: &mut PUPDR,
            ) -> #PIi<Input<PullUp>> {
                moder.moder().modify(|_, w| w.#moderi().input());
                pupdr.pupdr().modify(|_, w| w.#pupdri().pull_up());
                #PIi { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(
                self,
                moder: &mut MODER,
                otyper: &mut OTYPER,
            ) -> #PIi<Output<OpenDrain>> {
                moder.moder().modify(|_, w| w.#moderi().output());
                otyper.otyper().modify(|_, w| w.#oti().open_drain());
                #PIi { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(
                self,
                moder: &mut MODER,
                otyper: &mut OTYPER,
            ) -> #PIi<Output<PushPull>> {
                moder.moder().modify(|_, w| w.#moderi().output());
                otyper.otyper().modify(|_, w| w.#oti().push_pull());
                #PIi { _mode: PhantomData }
            }
            /// Configures the pin to operate as analog, with disabled schmitt trigger.
            /// This mode is suitable when the pin is connected to the DAC or ADC.
            pub fn into_analog(self, moder: &mut MODER, pupdr: &mut PUPDR) -> #PIi<Analog> {
                moder.moder().modify(|_, w| w.#moderi().analog());
                pupdr.pupdr().modify(|_, w| w.#pupdri().floating());
                #PIi { _mode: PhantomData }
            }
        }
        impl #PIi<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, pupdr: &mut PUPDR, on: bool) {
                pupdr.pupdr().modify(|_, w| {
                    if on {
                        w.#pupdri().pull_up()
                    } else {
                        w.#pupdri().floating()
                    }
                });
            }
        }
        impl<MODE> #PIi<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> #PIx<Output<MODE>> {
                #PIx {
                    i: #pin,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> #PIi<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> #PIx<Input<MODE>> {
                #PIx {
                    i: #pin,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> OutputPin for #PIi<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                unsafe { (*#GPIOI::ptr()).bsrr.write(|w| w.#bsi().set()) }
                Ok(())
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                unsafe { (*#GPIOI::ptr()).bsrr.write(|w| w.#bri().reset()) }
                Ok(())
            }
        }
        #[cfg(feature = "unproven")]
        impl<MODE> InputPin for #PIi<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_low()?)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*#GPIOI::ptr()).idr.read().#idri().is_low() })
            }
        }
        #[cfg(feature = "unproven")]
        impl InputPin for #PIi<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_low()?)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*#GPIOI::ptr()).idr.read().#idri().is_low() })
            }
        }
        #[cfg(feature = "unproven")]
        impl<MODE> StatefulOutputPin for #PIi<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|b| !b)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*#GPIOI::ptr()).odr.read().#odri().is_low() })
            }
        }
        #[cfg(feature = "unproven")]
        impl<MODE> toggleable::Default for #PIi<Output<MODE>> {}
    };

    eprintln!("{}", expanded);

    proc_macro::TokenStream::from(expanded)
}

/// Holds relevant information that is parsed from the gpio! macro invocation
struct GpioParsedInfo {
    port_lower: String,
    port_upper: String,
    pins_to_alternate_functions: Vec<(u8, Vec<u8>)>,
}

impl Parse for GpioParsedInfo {
    fn parse(input: ParseStream) -> Result<Self> {
        let _port: Ident = input.parse()?;
        let _collon: Token![:] = input.parse()?;
        let port: Ident = input.parse()?;
        let expected_port = Regex::new("^[a-h]$").unwrap();
        test_input_against_regex(&port.to_string(), &expected_port)?;
        let port_lower = port.to_string();
        let port_upper = port.to_string().to_uppercase();

        let _comma: Token![,] = input.parse()?;
        let _pins_ident: Ident = input.parse()?;
        let _collon: Token![:] = input.parse()?;
        let bracketed_content;
        bracketed!(bracketed_content in input);
        let pin_to_alternate_functions_parser = |input: ParseStream<'_>| {
            let pin: u8 = input.parse::<LitInt>()?.base10_parse::<u8>()?;

            let _arrow: Token![=>] = input.parse()?;
            let _af: Ident = input.parse()?;
            let _collon: Token![:] = input.parse()?;
            let alternate_functions;
            bracketed!(alternate_functions in input);
            let alternate_functions: Punctuated<LitInt, Token![,]> =
                Punctuated::parse_separated_nonempty(&alternate_functions)?; // being empty may be fine
            let alternate_functions: Vec<u8> = alternate_functions
                .iter()
                .map(|alternate_function| {
                    alternate_function
                        .base10_parse::<u8>()
                        .expect("unable to parse an integer for an alternate function")
                })
                .collect();

            Ok((pin, alternate_functions))
        };

        let pins_to_alternate_functions: Punctuated<(u8, Vec<u8>), Token![,]> =
            Punctuated::parse_separated_nonempty_with(
                &bracketed_content,
                pin_to_alternate_functions_parser,
            )?;

        //let pins_to_alternate_functions: syn::ExprArray = input.parse()?;
        let pins_to_alternate_functions: Vec<(u8, Vec<u8>)> =
            pins_to_alternate_functions.iter().cloned().collect();

        Ok(Self {
            port_lower,
            port_upper,
            pins_to_alternate_functions,
        })
    }
}

/// Creates the structs and implementations of gpio pins
/// # Usage
/// ```rust
/// gpio!(port: a, pins: [
///    0 => af: [1, 3, 7, 8, 9, 10, 15],
///    1 => af: [0, 1, 3, 7, 9, 15],
///    2 => af: [1, 3, 7, 8, 9, 15],
///    3 => af: [1, 3, 7, 9, 15],
///    4 => af: [2, 3, 5, 6, 7, 15],
///    5 => af: [1, 3, 5, 15],
///    6 => af: [1, 2, 3, 4, 5, 6, 8, 15],
///    7 => af: [1, 2, 3, 4, 5, 6, 8, 15],
///    8 => af: [0, 4, 5, 6, 7, 8, 10, 15],
///    9 => af: [3, 4, 5, 6, 7, 8, 9, 10, 15],
///    10 => af: [1, 3, 4, 6, 7, 8, 10, 11, 15],
///    11 => af: [6, 7, 8, 9, 10, 11, 12, 14, 15],
///    12 => af: [1, 6, 7, 8, 9, 10, 11, 14, 15],
///    13 => af: [0, 1, 3, 5, 7, 10, 15],
///    14 => af: [0, 3, 4, 5, 6, 7, 15],
///    15 => af: [0, 1, 2, 4, 5, 6, 7, 9, 15],
///])
/// ```
#[proc_macro]
pub fn gpio(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let GpioParsedInfo {
        port_lower,
        port_upper,
        pins_to_alternate_functions,
    } = parse_macro_input!(input as GpioParsedInfo);

    let GPIOI = format_ident!("GPIO{}", port_upper);
    let gpioi = format_ident!("gpio{}", port_lower);
    let pini: Vec<u8> = pins_to_alternate_functions
        .iter()
        .cloned()
        .map(|(pin, _alternate_functions)| pin)
        .collect();
    let alternate_functionsi = pins_to_alternate_functions
        .iter()
        .map(|(_pin, alternate_functions)| alternate_functions);
    let PIx = format_ident!("P{}x", port_upper);
    let PIis: Vec<Ident> = pini
        .iter()
        .map(|pin| format_ident!("P{}{}", port_upper, pin))
        .collect();
    let piis : Vec<Ident> = pini
        .iter()
        .map(|pin| format_ident!("p{}{}", port_lower, pin))
        .collect();
    // let AFIs = alternate_functionsi.clone().map(|alternate_functions| {
    //     alternate_functions
    //         .iter()
    //         .map(|alternate_function| format_ident!("AF{}", alternate_function))
    // });
    let resets = pini.iter().map(|pin| match (port_lower.as_str(), pin) {
        ("a", _pin @ 13..=15) => format_ident!("AF0"),
        ("b", _pin @ 3..=4) => format_ident!("AF0"),
        _ => format_ident!("INPUT<FLOATING>"),
    });
    let iopien = format_ident!("iop{}en", port_lower);
    let iopirst = format_ident!("iop{}rst", port_lower);

    // Is this a bug?
    let gpio_mapped = match port_lower.as_str() {
        "a" => format_ident!("gpioa"),
        "b" => format_ident!("gpiob"),
        _ => format_ident!("gpioc"),
        // What about the mapping for gpio-f373?
        // Should the alternate mapping be d -> d?
        // This is contrary to every other board.
    };

    let expanded = quote! {
        /// "All Pins and associated functions for GPIO Bank: `GPIOA`
        pub mod #gpioi {
            use core::marker::PhantomData;
            use core::convert::Infallible;
            use crate::hal::digital::v2::OutputPin;
            #[cfg(feature = "unproven")]
            use crate::hal::digital::v2::InputPin;
            #[cfg(feature = "unproven")]
            use crate::hal::digital::v2::StatefulOutputPin;
            #[cfg(feature = "unproven")]
            use crate::hal::digital::v2::toggleable;
            use crate::pac::{#gpioi, #GPIOI};
            use crate::rcc::AHB;
            #[allow(unused_imports)]
            use super::{
                AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15,
            };
            use super::{
                Floating, GpioExt, Input, OpenDrain, Output, Analog, PullDown, PullUp, PushPull, PXx,
                Gpio,
            };
            /// " GPIO parts
            pub struct Parts {
                /// " Opaque AFRH register
                pub afrh: AFRH,
                /// " Opaque AFRL register
                pub afrl: AFRL,
                /// " Opaque MODER register
                pub moder: MODER,
                /// " Opaque OTYPER register
                pub otyper: OTYPER,
                /// " Opaque PUPDR register
                pub pupdr: PUPDR,
                /// Pin
                #(pub #piis: #PIis<#resets>),*

            }
            impl GpioExt for #GPIOI {
                type Parts = Parts;
                fn split(self, ahb: &mut AHB) -> Parts {
                    ahb.enr().modify(|_, w| w.#iopien().set_bit());
                    ahb.rstr().modify(|_, w| w.#iopirst().set_bit());
                    ahb.rstr().modify(|_, w| w.#iopirst().clear_bit());
                    Parts {
                        afrh: AFRH { _0: () },
                        afrl: AFRL { _0: () },
                        moder: MODER { _0: () },
                        otyper: OTYPER { _0: () },
                        pupdr: PUPDR { _0: () },
                        #(#piis: #PIis { _mode: PhantomData }),*
                    }
                }
            }
            /// " Opaque AFRL register
            pub struct AFRL {
                _0: (),
            }
            impl AFRL {
                #[allow(dead_code)]
                pub(crate) fn afr(&mut self) -> &#gpio_mapped::AFRL {
                    unsafe { &(*#GPIOI::ptr()).afrl }
                }
            }
            /// " Opaque AFRH register
            pub struct AFRH {
                _0: (),
            }
            impl AFRH {
                #[allow(dead_code)]
                pub(crate) fn afr(&mut self) -> &#gpio_mapped::AFRH {
                    unsafe { &(*#GPIOI::ptr()).afrh }
                }
            }
            /// " Opaque MODER register
            pub struct MODER {
                _0: (),
            }
            impl MODER {
                pub(crate) fn moder(&mut self) -> &#gpio_mapped::MODER {
                    unsafe { &(*#GPIOI::ptr()).moder }
                }
            }
            /// " Opaque OTYPER register
            pub struct OTYPER {
                _0: (),
            }
            impl OTYPER {
                pub(crate) fn otyper(&mut self) -> &#gpio_mapped::OTYPER {
                    unsafe { &(*#GPIOI::ptr()).otyper }
                }
            }
            /// " Opaque PUPDR register
            pub struct PUPDR {
                _0: (),
            }
            impl PUPDR {
                pub(crate) fn pupdr(&mut self) -> &#gpio_mapped::PUPDR {
                    unsafe { &(*#GPIOI::ptr()).pupdr }
                }
            }

            gpio_partially_erased!(#PIx);
            #(gpio_pin!(#PIis, af: #alternate_functionsi));* // Currently have a compiler error on this line.
        }
    };

    proc_macro::TokenStream::from(expanded)
}
