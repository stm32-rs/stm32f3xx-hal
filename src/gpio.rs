//! # General Purpose Input / Output
//!
//! To use the GPIO pins, you first need to configure the GPIO port (GPIOA, GPIOB, ...) that you
//! are interested in. This is done using the [`GpioExt::split`] function.
//!
//! ```
//! let dp = pac::Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//!
//! let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//! ```
//!
//! The resulting [Parts](gpioa::Parts) struct contains one field for each pin, as well as some
//! shared registers. Every pin type is a specialized version of generic [pin](Pin) struct.
//!
//! To use a pin, first use the relevant `into_...` method of the [pin](Pin).
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
//! ## Pin Configuration
//!
//! ### Mode
//!
//! Each GPIO pin can be set to various modes by corresponding `into_...` method:
//!
//! - **Input**: The output buffer is disabled and the schmitt trigger input is activated
//! - **Output**: Both the output buffer and the schmitt trigger input is enabled
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!     mode. Can be used as an input in the `open` configuration
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals. The schmitt
//! trigger input is activated. The Output buffer is automatically enabled and disabled by
//! peripherals. Output behavior is same as the output mode
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!     mode
//! - **Analog**: Pin mode required for ADC, DAC, OPAMP, and COMP peripherals. It is also suitable
//! for minimize energy consumption as the output buffer and the schmitt trigger input is disabled
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! ### Output Speed
//!
//! Output speed (slew rate) for each pin is selectable from low, medium, and high by calling
//! [`set_speed`](Pin::set_speed) method. Refer to the device datasheet for specifications for each
//!  speed.
//!
//! ### Internal Resistor
//!
//! Weak internal pull-up and pull-down resistors for each pin is configurable by calling
//! [`set_internal_resistor`](Pin::set_internal_resistor) method. `into_..._input` methods are also
//! available for convenience.
//!
//! [InputPin]: embedded_hal::digital::v2::InputPin
//! [OutputPin]: embedded_hal::digital::v2::OutputPin
//! [examples/toggle.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.0/examples/toggle.rs
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the closure based `with_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode

#![allow(missing_docs)]

use core::marker::PhantomData;

use crate::pac::{Interrupt, EXTI};
use crate::rcc::AHB;
use crate::Switch;

mod convert;
use convert::PinMode;
mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};
mod erased;
pub use erased::{EPin, ErasedPin};
mod dynamic;
pub use dynamic::{Dynamic, DynamicPin};
mod hal_02;

pub use embedded_hal::digital::v2::PinState;

use core::fmt;

/// A filler pin type
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NoPin;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB) -> Self::Parts;
}

pub trait PinExt {
    type Mode;
    /// Return pin number
    fn pin_id(&self) -> u8;
    /// Return port number
    fn port_id(&self) -> u8;
}

/// Some alternate mode (type state)
pub struct Alternate<const A: u8, Otype = PushPull>(PhantomData<Otype>);

/// Input mode (type state)
pub struct Input;

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    /// Floating
    None = 0,
    /// Pulled up
    Up = 1,
    /// Pulled down
    Down = 2,
}

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Output mode (type state)
pub struct Output<MODE = PushPull> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

/// Analog mode (type state)
pub struct Analog;

pub type Debugger = Alternate<0, PushPull>;

mod sealed {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
    /// Marker trait for readable pin modes
    pub trait Readable {}
    /// Marker trait for slew rate configurable pin modes
    pub trait OutputSpeed {}
    /// Marker trait for active pin modes
    pub trait Active {}
    /// Marker trait for all pin modes except alternate
    pub trait NotAlt {}
}

impl sealed::Readable for Input {}
impl sealed::Readable for Output<OpenDrain> {}
impl sealed::Active for Input {}
impl<Otype> sealed::OutputSpeed for Output<Otype> {}
impl<const A: u8, Otype> sealed::OutputSpeed for Alternate<A, Otype> {}
impl<Otype> sealed::Active for Output<Otype> {}
impl<const A: u8, Otype> sealed::Active for Alternate<A, Otype> {}
impl sealed::NotAlt for Input {}
impl<Otype> sealed::NotAlt for Output<Otype> {}
impl sealed::NotAlt for Analog {}

/// GPIO Pin speed selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Speed {
    Low = 0,
    Medium = 1,
    High = 3,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

use sealed::Interruptable;
impl<MODE> Interruptable for Output<MODE> {}
impl Interruptable for Input {}

/// Opaque MODER register
pub struct MODER<const P: char>(());

/// Opaque OTYPER register
pub struct OTYPER<const P: char>(());

/// Opaque OSPEEDR register
pub struct OSPEEDR<const P: char>(());

/// Opaque PUPDR register
pub struct PUPDR<const P: char>(());

/// Opaque AFR register
pub struct Afr<const P: char, const H: bool>(());

/// Represents high or low configuration register
pub trait HL {
    /// Configuration register associated to pin
    type Afr;
}

/// Marker trait for pins with alternate function `A` mapping
pub trait IntoAf<const A: u8>: HL {}

macro_rules! cr {
    ($high:literal: [$($i:literal),+]) => {
        $(
            impl<const P: char, MODE> HL for Pin<P, $i, MODE> {
                type Afr = Afr<P, $high>;
            }
        )+
    }
}

cr!(false: [0, 1, 2, 3, 4, 5, 6, 7]);
cr!(true: [8, 9, 10, 11, 12, 13, 14, 15]);

macro_rules! af {
    ($i:literal, $AFi:ident) => {
        #[doc = concat!("Alternate function ", $i, " (type state)" )]
        pub type $AFi<Otype = PushPull> = Alternate<$i, Otype>;
    };

    ([$($i:literal),+ $(,)?]) => {
        paste::paste! {
            $(
                af!($i, [<AF $i>]);
            )+
        }
    };
}

af!([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);

/// Return an EXTI register for the current CPU
#[cfg(feature = "svd-f373")]
macro_rules! reg_for_cpu {
    ($exti:expr, $xr:ident) => {
        $exti.$xr
    };
}

/// Return an EXTI register for the current CPU
#[cfg(not(feature = "svd-f373"))]
macro_rules! reg_for_cpu {
    ($exti:expr, $xr:ident) => {
        paste::paste! {
            $exti.[<$xr 1>]
        }
    };
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Interruptable,
{
    /// NVIC interrupt number of interrupt from this pin
    ///
    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
    /// This is also useful for all other [`cortex_m::peripheral::NVIC`] functions.
    // TODO(Sh3rm4n): It would be cool to have this either const or have a const function.
    // But this is currenlty not possible, because index() is runtime defined.
    pub fn interrupt(&self) -> Interrupt {
        match N {
            0 => Interrupt::EXTI0,
            1 => Interrupt::EXTI1,
            #[cfg(feature = "svd-f373")]
            2 => Interrupt::EXTI2_TS,
            #[cfg(not(feature = "svd-f373"))]
            2 => Interrupt::EXTI2_TSC,
            3 => Interrupt::EXTI3,
            4 => Interrupt::EXTI4,
            #[cfg(feature = "svd-f373")]
            5..=9 => Interrupt::EXTI5_9,
            #[cfg(not(feature = "svd-f373"))]
            5..=9 => Interrupt::EXTI9_5,
            10..=15 => Interrupt::EXTI15_10,
            _ => crate::unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge, or both
    pub fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        const BITWIDTH: u8 = 1;
        let (rise, fall) = match edge {
            Edge::Rising => (true as u32, false as u32),
            Edge::Falling => (false as u32, true as u32),
            Edge::RisingFalling => (true as u32, true as u32),
        };
        // SAFETY: Unguarded write to the register, but behind a &mut
        unsafe {
            crate::modify_at!(reg_for_cpu!(exti, rtsr), BITWIDTH, N, rise);
            crate::modify_at!(reg_for_cpu!(exti, ftsr), BITWIDTH, N, fall);
        }
    }

    /// Configure external interrupts from this pin
    ///
    /// # Note
    ///
    /// Remeber to also configure the interrupt pin on
    /// the SysCfg site, with [`crate::syscfg::SysCfg::select_exti_interrupt_source()`]
    pub fn configure_interrupt(&mut self, exti: &mut EXTI, enable: impl Into<Switch>) {
        const BITWIDTH: u8 = 1;

        let enable: Switch = enable.into();
        let enable: bool = enable.into();

        let value = u32::from(enable);
        // SAFETY: Unguarded write to the register, but behind a &mut
        unsafe { crate::modify_at!(reg_for_cpu!(exti, imr), BITWIDTH, N, value) };
    }

    /// Enable external interrupts from this pin
    ///
    /// # Note
    ///
    /// Remeber to also configure the interrupt pin on
    /// the SysCfg site, with [`crate::syscfg::SysCfg::select_exti_interrupt_source()`]
    pub fn enable_interrupt(&mut self, exti: &mut EXTI) {
        self.configure_interrupt(exti, Switch::On)
    }

    /// Disable external interrupts from this pin
    pub fn disable_interrupt(&mut self, exti: &mut EXTI) {
        self.configure_interrupt(exti, Switch::Off)
    }

    /// Clear the interrupt pending bit for this pin
    pub fn clear_interrupt(&mut self) {
        // SAFETY: Atomic write to register without side-effects.
        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).write(|w| w.bits(1 << N)) };
    }

    /// Reads the interrupt pending bit for this pin
    pub fn is_interrupt_pending(&self) -> bool {
        // SAFETY: Atomic write to register without side-effects.
        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).read().bits() & (1 << N) != 0 }
    }
}

/// Generic pin type
///
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
pub struct Pin<const P: char, const N: u8, MODE = Input> {
    _mode: PhantomData<MODE>,
}
impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

impl<const P: char, const N: u8, MODE> fmt::Debug for Pin<P, N, MODE> {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_fmt(format_args!(
            "P{}{}<{}>",
            P,
            N,
            crate::stripped_type_name::<MODE>()
        ))
    }
}

#[cfg(feature = "defmt")]
impl<const P: char, const N: u8, MODE> defmt::Format for Pin<P, N, MODE> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "P{}{}<{}>", P, N, crate::stripped_type_name::<MODE>());
    }
}

impl<const P: char, const N: u8, MODE> PinExt for Pin<P, N, MODE> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        N
    }
    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - b'A'
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: sealed::OutputSpeed,
{
    /// Set pin speed
    pub fn set_speed(&mut self, _ospeedr: &mut OSPEEDR<P>, speed: Speed) {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr())
                .ospeedr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset)));
        }
    }

    /// Set pin speed
    pub fn speed(mut self, ospeedr: &mut OSPEEDR<P>, speed: Speed) -> Self {
        self.set_speed(ospeedr, speed);
        self
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: sealed::Active,
{
    /// Set the internal pull-up and pull-down resistor
    pub fn set_internal_resistor(&mut self, _pupdr: &mut PUPDR<P>, resistor: Pull) {
        let offset = 2 * { N };
        let value = resistor as u32;
        unsafe {
            (*Gpio::<P>::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)))
        };
    }

    /// Set the internal pull-up and pull-down resistor
    pub fn internal_resistor(mut self, pupdr: &mut PUPDR<P>, resistor: Pull) -> Self {
        self.set_internal_resistor(pupdr, resistor);
        self
    }

    /// Enables / disables the internal pull up
    pub fn internal_pull_up(self, pupdr: &mut PUPDR<P>, on: bool) -> Self {
        if on {
            self.internal_resistor(pupdr, Pull::Up)
        } else {
            self.internal_resistor(pupdr, Pull::None)
        }
    }

    /// Enables / disables the internal pull down
    pub fn internal_pull_down(self, pupdr: &mut PUPDR<P>, on: bool) -> Self {
        if on {
            self.internal_resistor(pupdr, Pull::Down)
        } else {
            self.internal_resistor(pupdr, Pull::None)
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase_number(self) -> PEPin<P, MODE> {
        PEPin::new(N)
    }

    /// Erases the pin number and the port from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase(self) -> EPin<MODE> {
        EPin::new(P as u8 - b'A', N)
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Set the output of the pin regardless of its mode.
    /// Primarily used to set the output value of the pin
    /// before changing its mode to an output to avoid
    /// a short spike of an incorrect value
    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }
    #[inline(always)]
    fn _set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N)) }
    }
    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << (16 + N))) }
    }
    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).odr.read().bits() & (1 << N) == 0 }
    }
    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << N) == 0 }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    #[inline(always)]
    pub fn set_high(&mut self) {
        self._set_high()
    }

    #[inline(always)]
    pub fn set_low(&mut self) {
        self._set_low()
    }

    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self.is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    #[inline(always)]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    #[inline(always)]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }

    #[inline(always)]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: sealed::Readable,
{
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline(always)]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PEPin:ident, $port_id:expr, $PXn:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, [$($A:literal),*] $(, $MODE:ty)?),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use crate::pac::$GPIOX;
            use crate::rcc::{Enable, Reset, AHB};
            use super::{Afr, Input, MODER, OTYPER, OSPEEDR, PUPDR, IntoAf};

            /// GPIO parts
            pub struct Parts {
                /// Opaque AFRH register
                pub afrh: Afr<$port_id, true>,
                /// Opaque AFRL register
                pub afrl: Afr<$port_id, false>,
                /// Opaque MODER register
                pub moder: MODER<$port_id>,
                /// Opaque OTYPER register
                pub otyper: OTYPER<$port_id>,
                /// Opaque OSPEEDR register
                pub ospeedr: OSPEEDR<$port_id>,
                /// Opaque PUPDR register
                pub pupdr: PUPDR<$port_id>,
                $(
                    /// Pin
                    pub $pxi: $PXi $(<$MODE>)?,
                )+
            }

            impl super::GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB) -> Parts {
                    <$GPIOX>::enable(ahb);
                    <$GPIOX>::reset(ahb);

                    Parts {
                        afrh: Afr(()),
                        afrl: Afr(()),
                        moder: MODER(()),
                        otyper: OTYPER(()),
                        ospeedr: OSPEEDR(()),
                        pupdr: PUPDR(()),
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }

            pub type $PXn<MODE> = super::PEPin<$port_id, MODE>;

            $(
                pub type $PXi<MODE = Input> = super::Pin<$port_id, $i, MODE>;

                $(
                    impl<MODE> IntoAf<$A> for $PXi<MODE> { }
                )*
            )+

        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

#[cfg(feature = "gpio-f302")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 3, 7, 15]),
    PA1: (pa1, 1, [0, 1, 3, 7, 9, 15]),
    PA2: (pa2, 2, [1, 3, 7, 8, 9, 15]),
    PA3: (pa3, 3, [1, 3, 7, 9, 15]),
    PA4: (pa4, 4, [3, 6, 7, 15]),
    PA5: (pa5, 5, [1, 3, 15]),
    PA6: (pa6, 6, [1, 3, 6, 15]),
    PA7: (pa7, 7, [1, 3, 6, 15]),
    PA8: (pa8, 8, [0, 3, 4, 5, 6, 7, 15]),
    PA9: (pa9, 9, [2, 3, 4, 5, 6, 7, 9, 10, 15]),
    PA10: (pa10, 10, [1, 3, 4, 5, 6, 7, 8, 10, 15]),
    PA11: (pa11, 11, [5, 6, 7, 9, 11, 12, 15]),
    PA12: (pa12, 12, [1, 5, 6, 7, 8, 9, 11, 15]),
    PA13: (pa13, 13, [0, 1, 3, 5, 7, 15], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 3, 4, 6, 7, 15], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 3, 4, 6, 7, 9, 15], super::Debugger), // JTDI, PullUp
]);

#[cfg(feature = "gpio-f302")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [3, 6, 15]),
    PB1: (pb1, 1, [3, 6, 8, 15]),
    PB2: (pb2, 2, [3, 15]),
    PB3: (pb3, 3, [0, 1, 3, 6, 7, 15], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 3, 6, 7, 10, 15], super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, [1, 4, 6, 7, 8, 10, 15]),
    PB6: (pb6, 6, [1, 3, 4, 7, 15]),
    PB7: (pb7, 7, [1, 3, 4, 7, 15]),
    PB8: (pb8, 8, [1, 3, 4, 7, 9, 12, 15]),
    PB9: (pb9, 9, [1, 4, 6, 7, 8, 9, 15]),
    PB10: (pb10, 10, [1, 3, 7, 15]),
    PB11: (pb11, 11, [1, 3, 7, 15]),
    PB12: (pb12, 12, [3, 4, 5, 6, 7, 15]),
    PB13: (pb13, 13, [3, 5, 6, 7, 15]),
    PB14: (pb14, 14, [1, 3, 5, 6, 7, 15]),
    PB15: (pb15, 15, [0, 1, 2, 4, 5, 15]),
]);

#[cfg(feature = "gpio-f302")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 2]),
    PC1: (pc1, 1, [1, 2]),
    PC2: (pc2, 2, [1, 2]),
    PC3: (pc3, 3, [1, 2, 6]),
    PC4: (pc4, 4, [1, 2, 7]),
    PC5: (pc5, 5, [1, 2, 3, 7]),
    PC6: (pc6, 6, [1, 6, 7]),
    PC7: (pc7, 7, [1, 6]),
    PC8: (pc8, 8, [1]),
    PC9: (pc9, 9, [1, 3, 5]),
    PC10: (pc10, 10, [1, 6, 7]),
    PC11: (pc11, 11, [1, 6, 7]),
    PC12: (pc12, 12, [1, 6, 7]),
    PC13: (pc13, 13, [4]),
    PC14: (pc14, 14, []),
    PC15: (pc15, 15, []),
]);

#[cfg(feature = "gpio-f302")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD2: (pd2, 2, [1]),
]);

#[cfg(feature = "gpio-f302")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 5, 6]),
    PF1: (pf1, 1, [4, 5]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 3, 7, 8, 9, 10, 15]),
    PA1: (pa1, 1, [0, 1, 3, 7, 9, 15]),
    PA2: (pa2, 2, [1, 3, 7, 8, 9, 15]),
    PA3: (pa3, 3, [1, 3, 7, 9, 15]),
    PA4: (pa4, 4, [2, 3, 5, 6, 7, 15]),
    PA5: (pa5, 5, [1, 3, 5, 15]),
    PA6: (pa6, 6, [1, 2, 3, 4, 5, 6, 8, 15]),
    PA7: (pa7, 7, [1, 2, 3, 4, 5, 6, 15]),
    PA8: (pa8, 8, [0, 3, 4, 5, 6, 7, 8, 10, 15]),
    PA9: (pa9, 9, [2, 3, 4, 5, 6, 7, 8, 9, 10, 15]),
    PA10: (pa10, 10, [1, 3, 4, 5, 6, 7, 8, 10, 11, 15]),
    PA11: (pa11, 11, [5, 6, 7, 8, 9, 10, 11, 12, 15]),
    PA12: (pa12, 12, [1, 5, 6, 7, 8, 9, 10, 11, 15]),
    PA13: (pa13, 13, [0, 1, 3, 5, 7, 10, 15], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 3, 4, 5, 6, 7, 15], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 2, 3, 4, 5, 6, 7, 9, 15], super::Debugger), // JTDI, PullUp
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [2, 3, 4, 6, 15]),
    PB1: (pb1, 1, [2, 3, 4, 6, 8, 15]),
    PB2: (pb2, 2, [3, 15]),
    PB3: (pb3, 3, [0, 1, 2, 3, 4, 5, 6, 7, 10, 15], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 3, 4, 5, 6, 7, 10, 15], super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, 10, 15]),
    PB6: (pb6, 6, [1, 2, 3, 4, 5, 6, 7, 10, 15]),
    PB7: (pb7, 7, [1, 2, 3, 4, 5, 7, 10, 12, 15]),
    PB8: (pb8, 8, [1, 2, 3, 4, 7, 8, 9, 10, 12, 15]),
    PB9: (pb9, 9, [1, 2, 4, 6, 7, 8, 9, 10, 15]),
    PB10: (pb10, 10, [1, 3, 7, 15]),
    PB11: (pb11, 11, [1, 3, 7, 15]),
    PB12: (pb12, 12, [3, 4, 5, 6, 7, 15]),
    PB13: (pb13, 13, [3, 5, 6, 7, 15]),
    PB14: (pb14, 14, [1, 3, 5, 6, 7, 15]),
    PB15: (pb15, 15, [0, 1, 2, 4, 5, 15]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 2]),
    PC1: (pc1, 1, [1, 2]),
    PC2: (pc2, 2, [1, 2, 3]),
    PC3: (pc3, 3, [1, 2, 6]),
    PC4: (pc4, 4, [1, 2, 7]),
    PC5: (pc5, 5, [1, 2, 3, 7]),
    PC6: (pc6, 6, [1, 2, 4, 6, 7]),
    PC7: (pc7, 7, [1, 2, 4, 6, 7]),
    PC8: (pc8, 8, [1, 2, 4, 7]),
    PC9: (pc9, 9, [1, 2, 3, 4, 5, 6]),
    PC10: (pc10, 10, [1, 4, 5, 6, 7]),
    PC11: (pc11, 11, [1, 4, 5, 6, 7]),
    PC12: (pc12, 12, [1, 4, 5, 6, 7]),
    PC13: (pc13, 13, [1, 4]),
    PC14: (pc14, 14, [1]),
    PC15: (pc15, 15, [1]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [1, 7, 12]),
    PD1: (pd1, 1, [1, 4, 6, 7, 12]),
    PD2: (pd2, 2, [1, 2, 4, 5]),
    PD3: (pd3, 3, [1, 2, 7, 12]),
    PD4: (pd4, 4, [1, 2, 7, 12]),
    PD5: (pd5, 5, [1, 7, 12]),
    PD6: (pd6, 6, [1, 2, 7, 12]),
    PD7: (pd7, 7, [1, 2, 7, 12]),
    PD8: (pd8, 8, [1, 7, 12]),
    PD9: (pd9, 9, [1, 7, 12]),
    PD10: (pd10, 10, [1, 7, 12]),
    PD11: (pd11, 11, [1, 7, 12]),
    PD12: (pd12, 12, [1, 2, 3, 7, 12]),
    PD13: (pd13, 13, [1, 2, 3, 12]),
    PD14: (pd14, 14, [1, 2, 3, 12]),
    PD15: (pd15, 15, [1, 2, 3, 6, 12]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [1, 2, 4, 6, 7, 12]),
    PE1: (pe1, 1, [1, 4, 6, 7, 12]),
    PE2: (pe2, 2, [0, 1, 2, 3, 5, 6, 12]),
    PE3: (pe3, 3, [0, 1, 2, 3, 5, 6, 12]),
    PE4: (pe4, 4, [0, 1, 2, 3, 5, 6, 12]),
    PE5: (pe5, 5, [0, 1, 2, 3, 5, 6, 12]),
    PE6: (pe6, 6, [0, 1, 5, 6, 12]),
    PE7: (pe7, 7, [1, 2, 12]),
    PE8: (pe8, 8, [1, 2, 12]),
    PE9: (pe9, 9, [1, 2, 12]),
    PE10: (pe10, 10, [1, 2, 12]),
    PE11: (pe11, 11, [1, 2, 5, 12]),
    PE12: (pe12, 12, [1, 2, 5, 12]),
    PE13: (pe13, 13, [1, 2, 5, 12]),
    PE14: (pe14, 14, [1, 2, 5, 6, 12]),
    PE15: (pe15, 15, [1, 2, 7, 12]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [1, 4, 5, 6]),
    PF1: (pf1, 1, [1, 4, 5]),
    PF2: (pf2, 2, [1, 2, 12]),
    PF3: (pf3, 3, [1, 2, 12]),
    PF4: (pf4, 4, [1, 2, 3, 12]),
    PF5: (pf5, 5, [1, 2, 12]),
    PF6: (pf6, 6, [1, 2, 4, 7, 12]),
    PF7: (pf7, 7, [1, 2, 12]),
    PF8: (pf8, 8, [1, 2, 12]),
    PF9: (pf9, 9, [1, 2, 3, 5, 12]),
    PF10: (pf10, 10, [1, 2, 3, 5, 12]),
    PF11: (pf11, 11, [1, 2]),
    PF12: (pf12, 12, [1, 2, 12]),
    PF13: (pf13, 13, [1, 2, 12]),
    PF14: (pf14, 14, [1, 2, 12]),
    PF15: (pf15, 15, [1, 2, 12]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOG, gpiog, PG, 'G', PGn, [
    PG0: (pg0, 0, [1, 2, 12]),
    PG1: (pg1, 1, [1, 2, 12]),
    PG2: (pg2, 2, [1, 2, 12]),
    PG3: (pg3, 3, [1, 2, 12]),
    PG4: (pg4, 4, [1, 2, 12]),
    PG5: (pg5, 5, [1, 2, 12]),
    PG6: (pg6, 6, [1, 12]),
    PG7: (pg7, 7, [1, 12]),
    PG8: (pg8, 8, [1]),
    PG9: (pg9, 9, [1, 12]),
    PG10: (pg10, 10, [1, 12]),
    PG11: (pg11, 11, [1, 12]),
    PG12: (pg12, 12, [1, 12]),
    PG13: (pg13, 13, [1, 12]),
    PG14: (pg14, 14, [1, 12]),
    PG15: (pg15, 15, [1]),
]);

#[cfg(feature = "gpio-f303e")]
gpio!(GPIOH, gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [1, 2, 12]),
    PH1: (ph1, 1, [1, 2, 12]),
    PH2: (ph2, 2, [1]),
]);

#[cfg(feature = "gpio-f303")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 3, 7, 8, 9, 10, 15]),
    PA1: (pa1, 1, [0, 1, 3, 7, 9, 15]),
    PA2: (pa2, 2, [1, 3, 7, 8, 9, 15]),
    PA3: (pa3, 3, [1, 3, 7, 9, 15]),
    PA4: (pa4, 4, [2, 3, 5, 6, 7, 15]),
    PA5: (pa5, 5, [1, 3, 5, 15]),
    PA6: (pa6, 6, [1, 2, 3, 4, 5, 6, 8, 15]),
    PA7: (pa7, 7, [1, 2, 3, 4, 5, 6, 8, 15]),
    PA8: (pa8, 8, [0, 4, 5, 6, 7, 8, 10, 15]),
    PA9: (pa9, 9, [3, 4, 5, 6, 7, 8, 9, 10, 15]),
    PA10: (pa10, 10, [1, 3, 4, 6, 7, 8, 10, 11, 15]),
    PA11: (pa11, 11, [6, 7, 8, 9, 10, 11, 12, 14, 15]),
    PA12: (pa12, 12, [1, 6, 7, 8, 9, 10, 11, 14, 15]),
    PA13: (pa13, 13, [0, 1, 3, 5, 7, 10, 15], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 3, 4, 5, 6, 7, 15], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 2, 4, 5, 6, 7, 9, 15], super::Debugger), // JTDI, PullUp
]);

#[cfg(feature = "gpio-f303")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [2, 3, 4, 6, 15]),
    PB1: (pb1, 1, [2, 3, 4, 6, 8, 15]),
    PB2: (pb2, 2, [3, 15]),
    PB3: (pb3, 3, [0, 1, 2, 3, 4, 5, 6, 7, 10, 15], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 3, 4, 5, 6, 7, 10, 15], super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 10, 15]),
    PB6: (pb6, 6, [1, 2, 3, 4, 5, 6, 7, 10, 15]),
    PB7: (pb7, 7, [1, 2, 3, 4, 5, 7, 10, 15]),
    PB8: (pb8, 8, [1, 2, 3, 4, 8, 9, 10, 12, 15]),
    PB9: (pb9, 9, [1, 2, 4, 6, 8, 9, 10, 15]),
    PB10: (pb10, 10, [1, 3, 7, 15]),
    PB11: (pb11, 11, [1, 3, 7, 15]),
    PB12: (pb12, 12, [3, 4, 5, 6, 7, 15]),
    PB13: (pb13, 13, [3, 5, 6, 7, 15]),
    PB14: (pb14, 14, [1, 3, 5, 6, 7, 15]),
    PB15: (pb15, 15, [0, 1, 2, 4, 5, 15]),
]);

#[cfg(feature = "gpio-f303")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1]),
    PC1: (pc1, 1, [1]),
    PC2: (pc2, 2, [1, 3]),
    PC3: (pc3, 3, [1, 6]),
    PC4: (pc4, 4, [1, 7]),
    PC5: (pc5, 5, [1, 3, 7]),
    PC6: (pc6, 6, [1, 2, 4, 6, 7]),
    PC7: (pc7, 7, [1, 2, 4, 6, 7]),
    PC8: (pc8, 8, [1, 2, 4, 7]),
    PC9: (pc9, 9, [1, 2, 4, 5, 6]),
    PC10: (pc10, 10, [1, 4, 5, 6, 7]),
    PC11: (pc11, 11, [1, 4, 5, 6, 7]),
    PC12: (pc12, 12, [1, 4, 5, 6, 7]),
    PC13: (pc13, 13, [4]),
    PC14: (pc14, 14, []),
    PC15: (pc15, 15, []),
]);

#[cfg(feature = "gpio-f303")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [1, 7]),
    PD1: (pd1, 1, [1, 4, 6, 7]),
    PD2: (pd2, 2, [1, 2, 4, 5]),
    PD3: (pd3, 3, [1, 2, 7]),
    PD4: (pd4, 4, [1, 2, 7]),
    PD5: (pd5, 5, [1, 7]),
    PD6: (pd6, 6, [1, 2, 7]),
    PD7: (pd7, 7, [1, 2, 7]),
    PD8: (pd8, 8, [1, 7]),
    PD9: (pd9, 9, [1, 7]),
    PD10: (pd10, 10, [1, 7]),
    PD11: (pd11, 11, [1, 7]),
    PD12: (pd12, 12, [1, 2, 3, 7]),
    PD13: (pd13, 13, [1, 2, 3]),
    PD14: (pd14, 14, [1, 2, 3]),
    PD15: (pd15, 15, [1, 2, 3, 6]),
]);

#[cfg(feature = "gpio-f303")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [1, 2, 4, 7]),
    PE1: (pe1, 1, [1, 4, 7]),
    PE2: (pe2, 2, [0, 1, 2, 3]),
    PE3: (pe3, 3, [0, 1, 2, 3]),
    PE4: (pe4, 4, [0, 1, 2, 3]),
    PE5: (pe5, 5, [0, 1, 2, 3]),
    PE6: (pe6, 6, [0, 1]),
    PE7: (pe7, 7, [1, 2]),
    PE8: (pe8, 8, [1, 2]),
    PE9: (pe9, 9, [1, 2]),
    PE10: (pe10, 10, [1, 2]),
    PE11: (pe11, 11, [1, 2]),
    PE12: (pe12, 12, [1, 2]),
    PE13: (pe13, 13, [1, 2]),
    PE14: (pe14, 14, [1, 2, 6]),
    PE15: (pe15, 15, [1, 2, 7]),
]);

#[cfg(feature = "gpio-f303")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 6]),
    PF1: (pf1, 1, [4]),
    PF2: (pf2, 2, [1]),
    PF4: (pf4, 4, [1, 2]),
    PF6: (pf6, 6, [1, 2, 4, 7]),
    PF9: (pf9, 9, [1, 3, 5]),
    PF10: (pf10, 10, [1, 3, 5]),
]);

#[cfg(feature = "gpio-f333")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 3, 7, 15]),
    PA1: (pa1, 1, [1, 3, 7, 9, 15]),
    PA2: (pa2, 2, [1, 3, 7, 8, 9, 15]),
    PA3: (pa3, 3, [1, 3, 7, 9, 15]),
    PA4: (pa4, 4, [2, 3, 5, 7, 15]),
    PA5: (pa5, 5, [1, 3, 5, 15]),
    PA6: (pa6, 6, [1, 2, 3, 5, 6, 13, 15]),
    PA7: (pa7, 7, [1, 2, 3, 5, 6, 15]),
    PA8: (pa8, 8, [0, 6, 7, 13, 15]),
    PA9: (pa9, 9, [3, 6, 7, 9, 10, 13, 15]),
    PA10: (pa10, 10, [1, 3, 6, 7, 8, 10, 13, 15]),
    PA11: (pa11, 11, [6, 7, 9, 11, 12, 13, 15]),
    PA12: (pa12, 12, [1, 6, 7, 8, 9, 11, 13, 15]),
    PA13: (pa13, 13, [0, 1, 3, 5, 7, 15], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 3, 4, 6, 7, 15], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 3, 4, 5, 7, 9, 13, 15], super::Debugger), // JTDI, PullUp
]);

#[cfg(feature = "gpio-f333")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [2, 3, 6, 15]),
    PB1: (pb1, 1, [2, 3, 6, 8, 13, 15]),
    PB2: (pb2, 2, [3, 13, 15]),
    PB3: (pb3, 3, [0, 1, 3, 5, 7, 10, 12, 13, 15], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 3, 5, 7, 10, 13, 15], super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, [1, 2, 4, 5, 7, 10, 13, 15]),
    PB6: (pb6, 6, [1, 3, 4, 7, 12, 13, 15]),
    PB7: (pb7, 7, [1, 3, 4, 7, 10, 13, 15]),
    PB8: (pb8, 8, [1, 3, 4, 7, 9, 12, 13, 15]),
    PB9: (pb9, 9, [1, 4, 6, 7, 8, 9, 13, 15]),
    PB10: (pb10, 10, [1, 3, 7, 13, 15]),
    PB11: (pb11, 11, [1, 3, 7, 13, 15]),
    PB12: (pb12, 12, [3, 6, 7, 13, 15]),
    PB13: (pb13, 13, [3, 6, 7, 13, 15]),
    PB14: (pb14, 14, [1, 3, 6, 7, 13, 15]),
    PB15: (pb15, 15, [1, 2, 4, 13, 15]),
]);

#[cfg(feature = "gpio-f333")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 2]),
    PC1: (pc1, 1, [1, 2]),
    PC2: (pc2, 2, [1, 2]),
    PC3: (pc3, 3, [1, 2, 6]),
    PC4: (pc4, 4, [1, 2, 7]),
    PC5: (pc5, 5, [1, 2, 3, 7]),
    PC6: (pc6, 6, [1, 2, 3, 7]),
    PC7: (pc7, 7, [1, 2, 3]),
    PC8: (pc8, 8, [1, 2, 3]),
    PC9: (pc9, 9, [1, 2, 3]),
    PC10: (pc10, 10, [1, 7]),
    PC11: (pc11, 11, [1, 3, 7]),
    PC12: (pc12, 12, [1, 3, 7]),
    PC13: (pc13, 13, [4]),
    PC14: (pc14, 14, []),
    PC15: (pc15, 15, []),
]);

#[cfg(feature = "gpio-f333")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD2: (pd2, 2, [1, 2]),
]);

#[cfg(feature = "gpio-f333")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [6]),
    PF1: (pf1, 1, []),
]);

#[cfg(feature = "gpio-f373")]
gpio!(GPIOA, gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 7, 8, 11, 15]),
    PA1: (pa1, 1, [0, 1, 2, 3, 6, 7, 9, 11, 15]),
    PA2: (pa2, 2, [1, 2, 3, 6, 7, 8, 9, 11, 15]),
    PA3: (pa3, 3, [1, 2, 3, 6, 7, 9, 11, 15]),
    PA4: (pa4, 4, [2, 3, 5, 6, 7, 10, 15]),
    PA5: (pa5, 5, [1, 3, 5, 7, 9, 10, 15]),
    PA6: (pa6, 6, [1, 2, 3, 5, 8, 9, 15]),
    PA7: (pa7, 7, [1, 2, 3, 5, 8, 9, 15]),
    PA8: (pa8, 8, [0, 2, 4, 5, 7, 10, 15]),
    PA9: (pa9, 9, [2, 3, 4, 5, 7, 9, 10, 15]),
    PA10: (pa10, 10, [1, 3, 4, 5, 7, 9, 10, 15]),
    PA11: (pa11, 11, [2, 5, 6, 7, 8, 9, 10, 14, 15]),
    PA12: (pa12, 12, [1, 2, 6, 7, 8, 9, 10, 14, 15]),
    PA13: (pa13, 13, [0, 1, 2, 3, 5, 6, 7, 10, 15], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 3, 4, 10, 15], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 3, 4, 5, 6, 10, 15], super::Debugger), // JTDI, PullUp
]);

#[cfg(feature = "gpio-f373")]
gpio!(GPIOB, gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [2, 3, 5, 10, 15]),
    PB1: (pb1, 1, [2, 3, 15]),
    PB2: (pb2, 2, [15]),
    PB3: (pb3, 3, [0, 1, 2, 3, 5, 6, 7, 9, 10, 15], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 3, 5, 6, 7, 9, 10, 15], super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, [1, 2, 4, 5, 6, 7, 10, 11, 15]),
    PB6: (pb6, 6, [1, 2, 3, 4, 7, 9, 10, 11, 15]),
    PB7: (pb7, 7, [1, 2, 3, 4, 7, 9, 10, 11, 15]),
    PB8: (pb8, 8, [1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 15]),
    PB9: (pb9, 9, [1, 2, 4, 5, 6, 7, 8, 9, 11, 15]),
    PB10: (pb10, 10, [1, 3, 5, 6, 7, 15]),
    PB14: (pb14, 14, [1, 3, 5, 7, 9, 15]),
    PB15: (pb15, 15, [0, 1, 2, 3, 5, 9, 15]),
]);

#[cfg(feature = "gpio-f373")]
gpio!(GPIOC, gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 2]),
    PC1: (pc1, 1, [1, 2]),
    PC2: (pc2, 2, [1, 2, 5]),
    PC3: (pc3, 3, [1, 2, 5]),
    PC4: (pc4, 4, [1, 2, 3, 7]),
    PC5: (pc5, 5, [1, 3, 7]),
    PC6: (pc6, 6, [1, 2, 5]),
    PC7: (pc7, 7, [1, 2, 5]),
    PC8: (pc8, 8, [1, 2, 5]),
    PC9: (pc9, 9, [1, 2, 5]),
    PC10: (pc10, 10, [1, 2, 6, 7]),
    PC11: (pc11, 11, [1, 2, 6, 7]),
    PC12: (pc12, 12, [1, 2, 6, 7]),
    PC13: (pc13, 13, []),
    PC14: (pc14, 14, []),
    PC15: (pc15, 15, []),
]);

#[cfg(feature = "gpio-f373")]
gpio!(GPIOD, gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [1, 2, 7]),
    PD1: (pd1, 1, [1, 2, 7]),
    PD2: (pd2, 2, [1, 2]),
    PD3: (pd3, 3, [1, 5, 7]),
    PD4: (pd4, 4, [1, 5, 7]),
    PD5: (pd5, 5, [1, 7]),
    PD6: (pd6, 6, [1, 5, 7]),
    PD7: (pd7, 7, [1, 5, 7]),
    PD8: (pd8, 8, [1, 3, 5, 7]),
    PD9: (pd9, 9, [1, 3, 7]),
    PD10: (pd10, 10, [1, 7]),
    PD11: (pd11, 11, [1, 7]),
    PD12: (pd12, 12, [1, 2, 3, 7]),
    PD13: (pd13, 13, [1, 2, 3]),
    PD14: (pd14, 14, [1, 2, 3]),
    PD15: (pd15, 15, [1, 2, 3]),
]);

#[cfg(feature = "gpio-f373")]
gpio!(GPIOE, gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [1, 2, 7]),
    PE1: (pe1, 1, [1, 7]),
    PE2: (pe2, 2, [0, 1, 3]),
    PE3: (pe3, 3, [0, 1, 3]),
    PE4: (pe4, 4, [0, 1, 3]),
    PE5: (pe5, 5, [0, 1, 3]),
    PE6: (pe6, 6, [0, 1]),
    PE7: (pe7, 7, [1]),
    PE8: (pe8, 8, [1]),
    PE9: (pe9, 9, [1]),
    PE10: (pe10, 10, [1]),
    PE11: (pe11, 11, [1]),
    PE12: (pe12, 12, [1]),
    PE13: (pe13, 13, [1]),
    PE14: (pe14, 14, [1]),
    PE15: (pe15, 15, [1, 7]),
]);

#[cfg(feature = "gpio-f373")]
gpio!(GPIOF, gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4]),
    PF1: (pf1, 1, [4]),
    PF2: (pf2, 2, [1, 4]),
    PF4: (pf4, 4, [1]),
    PF6: (pf6, 6, [1, 2, 4, 5, 7]),
    PF7: (pf7, 7, [1, 4, 7]),
    PF9: (pf9, 9, [1, 2]),
    PF10: (pf10, 10, [1]),
]);

struct Gpio<const P: char>;
impl<const P: char> Gpio<P> {
    const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
        match P {
            'A' => crate::pac::GPIOA::ptr(),
            'B' => crate::pac::GPIOB::ptr() as _,
            'C' => crate::pac::GPIOC::ptr() as _,
            'D' => crate::pac::GPIOD::ptr() as _,
            #[cfg(any(feature = "gpio-f303e", feature = "gpio-f303", feature = "gpio-f373"))]
            'E' => crate::pac::GPIOE::ptr() as _,
            'F' => crate::pac::GPIOF::ptr() as _,
            #[cfg(feature = "gpio-f303e")]
            'G' => crate::pac::GPIOG::ptr() as _,
            #[cfg(feature = "gpio-f303e")]
            'H' => crate::pac::GPIOH::ptr() as _,
            _ => crate::pac::GPIOA::ptr(),
        }
    }
}

// Make all GPIO peripheral trait extensions sealable.
impl<const P: char, const N: u8, MODE> crate::private::Sealed for Pin<P, N, MODE> {}
