//! General Purpose Input / Output
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
//! ### Internal Resistor
//!
//! Weak internal pull-up and pull-down resistors are configurable by calling
//! [`set_internal_resistor`](Pin::set_internal_resistor) method. `into_..._input` methods are also
//! available for convenience.
//!
//! [InputPin]: embedded_hal::digital::v2::InputPin
//! [OutputPin]: embedded_hal::digital::v2::OutputPin
//! [examples/toggle.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.1/examples/toggle.rs

use core::{convert::Infallible, marker::PhantomData};

use crate::{hal::digital::v2::OutputPin, pac::EXTI, rcc::AHB, syscfg::SysCfg};

#[cfg(feature = "unproven")]
use crate::hal::digital::v2::{toggleable, InputPin, StatefulOutputPin};

use typenum::{Unsigned, U0, U1, U10, U11, U12, U13, U14, U15, U2, U3, U4, U5, U6, U7, U8, U9};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The Parts to split the GPIO peripheral into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, ahb: &mut AHB) -> Self::Parts;
}

/// GPIO Register interface traits private to this module
mod private {
    pub trait GpioRegExt {
        fn is_low(&self, i: u8) -> bool;
        fn is_set_low(&self, i: u8) -> bool;
        fn set_high(&self, i: u8);
        fn set_low(&self, i: u8);
    }

    pub trait Moder {
        fn input(&mut self, i: u8);
        fn output(&mut self, i: u8);
        fn alternate(&mut self, i: u8);
        fn analog(&mut self, i: u8);
    }

    pub trait Otyper {
        fn push_pull(&mut self, i: u8);
        fn open_drain(&mut self, i: u8);
    }

    pub trait Ospeedr {
        fn low(&mut self, i: u8);
        fn medium(&mut self, i: u8);
        fn high(&mut self, i: u8);
    }

    pub trait Pupdr {
        fn floating(&mut self, i: u8);
        fn pull_up(&mut self, i: u8);
        fn pull_down(&mut self, i: u8);
    }

    pub trait Afr {
        fn afx(&mut self, i: u8, x: u8);
    }

    pub trait Gpio {
        type Reg: GpioRegExt + ?Sized;

        fn ptr(&self) -> *const Self::Reg;
        fn port_index(&self) -> u8;
    }
}

use private::{Afr, GpioRegExt, Moder, Ospeedr, Otyper, Pupdr};

/// Marker trait for GPIO ports
pub trait Gpio: private::Gpio {}

/// Marker trait for compile time defined GPIO ports
pub trait GpioStatic: Gpio {
    /// Associated MODER register
    type MODER: Moder;
    /// Associated OTYPER register
    type OTYPER: Otyper;
    /// Associated OSPEEDR register
    type OSPEEDR: Ospeedr;
    /// Associated PUPDR register
    type PUPDR: Pupdr;
}

/// Marker trait for pin number
pub trait Index {
    #[doc(hidden)]
    fn index(&self) -> u8;
}

impl<U> Index for U
where
    U: Unsigned,
{
    #[inline(always)]
    fn index(&self) -> u8 {
        Self::U8
    }
}

/// Marker trait for readable pin modes
pub trait Readable {}

/// Marker trait for slew rate configurable pin modes
pub trait OutputSpeed {}

/// Marker trait for active pin modes
pub trait Active {}

/// Runtime defined GPIO port (type state)
pub struct Gpiox {
    ptr: *const dyn GpioRegExt,
    index: u8,
}

impl private::Gpio for Gpiox {
    type Reg = dyn GpioRegExt;

    fn ptr(&self) -> *const Self::Reg {
        self.ptr
    }

    fn port_index(&self) -> u8 {
        self.index
    }
}

impl Gpio for Gpiox {}

/// Runtime defined pin number (type state)
pub struct Ux(u8);

impl Index for Ux {
    fn index(&self) -> u8 {
        self.0
    }
}

/// Input mode (type state)
pub struct Input;
/// Output mode (type state)
pub struct Output<OTYPE>(PhantomData<OTYPE>);
/// Alternate function (type state)
pub struct Alternate<AF, OTYPE>(PhantomData<AF>, PhantomData<OTYPE>);
/// Analog mode (type state)
pub struct Analog;

/// Push-pull output (type state)
pub struct PushPull;
/// Open-drain output (type state)
pub struct OpenDrain;

impl Readable for Input {}
impl Readable for Output<OpenDrain> {}
impl<OTYPE> OutputSpeed for Output<OTYPE> {}
impl<AF, OTYPE> OutputSpeed for Alternate<AF, OTYPE> {}
impl Active for Input {}
impl<OTYPE> Active for Output<OTYPE> {}
impl<AF, OTYPE> Active for Alternate<AF, OTYPE> {}

/// Slew rate configuration
pub enum Speed {
    /// Low speed
    Low,
    /// Medium speed
    Medium,
    /// High speed
    High,
}

/// Internal pull-up and pull-down resistor configuration
pub enum Resistor {
    /// Floating
    Floating,
    /// Pulled up
    PullUp,
    /// Pulled down
    PullDown,
}

/// GPIO interrupt trigger edge selection
pub enum Edge {
    /// Rising edge of voltage
    Rising,
    /// Falling edge of voltage
    Falling,
    /// Rising and falling edge of voltage
    RisingFalling,
}

/// Generic pin
pub struct Pin<GPIO, INDEX, MODE> {
    gpio: GPIO,
    index: INDEX,
    _mode: PhantomData<MODE>,
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
pub type PXx<MODE> = Pin<Gpiox, Ux, MODE>;

macro_rules! modify_at {
    ($xr:expr, $bits:expr, $i:expr, $val:expr) => {
        $xr.modify(|r, w| {
            w.bits(r.bits() & !(u32::MAX >> 32 - $bits << $bits * $i) | $val << $bits * $i)
        });
    };
}

impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE>
where
    INDEX: Unsigned,
{
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn downgrade(self) -> Pin<GPIO, Ux, MODE> {
        Pin {
            gpio: self.gpio,
            index: Ux(INDEX::U8),
            _mode: self._mode,
        }
    }
}

impl<GPIO, MODE> Pin<GPIO, Ux, MODE>
where
    GPIO: GpioStatic,
    GPIO::Reg: 'static + Sized,
{
    /// Erases the port letter from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn downgrade(self) -> PXx<MODE> {
        PXx {
            gpio: Gpiox {
                ptr: self.gpio.ptr(),
                index: self.gpio.port_index(),
            },
            index: self.index,
            _mode: self._mode,
        }
    }
}

impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE> {
    fn into_mode<NEW_MODE>(self) -> Pin<GPIO, INDEX, NEW_MODE> {
        Pin {
            gpio: self.gpio,
            index: self.index,
            _mode: PhantomData,
        }
    }
}

impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE>
where
    GPIO: GpioStatic,
    INDEX: Index,
{
    /// Configures the pin to operate as an input pin
    pub fn into_input(self, moder: &mut GPIO::MODER) -> Pin<GPIO, INDEX, Input> {
        moder.input(self.index.index());
        self.into_mode()
    }

    /// Convenience method to configure the pin to operate as an input pin
    /// and set the internal resistor floating
    pub fn into_floating_input(
        self,
        moder: &mut GPIO::MODER,
        pupdr: &mut GPIO::PUPDR,
    ) -> Pin<GPIO, INDEX, Input> {
        moder.input(self.index.index());
        pupdr.floating(self.index.index());
        self.into_mode()
    }

    /// Convenience method to configure the pin to operate as an input pin
    /// and set the internal resistor pull-up
    pub fn into_pull_up_input(
        self,
        moder: &mut GPIO::MODER,
        pupdr: &mut GPIO::PUPDR,
    ) -> Pin<GPIO, INDEX, Input> {
        moder.input(self.index.index());
        pupdr.pull_up(self.index.index());
        self.into_mode()
    }

    /// Convenience method to configure the pin to operate as an input pin
    /// and set the internal resistor pull-down
    pub fn into_pull_down_input(
        self,
        moder: &mut GPIO::MODER,
        pupdr: &mut GPIO::PUPDR,
    ) -> Pin<GPIO, INDEX, Input> {
        moder.input(self.index.index());
        pupdr.pull_down(self.index.index());
        self.into_mode()
    }

    /// Configures the pin to operate as a push-pull output pin
    pub fn into_push_pull_output(
        self,
        moder: &mut GPIO::MODER,
        otyper: &mut GPIO::OTYPER,
    ) -> Pin<GPIO, INDEX, Output<PushPull>> {
        moder.output(self.index.index());
        otyper.push_pull(self.index.index());
        self.into_mode()
    }

    /// Configures the pin to operate as an open-drain output pin
    pub fn into_open_drain_output(
        self,
        moder: &mut GPIO::MODER,
        otyper: &mut GPIO::OTYPER,
    ) -> Pin<GPIO, INDEX, Output<OpenDrain>> {
        moder.output(self.index.index());
        otyper.open_drain(self.index.index());
        self.into_mode()
    }

    /// Configures the pin to operate as an analog pin, with disabled schmitt trigger.
    pub fn into_analog(
        self,
        moder: &mut GPIO::MODER,
        pupdr: &mut GPIO::PUPDR,
    ) -> Pin<GPIO, INDEX, Analog> {
        moder.analog(self.index.index());
        pupdr.floating(self.index.index());
        self.into_mode()
    }
}

impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE>
where
    GPIO: GpioStatic,
    INDEX: Index,
    MODE: OutputSpeed,
{
    /// Set pin output slew rate
    pub fn set_speed(&mut self, ospeedr: &mut GPIO::OSPEEDR, speed: Speed) {
        match speed {
            Speed::Low => ospeedr.low(self.index.index()),
            Speed::Medium => ospeedr.medium(self.index.index()),
            Speed::High => ospeedr.high(self.index.index()),
        }
    }
}

impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE>
where
    GPIO: GpioStatic,
    INDEX: Index,
    MODE: Active,
{
    /// Set the internal pull-up and pull-down resistor
    pub fn set_internal_resistor(&mut self, pupdr: &mut GPIO::PUPDR, resistor: Resistor) {
        match resistor {
            Resistor::Floating => pupdr.floating(self.index.index()),
            Resistor::PullUp => pupdr.pull_up(self.index.index()),
            Resistor::PullDown => pupdr.pull_down(self.index.index()),
        }
    }

    /// Enables / disables the internal pull up (Provided for compatibility with other stm32 HALs)
    pub fn internal_pull_up(&mut self, pupdr: &mut GPIO::PUPDR, on: bool) {
        if on {
            pupdr.pull_up(self.index.index());
        } else {
            pupdr.floating(self.index.index());
        }
    }
}

impl<GPIO, INDEX, OTYPE> OutputPin for Pin<GPIO, INDEX, Output<OTYPE>>
where
    GPIO: Gpio,
    INDEX: Index,
{
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*self.gpio.ptr()).set_high(self.index.index()) };
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*self.gpio.ptr()).set_low(self.index.index()) };
        Ok(())
    }
}

#[cfg(feature = "unproven")]
impl<GPIO, INDEX, MODE> InputPin for Pin<GPIO, INDEX, MODE>
where
    GPIO: Gpio,
    INDEX: Index,
    MODE: Readable,
{
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_low()?)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*self.gpio.ptr()).is_low(self.index.index()) })
    }
}

#[cfg(feature = "unproven")]
impl<GPIO, INDEX, OTYPE> StatefulOutputPin for Pin<GPIO, INDEX, Output<OTYPE>>
where
    GPIO: Gpio,
    INDEX: Index,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_low()?)
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*self.gpio.ptr()).is_set_low(self.index.index()) })
    }
}

#[cfg(feature = "unproven")]
impl<GPIO, INDEX, OTYPE> toggleable::Default for Pin<GPIO, INDEX, Output<OTYPE>>
where
    GPIO: Gpio,
    INDEX: Index,
{
}

/// Return an EXTI register for the current CPU
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
macro_rules! reg_for_cpu {
    ($exti:expr, $xr:ident) => {
        $exti.$xr
    };
}

/// Return an EXTI register for the current CPU
#[cfg(not(any(feature = "stm32f373", feature = "stm32f378")))]
macro_rules! reg_for_cpu {
    ($exti:expr, $xr:ident) => {
        paste::paste! {
            $exti.[<$xr 1>]
        }
    };
}

impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE>
where
    GPIO: Gpio,
    INDEX: Index,
    MODE: Active,
{
    /// Make corresponding EXTI line sensitive to this pin
    pub fn make_interrupt_source(&mut self, syscfg: &mut SysCfg) {
        let i = self.index.index() % 4;
        let extigpionr = self.gpio.port_index() as u32;
        match self.index.index() {
            0..=3 => unsafe { modify_at!(syscfg.exticr1, 4, i, extigpionr) },
            4..=7 => unsafe { modify_at!(syscfg.exticr2, 4, i, extigpionr) },
            8..=11 => unsafe { modify_at!(syscfg.exticr3, 4, i, extigpionr) },
            12..=15 => unsafe { modify_at!(syscfg.exticr4, 4, i, extigpionr) },
            _ => unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge, or both
    pub fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        let (rise, fall) = match edge {
            Edge::Rising => (true, false),
            Edge::Falling => (false, true),
            Edge::RisingFalling => (true, true),
        };
        unsafe {
            modify_at!(reg_for_cpu!(exti, rtsr), 1, self.index.index(), rise as u32);
            modify_at!(reg_for_cpu!(exti, ftsr), 1, self.index.index(), fall as u32);
        }
    }

    /// Enable external interrupts from this pin.
    pub fn enable_interrupt(&mut self, exti: &mut EXTI) {
        unsafe { modify_at!(reg_for_cpu!(exti, imr), 1, self.index.index(), 1) };
    }

    /// Disable external interrupts from this pin
    pub fn disable_interrupt(&mut self, exti: &mut EXTI) {
        unsafe { modify_at!(reg_for_cpu!(exti, imr), 1, self.index.index(), 0) };
    }

    /// Clear the interrupt pending bit for this pin
    pub fn clear_interrupt_pending_bit(&mut self) {
        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).write(|w| w.bits(1 << self.index.index())) };
    }

    /// Reads the interrupt pending bit for this pin
    pub fn check_interrupt(&self) -> bool {
        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).read().bits() & (1 << self.index.index()) != 0 }
    }
}

macro_rules! af {
    ($i:literal, $Ui:ty, $AFi:ident, $IntoAfi:ident, $into_afi_push_pull:ident, $into_afi_open_drain:ident) => {
        paste::paste! {
            #[doc = "Marker trait for pins with alternate function " $i " mapping"]
            pub trait $IntoAfi {
                /// Associated AFR register
                type AFR: Afr;
            }
        }

        paste::paste! {
            #[doc = "Alternate function " $i " (type state)"]
            pub type $AFi<OTYPE> = Alternate<$Ui, OTYPE>;
        }

        impl<GPIO, INDEX, MODE> Pin<GPIO, INDEX, MODE>
        where
            Self: $IntoAfi,
            GPIO: GpioStatic,
            INDEX: Index,
        {
            /// Configures the pin to operate as an alternate function push-pull output pin
            pub fn $into_afi_push_pull(
                self,
                moder: &mut GPIO::MODER,
                otyper: &mut GPIO::OTYPER,
                afr: &mut <Self as $IntoAfi>::AFR,
            ) -> Pin<GPIO, INDEX, $AFi<PushPull>> {
                moder.alternate(self.index.index());
                otyper.push_pull(self.index.index());
                afr.afx(self.index.index(), $i);
                self.into_mode()
            }

            /// Configures the pin to operate as an alternate function open-drain output pin
            pub fn $into_afi_open_drain(
                self,
                moder: &mut GPIO::MODER,
                otyper: &mut GPIO::OTYPER,
                afr: &mut <Self as $IntoAfi>::AFR,
            ) -> Pin<GPIO, INDEX, $AFi<OpenDrain>> {
                moder.alternate(self.index.index());
                otyper.open_drain(self.index.index());
                afr.afx(self.index.index(), $i);
                self.into_mode()
            }
        }
    };

    ([$($i:literal),+ $(,)?]) => {
        paste::paste! {
            $(
                af!($i, [<U $i>], [<AF $i>], [<IntoAf $i>],  [<into_af $i _push_pull>],  [<into_af $i _open_drain>]);
            )+
        }
    };
}

af!([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);

macro_rules! gpio_trait {
    ([$($gpioy:ident),+ $(,)?]) => {
        $(
            impl GpioRegExt for crate::pac::$gpioy::RegisterBlock {
                #[inline(always)]
                fn is_low(&self, i: u8) -> bool {
                    self.idr.read().bits() & (1 << i) == 0
                }

                #[inline(always)]
                fn is_set_low(&self, i: u8) -> bool {
                    self.odr.read().bits() & (1 << i) == 0
                }

                #[inline(always)]
                fn set_high(&self, i: u8) {
                    // NOTE(unsafe, write) atomic write to a stateless register
                    unsafe { self.bsrr.write(|w| w.bits(1 << i)); }
                }

                #[inline(always)]
                fn set_low(&self, i: u8) {
                    // NOTE(unsafe, write) atomic write to a stateless register
                    unsafe { self.bsrr.write(|w| w.bits(1 << (16 + i))); }
                }
            }
        )+
    };
}

macro_rules! r_trait {
    (
        ($GPIOX:ident, $gpioy:ident::$xr:ident::$enum:ident, $bits:expr);
        impl $Xr:ident for $XR:ty {
            $(
                fn $fn:ident { $VARIANT:ident }
            )+
        }
    ) => {
        impl $Xr for $XR {
            $(
                #[inline]
                fn $fn(&mut self, i: u8) {
                    unsafe {
                        modify_at!((*$GPIOX::ptr()).$xr, $bits, i, $gpioy::$xr::$enum::$VARIANT as u32);
                    }
                }
            )+
        }
    };
}

macro_rules! gpio {
    ({
        GPIO: $GPIOX:ident,
        gpio: $gpiox:ident,
        Gpio: $Gpiox:ty,
        port_index: $port_index:literal,
        gpio_mapped: $gpioy:ident,
        iopen: $iopxen:ident,
        ioprst: $iopxrst:ident,
        partially_erased_pin: $PXx:ty,
        pins: {$(
            $PXi:ty: (
                $pxi:ident, $Ui:ty, $MODE:ty, $AFR:ident, [$($IntoAfi:ident),*],
            ),
        )+},
    }) => {
        paste::paste!{
            #[doc = "GPIO port " $GPIOX " (type state)"]
            pub struct $Gpiox;
        }

        impl private::Gpio for $Gpiox {
            type Reg = crate::pac::$gpioy::RegisterBlock;

            #[inline(always)]
            fn ptr(&self) -> *const Self::Reg {
                crate::pac::$GPIOX::ptr()
            }

            #[inline(always)]
            fn port_index(&self) -> u8 {
                $port_index
            }
        }

        impl Gpio for $Gpiox {}

        impl GpioStatic for $Gpiox {
            type MODER = $gpiox::MODER;
            type OTYPER = $gpiox::OTYPER;
            type OSPEEDR = $gpiox::OSPEEDR;
            type PUPDR = $gpiox::PUPDR;
        }

        paste::paste!{
            #[doc = "All Pins and associated registers for GPIO port " $GPIOX]
            pub mod $gpiox {
                use core::marker::PhantomData;

                use crate::{
                    pac::{$gpioy, $GPIOX},
                    rcc::AHB,
                };

                use super::{Afr, $Gpiox, GpioExt, Moder, Ospeedr, Otyper, Pin, Pupdr, Ux};

                #[allow(unused_imports)]
                use super::{
                    Input, Output, Analog, PushPull, OpenDrain,
                    IntoAf0, IntoAf1, IntoAf2, IntoAf3, IntoAf4, IntoAf5, IntoAf6, IntoAf7,
                    IntoAf8, IntoAf9, IntoAf10, IntoAf11, IntoAf12, IntoAf13, IntoAf14, IntoAf15,
                    AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15,
                };

                #[allow(unused_imports)]
                use typenum::{
                    U0, U1, U2, U3, U4, U5, U6, U7, U8, U9, U10, U11, U12, U13, U14, U15
                };

                /// GPIO parts
                pub struct Parts {
                    /// Opaque AFRH register
                    pub afrh: AFRH,
                    /// Opaque AFRL register
                    pub afrl: AFRL,
                    /// Opaque MODER register
                    pub moder: MODER,
                    /// Opaque OSPEEDR register
                    pub ospeedr: OSPEEDR,
                    /// Opaque OTYPER register
                    pub otyper: OTYPER,
                    /// Opaque PUPDR register
                    pub pupdr: PUPDR,
                    $(
                        #[doc = "Pin " $PXi]
                        pub $pxi: $PXi<$MODE>,
                    )+
                }

                impl GpioExt for $GPIOX {
                    type Parts = Parts;

                    fn split(self, ahb: &mut AHB) -> Parts {
                        ahb.enr().modify(|_, w| w.$iopxen().set_bit());
                        ahb.rstr().modify(|_, w| w.$iopxrst().set_bit());
                        ahb.rstr().modify(|_, w| w.$iopxrst().clear_bit());

                        Parts {
                            afrh: AFRH(()),
                            afrl: AFRL(()),
                            moder: MODER(()),
                            ospeedr: OSPEEDR(()),
                            otyper: OTYPER(()),
                            pupdr: PUPDR(()),
                            $(
                                $pxi: $PXi {
                                    gpio: $Gpiox,
                                    index: $Ui::new(),
                                    _mode: PhantomData,
                                },
                            )+
                        }
                    }
                }

                /// Opaque AFRH register
                pub struct AFRH(());

                impl Afr for AFRH {
                    #[inline]
                    fn afx(&mut self, i: u8, x: u8) {
                        unsafe { modify_at!((*$GPIOX::ptr()).afrh, 4, i - 8, x as u32); }
                    }
                }

                /// Opaque AFRL register
                pub struct AFRL(());

                impl Afr for AFRL {
                    #[inline]
                    fn afx(&mut self, i: u8, x: u8) {
                        unsafe { modify_at!((*$GPIOX::ptr()).afrl, 4, i, x as u32); }
                    }
                }

                /// Opaque MODER register
                pub struct MODER(());

                r_trait! {
                    ($GPIOX, $gpioy::moder::MODER15_A, 2);
                    impl Moder for MODER {
                        fn input { INPUT }
                        fn output { OUTPUT }
                        fn alternate { ALTERNATE }
                        fn analog { ANALOG }
                    }
                }

                /// Opaque OSPEEDR register
                pub struct OSPEEDR(());

                r_trait! {
                    ($GPIOX, $gpioy::ospeedr::OSPEEDR15_A, 2);
                    impl Ospeedr for OSPEEDR {
                        fn low { LOWSPEED }
                        fn medium { MEDIUMSPEED }
                        fn high { HIGHSPEED }
                    }
                }

                /// Opaque OTYPER register
                pub struct OTYPER(());

                r_trait! {
                    ($GPIOX, $gpioy::otyper::OT15_A, 1);
                    impl Otyper for OTYPER {
                        fn push_pull { PUSHPULL }
                        fn open_drain { OPENDRAIN }
                    }
                }

                /// Opaque PUPDR register
                pub struct PUPDR(());

                r_trait! {
                    ($GPIOX, $gpioy::pupdr::PUPDR15_A, 2);
                    impl Pupdr for PUPDR {
                        fn floating { FLOATING }
                        fn pull_up { PULLUP }
                        fn pull_down { PULLDOWN }
                    }
                }

                /// Partially erased pin
                pub type $PXx<MODE> = Pin<$Gpiox, Ux, MODE>;

                $(
                    #[doc = "Pin " $PXi]
                    pub type $PXi<MODE> = Pin<$Gpiox, $Ui, MODE>;

                    $(
                        impl<MODE> $IntoAfi for $PXi<MODE> {
                            type AFR = $AFR;
                        }
                    )*
                )+
            }
        }
    };

    ({
        pacs: $pacs:tt,
        ports: [$(
            {
                port: ($X:ident/$x:ident, $port_index:literal, $gpioy:ident),
                pins: [$(
                    $i:literal => {
                        reset: $MODE:ty,
                        afr: $LH:ident,
                        af: [$($af:literal),*]
                    },
                )+],
            },
        )+],
    }) => {
        paste::paste! {
            gpio_trait!($pacs);
            $(
                gpio!({
                    GPIO: [<GPIO $X>],
                    gpio: [<gpio $x>],
                    Gpio: [<Gpio $x>],
                    port_index: $port_index,
                    gpio_mapped: $gpioy,
                    iopen: [<iop $x en>],
                    ioprst: [<iop $x rst>],
                    partially_erased_pin: [<P $X x>],
                    pins: {$(
                        [<P $X $i>]: (
                            [<p $x $i>], [<U $i>], $MODE, [<AFR $LH>], [$([<IntoAf $af>]),*],
                        ),
                    )+},
                });
            )+
        }
    };
}
// auto-generated using codegen
// STM32CubeMX DB release: DB.6.0.10

#[cfg(feature = "gpio-f302")]
gpio!({
    pacs: [gpioa, gpiob, gpioc],
    ports: [
        {
            port: (A/a, 0, gpioa),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 3, 7, 15] },
                1 => { reset: Input, afr: L, af: [0, 1, 3, 7, 9, 15] },
                2 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 15] },
                3 => { reset: Input, afr: L, af: [1, 3, 7, 9, 15] },
                4 => { reset: Input, afr: L, af: [3, 6, 7, 15] },
                5 => { reset: Input, afr: L, af: [1, 3, 15] },
                6 => { reset: Input, afr: L, af: [1, 3, 6, 15] },
                7 => { reset: Input, afr: L, af: [1, 3, 6, 15] },
                8 => { reset: Input, afr: H, af: [0, 3, 4, 5, 6, 7, 15] },
                9 => { reset: Input, afr: H, af: [2, 3, 4, 5, 6, 7, 9, 10, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 4, 5, 6, 7, 8, 10, 15] },
                11 => { reset: Input, afr: H, af: [5, 6, 7, 9, 11, 12, 15] },
                12 => { reset: Input, afr: H, af: [1, 5, 6, 7, 8, 9, 11, 15] },
                13 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 5, 7, 15] },
                14 => { reset: AF0<PushPull>, afr: H, af: [0, 3, 4, 6, 7, 15] },
                15 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 4, 6, 7, 9, 15] },
            ],
        },
        {
            port: (B/b, 1, gpiob),
            pins: [
                0 => { reset: Input, afr: L, af: [3, 6, 15] },
                1 => { reset: Input, afr: L, af: [3, 6, 8, 15] },
                2 => { reset: Input, afr: L, af: [3, 15] },
                3 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 3, 6, 7, 15] },
                4 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 3, 6, 7, 10, 15] },
                5 => { reset: Input, afr: L, af: [1, 4, 6, 7, 8, 10, 15] },
                6 => { reset: Input, afr: L, af: [1, 3, 4, 7, 15] },
                7 => { reset: Input, afr: L, af: [1, 3, 4, 7, 15] },
                8 => { reset: Input, afr: H, af: [1, 3, 4, 7, 9, 12, 15] },
                9 => { reset: Input, afr: H, af: [1, 4, 6, 7, 8, 9, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                11 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                12 => { reset: Input, afr: H, af: [3, 4, 5, 6, 7, 15] },
                13 => { reset: Input, afr: H, af: [3, 5, 6, 7, 15] },
                14 => { reset: Input, afr: H, af: [1, 3, 5, 6, 7, 15] },
                15 => { reset: Input, afr: H, af: [0, 1, 2, 4, 5, 15] },
            ],
        },
        {
            port: (C/c, 2, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2] },
                1 => { reset: Input, afr: L, af: [1, 2] },
                2 => { reset: Input, afr: L, af: [1, 2] },
                3 => { reset: Input, afr: L, af: [1, 2, 6] },
                4 => { reset: Input, afr: L, af: [1, 2, 7] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 6, 7] },
                7 => { reset: Input, afr: L, af: [1, 6] },
                8 => { reset: Input, afr: H, af: [1] },
                9 => { reset: Input, afr: H, af: [1, 3, 5] },
                10 => { reset: Input, afr: H, af: [1, 6, 7] },
                11 => { reset: Input, afr: H, af: [1, 6, 7] },
                12 => { reset: Input, afr: H, af: [1, 6, 7] },
                13 => { reset: Input, afr: H, af: [4] },
                14 => { reset: Input, afr: H, af: [] },
                15 => { reset: Input, afr: H, af: [] },
            ],
        },
        {
            port: (D/d, 3, gpioc),
            pins: [
                2 => { reset: Input, afr: L, af: [1] },
            ],
        },
        {
            port: (F/f, 5, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [4, 5, 6] },
                1 => { reset: Input, afr: L, af: [4, 5] },
            ],
        },
    ],
});

#[cfg(feature = "gpio-f303e")]
gpio!({
    pacs: [gpioa, gpiob, gpioc],
    ports: [
        {
            port: (A/a, 0, gpioa),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 10, 15] },
                1 => { reset: Input, afr: L, af: [0, 1, 3, 7, 9, 15] },
                2 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 15] },
                3 => { reset: Input, afr: L, af: [1, 3, 7, 9, 15] },
                4 => { reset: Input, afr: L, af: [2, 3, 5, 6, 7, 15] },
                5 => { reset: Input, afr: L, af: [1, 3, 5, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 8, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 15] },
                8 => { reset: Input, afr: H, af: [0, 3, 4, 5, 6, 7, 8, 10, 15] },
                9 => { reset: Input, afr: H, af: [2, 3, 4, 5, 6, 7, 8, 9, 10, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 4, 5, 6, 7, 8, 10, 11, 15] },
                11 => { reset: Input, afr: H, af: [5, 6, 7, 8, 9, 10, 11, 12, 15] },
                12 => { reset: Input, afr: H, af: [1, 5, 6, 7, 8, 9, 10, 11, 15] },
                13 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 5, 7, 10, 15] },
                14 => { reset: AF0<PushPull>, afr: H, af: [0, 3, 4, 5, 6, 7, 15] },
                15 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 2, 3, 4, 5, 6, 7, 9, 15] },
            ],
        },
        {
            port: (B/b, 1, gpiob),
            pins: [
                0 => { reset: Input, afr: L, af: [2, 3, 4, 6, 15] },
                1 => { reset: Input, afr: L, af: [2, 3, 4, 6, 8, 15] },
                2 => { reset: Input, afr: L, af: [3, 15] },
                3 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
                4 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 7, 8, 10, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 7, 10, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 7, 10, 12, 15] },
                8 => { reset: Input, afr: H, af: [1, 2, 3, 4, 7, 8, 9, 10, 12, 15] },
                9 => { reset: Input, afr: H, af: [1, 2, 4, 6, 7, 8, 9, 10, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                11 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                12 => { reset: Input, afr: H, af: [3, 4, 5, 6, 7, 15] },
                13 => { reset: Input, afr: H, af: [3, 5, 6, 7, 15] },
                14 => { reset: Input, afr: H, af: [1, 3, 5, 6, 7, 15] },
                15 => { reset: Input, afr: H, af: [0, 1, 2, 4, 5, 15] },
            ],
        },
        {
            port: (C/c, 2, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2] },
                1 => { reset: Input, afr: L, af: [1, 2] },
                2 => { reset: Input, afr: L, af: [1, 2, 3] },
                3 => { reset: Input, afr: L, af: [1, 2, 6] },
                4 => { reset: Input, afr: L, af: [1, 2, 7] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 2, 4, 6, 7] },
                7 => { reset: Input, afr: L, af: [1, 2, 4, 6, 7] },
                8 => { reset: Input, afr: H, af: [1, 2, 4, 7] },
                9 => { reset: Input, afr: H, af: [1, 2, 3, 4, 5, 6] },
                10 => { reset: Input, afr: H, af: [1, 4, 5, 6, 7] },
                11 => { reset: Input, afr: H, af: [1, 4, 5, 6, 7] },
                12 => { reset: Input, afr: H, af: [1, 4, 5, 6, 7] },
                13 => { reset: Input, afr: H, af: [1, 4] },
                14 => { reset: Input, afr: H, af: [1] },
                15 => { reset: Input, afr: H, af: [1] },
            ],
        },
        {
            port: (D/d, 3, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 7, 12] },
                1 => { reset: Input, afr: L, af: [1, 4, 6, 7, 12] },
                2 => { reset: Input, afr: L, af: [1, 2, 4, 5] },
                3 => { reset: Input, afr: L, af: [1, 2, 7, 12] },
                4 => { reset: Input, afr: L, af: [1, 2, 7, 12] },
                5 => { reset: Input, afr: L, af: [1, 7, 12] },
                6 => { reset: Input, afr: L, af: [1, 2, 7, 12] },
                7 => { reset: Input, afr: L, af: [1, 2, 7, 12] },
                8 => { reset: Input, afr: H, af: [1, 7, 12] },
                9 => { reset: Input, afr: H, af: [1, 7, 12] },
                10 => { reset: Input, afr: H, af: [1, 7, 12] },
                11 => { reset: Input, afr: H, af: [1, 7, 12] },
                12 => { reset: Input, afr: H, af: [1, 2, 3, 7, 12] },
                13 => { reset: Input, afr: H, af: [1, 2, 3, 12] },
                14 => { reset: Input, afr: H, af: [1, 2, 3, 12] },
                15 => { reset: Input, afr: H, af: [1, 2, 3, 6, 12] },
            ],
        },
        {
            port: (E/e, 4, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 4, 6, 7, 12] },
                1 => { reset: Input, afr: L, af: [1, 4, 6, 7, 12] },
                2 => { reset: Input, afr: L, af: [0, 1, 2, 3, 5, 6, 12] },
                3 => { reset: Input, afr: L, af: [0, 1, 2, 3, 5, 6, 12] },
                4 => { reset: Input, afr: L, af: [0, 1, 2, 3, 5, 6, 12] },
                5 => { reset: Input, afr: L, af: [0, 1, 2, 3, 5, 6, 12] },
                6 => { reset: Input, afr: L, af: [0, 1, 5, 6, 12] },
                7 => { reset: Input, afr: L, af: [1, 2, 12] },
                8 => { reset: Input, afr: H, af: [1, 2, 12] },
                9 => { reset: Input, afr: H, af: [1, 2, 12] },
                10 => { reset: Input, afr: H, af: [1, 2, 12] },
                11 => { reset: Input, afr: H, af: [1, 2, 5, 12] },
                12 => { reset: Input, afr: H, af: [1, 2, 5, 12] },
                13 => { reset: Input, afr: H, af: [1, 2, 5, 12] },
                14 => { reset: Input, afr: H, af: [1, 2, 5, 6, 12] },
                15 => { reset: Input, afr: H, af: [1, 2, 7, 12] },
            ],
        },
        {
            port: (F/f, 5, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 4, 5, 6] },
                1 => { reset: Input, afr: L, af: [1, 4, 5] },
                2 => { reset: Input, afr: L, af: [1, 2, 12] },
                3 => { reset: Input, afr: L, af: [1, 2, 12] },
                4 => { reset: Input, afr: L, af: [1, 2, 3, 12] },
                5 => { reset: Input, afr: L, af: [1, 2, 12] },
                6 => { reset: Input, afr: L, af: [1, 2, 4, 7, 12] },
                7 => { reset: Input, afr: L, af: [1, 2, 12] },
                8 => { reset: Input, afr: H, af: [1, 2, 12] },
                9 => { reset: Input, afr: H, af: [1, 2, 3, 5, 12] },
                10 => { reset: Input, afr: H, af: [1, 2, 3, 5, 12] },
                11 => { reset: Input, afr: H, af: [1, 2] },
                12 => { reset: Input, afr: H, af: [1, 2, 12] },
                13 => { reset: Input, afr: H, af: [1, 2, 12] },
                14 => { reset: Input, afr: H, af: [1, 2, 12] },
                15 => { reset: Input, afr: H, af: [1, 2, 12] },
            ],
        },
        {
            port: (G/g, 6, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 12] },
                1 => { reset: Input, afr: L, af: [1, 2, 12] },
                2 => { reset: Input, afr: L, af: [1, 2, 12] },
                3 => { reset: Input, afr: L, af: [1, 2, 12] },
                4 => { reset: Input, afr: L, af: [1, 2, 12] },
                5 => { reset: Input, afr: L, af: [1, 2, 12] },
                6 => { reset: Input, afr: L, af: [1, 12] },
                7 => { reset: Input, afr: L, af: [1, 12] },
                8 => { reset: Input, afr: H, af: [1] },
                9 => { reset: Input, afr: H, af: [1, 12] },
                10 => { reset: Input, afr: H, af: [1, 12] },
                11 => { reset: Input, afr: H, af: [1, 12] },
                12 => { reset: Input, afr: H, af: [1, 12] },
                13 => { reset: Input, afr: H, af: [1, 12] },
                14 => { reset: Input, afr: H, af: [1, 12] },
                15 => { reset: Input, afr: H, af: [1] },
            ],
        },
        {
            port: (H/h, 7, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 12] },
                1 => { reset: Input, afr: L, af: [1, 2, 12] },
                2 => { reset: Input, afr: L, af: [1] },
            ],
        },
    ],
});

#[cfg(feature = "gpio-f303")]
gpio!({
    pacs: [gpioa, gpiob, gpioc],
    ports: [
        {
            port: (A/a, 0, gpioa),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 10, 15] },
                1 => { reset: Input, afr: L, af: [0, 1, 3, 7, 9, 15] },
                2 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 15] },
                3 => { reset: Input, afr: L, af: [1, 3, 7, 9, 15] },
                4 => { reset: Input, afr: L, af: [2, 3, 5, 6, 7, 15] },
                5 => { reset: Input, afr: L, af: [1, 3, 5, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 8, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 8, 15] },
                8 => { reset: Input, afr: H, af: [0, 4, 5, 6, 7, 8, 10, 15] },
                9 => { reset: Input, afr: H, af: [3, 4, 5, 6, 7, 8, 9, 10, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 4, 6, 7, 8, 10, 11, 15] },
                11 => { reset: Input, afr: H, af: [6, 7, 8, 9, 10, 11, 12, 14, 15] },
                12 => { reset: Input, afr: H, af: [1, 6, 7, 8, 9, 10, 11, 14, 15] },
                13 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 5, 7, 10, 15] },
                14 => { reset: AF0<PushPull>, afr: H, af: [0, 3, 4, 5, 6, 7, 15] },
                15 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 2, 4, 5, 6, 7, 9, 15] },
            ],
        },
        {
            port: (B/b, 1, gpiob),
            pins: [
                0 => { reset: Input, afr: L, af: [2, 3, 4, 6, 15] },
                1 => { reset: Input, afr: L, af: [2, 3, 4, 6, 8, 15] },
                2 => { reset: Input, afr: L, af: [3, 15] },
                3 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
                4 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 4, 5, 6, 7, 10, 15] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 7, 10, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 6, 7, 10, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 4, 5, 7, 10, 15] },
                8 => { reset: Input, afr: H, af: [1, 2, 3, 4, 8, 9, 10, 12, 15] },
                9 => { reset: Input, afr: H, af: [1, 2, 4, 6, 8, 9, 10, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                11 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                12 => { reset: Input, afr: H, af: [3, 4, 5, 6, 7, 15] },
                13 => { reset: Input, afr: H, af: [3, 5, 6, 7, 15] },
                14 => { reset: Input, afr: H, af: [1, 3, 5, 6, 7, 15] },
                15 => { reset: Input, afr: H, af: [0, 1, 2, 4, 5, 15] },
            ],
        },
        {
            port: (C/c, 2, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1] },
                1 => { reset: Input, afr: L, af: [1] },
                2 => { reset: Input, afr: L, af: [1, 3] },
                3 => { reset: Input, afr: L, af: [1, 6] },
                4 => { reset: Input, afr: L, af: [1, 7] },
                5 => { reset: Input, afr: L, af: [1, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 2, 4, 6, 7] },
                7 => { reset: Input, afr: L, af: [1, 2, 4, 6, 7] },
                8 => { reset: Input, afr: H, af: [1, 2, 4, 7] },
                9 => { reset: Input, afr: H, af: [1, 2, 4, 5, 6] },
                10 => { reset: Input, afr: H, af: [1, 4, 5, 6, 7] },
                11 => { reset: Input, afr: H, af: [1, 4, 5, 6, 7] },
                12 => { reset: Input, afr: H, af: [1, 4, 5, 6, 7] },
                13 => { reset: Input, afr: H, af: [4] },
                14 => { reset: Input, afr: H, af: [] },
                15 => { reset: Input, afr: H, af: [] },
            ],
        },
        {
            port: (D/d, 3, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 7] },
                1 => { reset: Input, afr: L, af: [1, 4, 6, 7] },
                2 => { reset: Input, afr: L, af: [1, 2, 4, 5] },
                3 => { reset: Input, afr: L, af: [1, 2, 7] },
                4 => { reset: Input, afr: L, af: [1, 2, 7] },
                5 => { reset: Input, afr: L, af: [1, 7] },
                6 => { reset: Input, afr: L, af: [1, 2, 7] },
                7 => { reset: Input, afr: L, af: [1, 2, 7] },
                8 => { reset: Input, afr: H, af: [1, 7] },
                9 => { reset: Input, afr: H, af: [1, 7] },
                10 => { reset: Input, afr: H, af: [1, 7] },
                11 => { reset: Input, afr: H, af: [1, 7] },
                12 => { reset: Input, afr: H, af: [1, 2, 3, 7] },
                13 => { reset: Input, afr: H, af: [1, 2, 3] },
                14 => { reset: Input, afr: H, af: [1, 2, 3] },
                15 => { reset: Input, afr: H, af: [1, 2, 3, 6] },
            ],
        },
        {
            port: (E/e, 4, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 4, 7] },
                1 => { reset: Input, afr: L, af: [1, 4, 7] },
                2 => { reset: Input, afr: L, af: [0, 1, 2, 3] },
                3 => { reset: Input, afr: L, af: [0, 1, 2, 3] },
                4 => { reset: Input, afr: L, af: [0, 1, 2, 3] },
                5 => { reset: Input, afr: L, af: [0, 1, 2, 3] },
                6 => { reset: Input, afr: L, af: [0, 1] },
                7 => { reset: Input, afr: L, af: [1, 2] },
                8 => { reset: Input, afr: H, af: [1, 2] },
                9 => { reset: Input, afr: H, af: [1, 2] },
                10 => { reset: Input, afr: H, af: [1, 2] },
                11 => { reset: Input, afr: H, af: [1, 2] },
                12 => { reset: Input, afr: H, af: [1, 2] },
                13 => { reset: Input, afr: H, af: [1, 2] },
                14 => { reset: Input, afr: H, af: [1, 2, 6] },
                15 => { reset: Input, afr: H, af: [1, 2, 7] },
            ],
        },
        {
            port: (F/f, 5, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [4, 6] },
                1 => { reset: Input, afr: L, af: [4] },
                2 => { reset: Input, afr: L, af: [1] },
                4 => { reset: Input, afr: L, af: [1, 2] },
                6 => { reset: Input, afr: L, af: [1, 2, 4, 7] },
                9 => { reset: Input, afr: H, af: [1, 3, 5] },
                10 => { reset: Input, afr: H, af: [1, 3, 5] },
            ],
        },
    ],
});

#[cfg(feature = "gpio-f333")]
gpio!({
    pacs: [gpioa, gpiob, gpioc],
    ports: [
        {
            port: (A/a, 0, gpioa),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 3, 7, 15] },
                1 => { reset: Input, afr: L, af: [1, 3, 7, 9, 15] },
                2 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 15] },
                3 => { reset: Input, afr: L, af: [1, 3, 7, 9, 15] },
                4 => { reset: Input, afr: L, af: [2, 3, 5, 7, 15] },
                5 => { reset: Input, afr: L, af: [1, 3, 5, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 5, 6, 13, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 5, 6, 15] },
                8 => { reset: Input, afr: H, af: [0, 6, 7, 13, 15] },
                9 => { reset: Input, afr: H, af: [3, 6, 7, 9, 10, 13, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 6, 7, 8, 10, 13, 15] },
                11 => { reset: Input, afr: H, af: [6, 7, 9, 11, 12, 13, 15] },
                12 => { reset: Input, afr: H, af: [1, 6, 7, 8, 9, 11, 13, 15] },
                13 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 5, 7, 15] },
                14 => { reset: AF0<PushPull>, afr: H, af: [0, 3, 4, 6, 7, 15] },
                15 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 4, 5, 7, 9, 13, 15] },
            ],
        },
        {
            port: (B/b, 1, gpiob),
            pins: [
                0 => { reset: Input, afr: L, af: [2, 3, 6, 15] },
                1 => { reset: Input, afr: L, af: [2, 3, 6, 8, 13, 15] },
                2 => { reset: Input, afr: L, af: [3, 13, 15] },
                3 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 3, 5, 7, 10, 12, 13, 15] },
                4 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 5, 7, 10, 13, 15] },
                5 => { reset: Input, afr: L, af: [1, 2, 4, 5, 7, 10, 13, 15] },
                6 => { reset: Input, afr: L, af: [1, 3, 4, 7, 12, 13, 15] },
                7 => { reset: Input, afr: L, af: [1, 3, 4, 7, 10, 13, 15] },
                8 => { reset: Input, afr: H, af: [1, 3, 4, 7, 9, 12, 13, 15] },
                9 => { reset: Input, afr: H, af: [1, 4, 6, 7, 8, 9, 13, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 7, 13, 15] },
                11 => { reset: Input, afr: H, af: [1, 3, 7, 13, 15] },
                12 => { reset: Input, afr: H, af: [3, 6, 7, 13, 15] },
                13 => { reset: Input, afr: H, af: [3, 6, 7, 13, 15] },
                14 => { reset: Input, afr: H, af: [1, 3, 6, 7, 13, 15] },
                15 => { reset: Input, afr: H, af: [1, 2, 4, 13, 15] },
            ],
        },
        {
            port: (C/c, 2, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2] },
                1 => { reset: Input, afr: L, af: [1, 2] },
                2 => { reset: Input, afr: L, af: [1, 2] },
                3 => { reset: Input, afr: L, af: [1, 2, 6] },
                4 => { reset: Input, afr: L, af: [1, 2, 7] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                7 => { reset: Input, afr: L, af: [1, 2, 3] },
                8 => { reset: Input, afr: H, af: [1, 2, 3] },
                9 => { reset: Input, afr: H, af: [1, 2, 3] },
                10 => { reset: Input, afr: H, af: [1, 7] },
                11 => { reset: Input, afr: H, af: [1, 3, 7] },
                12 => { reset: Input, afr: H, af: [1, 3, 7] },
                13 => { reset: Input, afr: H, af: [4] },
                14 => { reset: Input, afr: H, af: [] },
                15 => { reset: Input, afr: H, af: [] },
            ],
        },
        {
            port: (D/d, 3, gpioc),
            pins: [
                2 => { reset: Input, afr: L, af: [1, 2] },
            ],
        },
        {
            port: (F/f, 5, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [6] },
                1 => { reset: Input, afr: L, af: [] },
            ],
        },
    ],
});

#[cfg(feature = "gpio-f373")]
gpio!({
    pacs: [gpioa, gpiob, gpioc, gpiod],
    ports: [
        {
            port: (A/a, 0, gpioa),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 3, 7, 8, 11, 15] },
                1 => { reset: Input, afr: L, af: [0, 1, 2, 3, 6, 7, 9, 11, 15] },
                2 => { reset: Input, afr: L, af: [1, 2, 3, 6, 7, 8, 9, 11, 15] },
                3 => { reset: Input, afr: L, af: [1, 2, 3, 6, 7, 9, 11, 15] },
                4 => { reset: Input, afr: L, af: [2, 3, 5, 6, 7, 10, 15] },
                5 => { reset: Input, afr: L, af: [1, 3, 5, 7, 9, 10, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 5, 8, 9, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 5, 8, 9, 15] },
                8 => { reset: Input, afr: H, af: [0, 2, 4, 5, 7, 10, 15] },
                9 => { reset: Input, afr: H, af: [2, 3, 4, 5, 7, 9, 10, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 4, 5, 7, 9, 10, 15] },
                11 => { reset: Input, afr: H, af: [2, 5, 6, 7, 8, 9, 10, 14, 15] },
                12 => { reset: Input, afr: H, af: [1, 2, 6, 7, 8, 9, 10, 14, 15] },
                13 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 2, 3, 5, 6, 7, 10, 15] },
                14 => { reset: AF0<PushPull>, afr: H, af: [0, 3, 4, 10, 15] },
                15 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 4, 5, 6, 10, 15] },
            ],
        },
        {
            port: (B/b, 1, gpiob),
            pins: [
                0 => { reset: Input, afr: L, af: [2, 3, 5, 10, 15] },
                1 => { reset: Input, afr: L, af: [2, 3, 15] },
                2 => { reset: Input, afr: L, af: [15] },
                3 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 5, 6, 7, 9, 10, 15] },
                4 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 2, 3, 5, 6, 7, 9, 10, 15] },
                5 => { reset: Input, afr: L, af: [1, 2, 4, 5, 6, 7, 10, 11, 15] },
                6 => { reset: Input, afr: L, af: [1, 2, 3, 4, 7, 9, 10, 11, 15] },
                7 => { reset: Input, afr: L, af: [1, 2, 3, 4, 7, 9, 10, 11, 15] },
                8 => { reset: Input, afr: H, af: [1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 15] },
                9 => { reset: Input, afr: H, af: [1, 2, 4, 5, 6, 7, 8, 9, 11, 15] },
                10 => { reset: Input, afr: H, af: [1, 3, 5, 6, 7, 15] },
                14 => { reset: Input, afr: H, af: [1, 3, 5, 7, 9, 15] },
                15 => { reset: Input, afr: H, af: [0, 1, 2, 3, 5, 9, 15] },
            ],
        },
        {
            port: (C/c, 2, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2] },
                1 => { reset: Input, afr: L, af: [1, 2] },
                2 => { reset: Input, afr: L, af: [1, 2, 5] },
                3 => { reset: Input, afr: L, af: [1, 2, 5] },
                4 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                5 => { reset: Input, afr: L, af: [1, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 2, 5] },
                7 => { reset: Input, afr: L, af: [1, 2, 5] },
                8 => { reset: Input, afr: H, af: [1, 2, 5] },
                9 => { reset: Input, afr: H, af: [1, 2, 5] },
                10 => { reset: Input, afr: H, af: [1, 2, 6, 7] },
                11 => { reset: Input, afr: H, af: [1, 2, 6, 7] },
                12 => { reset: Input, afr: H, af: [1, 2, 6, 7] },
                13 => { reset: Input, afr: H, af: [] },
                14 => { reset: Input, afr: H, af: [] },
                15 => { reset: Input, afr: H, af: [] },
            ],
        },
        {
            port: (D/d, 3, gpiod),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 7] },
                1 => { reset: Input, afr: L, af: [1, 2, 7] },
                2 => { reset: Input, afr: L, af: [1, 2] },
                3 => { reset: Input, afr: L, af: [1, 5, 7] },
                4 => { reset: Input, afr: L, af: [1, 5, 7] },
                5 => { reset: Input, afr: L, af: [1, 7] },
                6 => { reset: Input, afr: L, af: [1, 5, 7] },
                7 => { reset: Input, afr: L, af: [1, 5, 7] },
                8 => { reset: Input, afr: H, af: [1, 3, 5, 7] },
                9 => { reset: Input, afr: H, af: [1, 3, 7] },
                10 => { reset: Input, afr: H, af: [1, 7] },
                11 => { reset: Input, afr: H, af: [1, 7] },
                12 => { reset: Input, afr: H, af: [1, 2, 3, 7] },
                13 => { reset: Input, afr: H, af: [1, 2, 3] },
                14 => { reset: Input, afr: H, af: [1, 2, 3] },
                15 => { reset: Input, afr: H, af: [1, 2, 3] },
            ],
        },
        {
            port: (E/e, 4, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2, 7] },
                1 => { reset: Input, afr: L, af: [1, 7] },
                2 => { reset: Input, afr: L, af: [0, 1, 3] },
                3 => { reset: Input, afr: L, af: [0, 1, 3] },
                4 => { reset: Input, afr: L, af: [0, 1, 3] },
                5 => { reset: Input, afr: L, af: [0, 1, 3] },
                6 => { reset: Input, afr: L, af: [0, 1] },
                7 => { reset: Input, afr: L, af: [1] },
                8 => { reset: Input, afr: H, af: [1] },
                9 => { reset: Input, afr: H, af: [1] },
                10 => { reset: Input, afr: H, af: [1] },
                11 => { reset: Input, afr: H, af: [1] },
                12 => { reset: Input, afr: H, af: [1] },
                13 => { reset: Input, afr: H, af: [1] },
                14 => { reset: Input, afr: H, af: [1] },
                15 => { reset: Input, afr: H, af: [1, 7] },
            ],
        },
        {
            port: (F/f, 5, gpioc),
            pins: [
                0 => { reset: Input, afr: L, af: [4] },
                1 => { reset: Input, afr: L, af: [4] },
                2 => { reset: Input, afr: L, af: [1, 4] },
                4 => { reset: Input, afr: L, af: [1] },
                6 => { reset: Input, afr: L, af: [1, 2, 4, 5, 7] },
                7 => { reset: Input, afr: L, af: [1, 4, 7] },
                9 => { reset: Input, afr: H, af: [1, 2] },
                10 => { reset: Input, afr: H, af: [1] },
            ],
        },
    ],
});
