//! Controller Area Network.
//!
//! CAN is currently not enabled by default, and
//! can be enabled by the `can` feature.
//!
//! It is a implementation of the [`bxcan`][can] traits.
//!
//! [can]: bxcan
//!
//! A usage example of the can peripheral can be found at [examples/can.rs]
//!
//! [examples/can.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.7.0/examples/can.rs

use crate::gpio::{gpioa, gpiob};
use crate::gpio::{PushPull, AF7, AF9};
use crate::pac;

use crate::rcc::APB1;

pub use bxcan;
use bxcan::RegisterBlock;

use cfg_if::cfg_if;

mod sealed {
    pub trait Sealed {}
}

/// Marker trait for pins (with specific AF mode) that can be used as a CAN RX pin.
pub trait RxPin: sealed::Sealed {}

/// Marker trait for pins (with specific AF mode) that can be used as a CAN TX pin.
pub trait TxPin: sealed::Sealed {}

cfg_if! {
    if #[cfg(any(feature = "gpio-f302", feature = "gpio-f303"))] {
        use crate::gpio::gpiod;

        impl sealed::Sealed for gpioa::PA11<AF9<PushPull>> {}
        impl RxPin for gpioa::PA11<AF9<PushPull>> {}
        impl sealed::Sealed for gpioa::PA12<AF9<PushPull>> {}
        impl TxPin for gpioa::PA12<AF9<PushPull>> {}

        impl sealed::Sealed for gpiob::PB8<AF9<PushPull>> {}
        impl RxPin for gpiob::PB8<AF9<PushPull>> {}
        impl sealed::Sealed for gpiob::PB9<AF9<PushPull>> {}
        impl TxPin for gpiob::PB9<AF9<PushPull>> {}

        impl sealed::Sealed for gpiod::PD0<AF7<PushPull>> {}
        impl RxPin for gpiod::PD0<AF7<PushPull>> {}
        impl sealed::Sealed for gpiod::PD1<AF7<PushPull>> {}
        impl TxPin for gpiod::PD1<AF7<PushPull>> {}
    }
}

/// Struct representing a CAN peripheral and its configured TX and RX pins.
///
/// See [`bxcan::Instance`] for more information on how to use the CAN interface.
pub struct Can<TX, RX> {
    _can: stm32::CAN,
    _tx: TX,
    _rx: RX,
}

impl<TX, RX> Can<TX, RX>
where
    TX: TxPin,
    RX: RxPin,
{
    /// Create a new CAN instance, using the specified TX and RX pins.
    ///
    /// Note: this does not actually initialize the CAN bus.
    /// You will need to first call [`bxcan::Can::new`] and  set the bus configuration and filters
    /// before the peripheral can be enabled.
    /// See the CAN example, for a more thorough example of the full setup process.
    pub fn new(can: pac::CAN, tx: TX, rx: RX, apb1: &mut APB1) -> bxcan::Can<Self> {
        apb1.enr().modify(|_, w| w.canen().enabled());
        apb1.rstr().modify(|_, w| w.canrst().set_bit());
        apb1.rstr().modify(|_, w| w.canrst().clear_bit());

        bxcan::Can::new(Can {
            _can: can,
            _tx: tx,
            _rx: rx,
        })
    }
}

unsafe impl<TX, RX> bxcan::Instance for Can<TX, RX> {
    const REGISTERS: *mut RegisterBlock = stm32::CAN::ptr() as *mut _;
}

unsafe impl<TX, RX> bxcan::FilterOwner for Can<TX, RX> {
    const NUM_FILTER_BANKS: u8 = 28;
}
