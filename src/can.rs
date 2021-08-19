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
//! [examples/can.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.8.0/examples/can.rs

use crate::gpio::{gpioa, gpiob};
use crate::gpio::{PushPull, AF7, AF9};
use crate::pac;

use crate::rcc::APB1;

pub use bxcan;
use bxcan::RegisterBlock;

use cfg_if::cfg_if;

/// Marker trait for pins (with specific AF mode) that can be used as a CAN RX pin.
pub trait RxPin: crate::private::Sealed {}

/// Marker trait for pins (with specific AF mode) that can be used as a CAN TX pin.
pub trait TxPin: crate::private::Sealed {}

cfg_if! {
    if #[cfg(any(feature = "gpio-f302", feature = "gpio-f303"))] {
        use crate::gpio::gpiod;

        impl RxPin for gpioa::PA11<AF9<PushPull>> {}
        impl TxPin for gpioa::PA12<AF9<PushPull>> {}

        impl RxPin for gpiob::PB8<AF9<PushPull>> {}
        impl TxPin for gpiob::PB9<AF9<PushPull>> {}

        impl RxPin for gpiod::PD0<AF7<PushPull>> {}
        impl TxPin for gpiod::PD1<AF7<PushPull>> {}
    }
}

/// Struct representing a CAN peripheral and its configured TX and RX pins.
///
/// See [`bxcan::Instance`] for more information on how to use the CAN interface.
pub struct Can<Tx, Rx> {
    can: pac::CAN,
    tx: Tx,
    rx: Rx,
}

impl<Tx, Rx> Can<Tx, Rx>
where
    Tx: TxPin,
    Rx: RxPin,
{
    /// Create a new CAN instance, using the specified TX and RX pins.
    ///
    /// Note: this does not actually initialize the CAN bus.
    /// You will need to first call [`bxcan::Can::new`] and  set the bus configuration and filters
    /// before the peripheral can be enabled.
    /// See the CAN example, for a more thorough example of the full setup process.
    pub fn new(can: pac::CAN, tx: Tx, rx: Rx, apb1: &mut APB1) -> bxcan::Can<Self> {
        apb1.enr().modify(|_, w| w.canen().enabled());
        apb1.rstr().modify(|_, w| w.canrst().set_bit());
        apb1.rstr().modify(|_, w| w.canrst().clear_bit());

        bxcan::Can::new(Can { can, tx, rx })
    }

    /// Releases the CAN peripheral and associated pins
    pub fn free(self) -> (pac::CAN, Tx, Rx) {
        (self.can, self.tx, self.rx)
    }
}

unsafe impl<Tx, Rx> bxcan::Instance for Can<Tx, Rx> {
    const REGISTERS: *mut RegisterBlock = pac::CAN::ptr() as *mut _;
}

unsafe impl<Tx, Rx> bxcan::FilterOwner for Can<Tx, Rx> {
    const NUM_FILTER_BANKS: u8 = 28;
}
