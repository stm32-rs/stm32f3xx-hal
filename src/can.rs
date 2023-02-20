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
//! [examples/can.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.2/examples/can.rs

use crate::gpio::{gpioa, gpiob};
use crate::gpio::{PushPull, AF9};
use crate::pac;

use crate::rcc::{Enable, Reset, APB1};

use bxcan::RegisterBlock;

use cfg_if::cfg_if;

/// Marker trait for pins (with specific AF mode) that can be used as a CAN RX pin.
pub trait RxPin: crate::private::Sealed {}

/// Marker trait for pins (with specific AF mode) that can be used as a CAN TX pin.
pub trait TxPin: crate::private::Sealed {}

impl RxPin for gpioa::PA11<AF9<PushPull>> {}
impl TxPin for gpioa::PA12<AF9<PushPull>> {}
impl RxPin for gpiob::PB8<AF9<PushPull>> {}
impl TxPin for gpiob::PB9<AF9<PushPull>> {}

cfg_if! {
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e", feature = "gpio-f373"))] {
        use crate::gpio::{gpiod, AF7};
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
    /// Create a new `bxcan::CAN` instance.
    pub fn new(can: pac::CAN, tx: Tx, rx: Rx, apb1: &mut APB1) -> Self {
        pac::CAN::enable(apb1);
        pac::CAN::reset(apb1);

        Can { can, tx, rx }
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
