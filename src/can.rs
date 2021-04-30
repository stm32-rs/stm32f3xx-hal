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
use crate::stm32;
use bxcan::RegisterBlock;
use nb::{self, Error};

use cfg_if::cfg_if;

mod sealed {
    pub trait Sealed {}
}

pub trait RxPin: sealed::Sealed {}

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
    pub fn new(can: stm32::CAN, tx: TX, rx: RX, apb1: &mut APB1) -> Self {
        apb1.enr().modify(|_, w| w.canen().enabled());
        apb1.rstr().modify(|_, w| w.canrst().set_bit());
        apb1.rstr().modify(|_, w| w.canrst().clear_bit());

        Can {
            _can: can,
            _tx: tx,
            _rx: rx,
        }
    }
}

unsafe impl<TX, RX> bxcan::Instance for Can<TX, RX> {
    const REGISTERS: *mut RegisterBlock = stm32::CAN::ptr() as *mut _;
}

unsafe impl<TX, RX> bxcan::FilterOwner for Can<TX, RX> {
    const NUM_FILTER_BANKS: u8 = 28;
}
