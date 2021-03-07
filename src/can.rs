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

use crate::gpio::gpioa;
use crate::gpio::{PushPull, AF9};
use crate::pac;
use crate::rcc::APB1;
use crate::stm32;
use bxcan::RegisterBlock;
use nb::{self, Error};

mod sealed {
    pub trait Sealed {}
}

pub struct Can {
    _can: stm32::CAN,
}

impl Can {
    pub fn new(can: stm32::CAN, apb1: &mut APB1) -> Self {
        apb1.enr().modify(|_, w| w.canen().enabled());
        apb1.rstr().modify(|_, w| w.canrst().set_bit());
        apb1.rstr().modify(|_, w| w.canrst().clear_bit());

        Can { _can: can }
    }
}

unsafe impl bxcan::Instance for Can {
    const REGISTERS: *mut RegisterBlock = stm32::CAN::ptr() as *mut _;
}

unsafe impl bxcan::FilterOwner for Can {
    const NUM_FILTER_BANKS: u8 = 28;
}
