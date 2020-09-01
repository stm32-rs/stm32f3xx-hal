//! Configure the internal DAC on the stm32f3xx.
//! Incomplete, but includes basic operation.
//!
//! You should have the approprite dac pin set up as an analog input, to prevent
//! parasitic power consumption. For example:
//! ```rust
//! let _dac1_pin = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
//! let _dac2_pin = gpioa.pa5.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
//! ```
//!


// todo: DAC trait that will go in a PR to embeddd hal.
// [PR](https://github.com/rust-embedded/embedded-hal/pull/247)

//! Digital-analog conversion traits
//!
//! A trait used to identify a digital-to-analog converter, and its
//! most fundamental features.

use crate::{pac, rcc::APB1};

pub trait DacTrait {
    /// Enable the DAC.
    fn enable(&mut self, p: &mut APB1);  // todo: generalize periph clock

    /// Disable the DAC.
    fn disable(&mut self);

    /// Output a constant signal, given a bit word.
    fn set_value(&mut self, value: u32);

    /// Set a constant signal, given a voltage.
    fn set_voltage(&mut self, volts: f32);
}

#[derive(Clone, Copy, Debug)]
pub enum DacId {
    One,
    Two
}

#[derive(Clone, Copy, Debug)]
pub enum DacBits {
    EightR,
    TwelveL,
    TwelveR,
}

pub struct Dac {
    regs: pac::DAC,
    id: DacId,
    bits: DacBits,
    vref: f32,
}

impl Dac {
    /// Create a new DAC instances
    pub fn new(regs: pac::DAC, id: DacId, bits: DacBits, vref: f32) -> Self {
        Self { regs, id, bits, vref }
    }

    //// Independent trigger with single LFSR generation
    // pub fn trigger_lfsr(&mut self) {
    //     // todo: incomplete
    //     match self.id {
    //         DacId::One => {
    //             self.regs.cr.modify(|_, w| w.ten1().set_bit());
    //             self.regs.cr.modify(|_, w| unsafe { w.tsel1().bits(0)});
    //             self.regs.cr.modify(|_, w| unsafe { w.wave1().bits(0b01)});
    //             self.regs.cr.modify(|_, w| unsafe { w.mamp1().bits(0)});
    //         }
    //         DacId::Two => {
    //             self.regs.cr.modify(|_, w| w.ten2().set_bit());
    //             self.regs.cr.modify(|_, w| unsafe { w.tsel2().bits(0)});
    //             self.regs.cr.modify(|_, w| unsafe { w.wave2().bits(0b01)});
    //             self.regs.cr.modify(|_, w| unsafe { w.mamp2().bits(0)});
    //         }
    //     }
    //
    // }
    //
    // /// Independent trigger with single triangle generation
    // pub fn trigger_triangle(&mut self) {
    //     // todo: incomplete
    //     match self.id {
    //         DacId::One => {
    //             self.regs.cr.modify(|_, w| w.ten1().set_bit());
    //             self.regs.cr.modify(|_, w| unsafe { w.tsel1().bits(0)});
    //             // self.regs.cr.modify(|_, w| unsafe { w.wave1().bits(0b1x)});
    //             self.regs.cr.modify(|_, w| unsafe { w.mamp1().bits(0)});
    //         }
    //         DacId::Two => {
    //             self.regs.cr.modify(|_, w| w.ten2().set_bit());
    //             self.regs.cr.modify(|_, w| unsafe { w.tsel2().bits(0)});
    //             // self.regs.cr.modify(|_, w| unsafe { w.wave2().bits(0b1x)});
    //             self.regs.cr.modify(|_, w| unsafe { w.mamp2().bits(0)});
    //         }
    //     }
    // }
}

impl DacTrait for Dac {
    /// Enable the DAC.
    fn enable(&mut self, apb1: &mut APB1) {
        match self.id {
            DacId::One => {
                apb1.enr().modify(|_, w| w.dac1en().set_bit());
                self.regs.cr.modify(|_, w| w.en1().set_bit());
                
            },
            DacId::Two => {
                apb1.enr().modify(|_, w| w.dac2en().set_bit());
                self.regs.cr.modify(|_, w| w.en2().set_bit());
            }
        }
    }

    fn disable(&mut self) {
        match self.id {
            DacId::One => self.regs.cr.modify(|_, w| w.en1().clear_bit()),
            DacId::Two => self.regs.cr.modify(|_, w| w.en2().clear_bit()),
        }
    }

    /// Set the DAC value as an integer.
    fn set_value(&mut self, val: u32) {
        match self.id {
            DacId::One => {
                match self.bits {
                    DacBits::EightR => self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val)}),
                    DacBits::TwelveL => self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val)}),
                    DacBits::TwelveR => self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val)}),
                }
            }
            DacId::Two => {
                match self.bits {
                    DacBits::EightR => self.regs.dhr8r2.modify(|_, w| unsafe { w.bits(val)}),
                    DacBits::TwelveL => self.regs.dhr12l2.modify(|_, w| unsafe { w.bits(val)}),
                    DacBits::TwelveR => self.regs.dhr12r2.modify(|_, w| unsafe { w.bits(val)}),
                }
            }
        }
    }

    /// Set the DAC voltage. `v` is in Volts.
    fn set_voltage(&mut self, volts: f32) {
        let val = match self.bits {
            DacBits::EightR => ((volts / self.vref) * 255.) as u32,
            DacBits::TwelveL => ((volts / self.vref) * 4_095.) as u32,
            DacBits::TwelveR => ((volts / self.vref) * 4_095.) as u32,
        };

        self.set_value(val);
    }
}