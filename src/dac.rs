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

use core::fmt;

use crate::{pac, rcc::APB1};

pub trait SingleChannelDac {
    /// Error type returned by DAC methods
    type Error;
    type Word;

    /// Output a constant signal, given a bit word.
    fn try_set_value(&mut self, value: Self::Word) -> Result<(), Self::Error>;
}

#[derive(Clone, Copy, Debug)]
pub enum DacId {
    One,
    Two,
}

#[derive(Clone, Copy, Debug)]
pub enum DacBits {
    EightR,
    TwelveL,
    TwelveR,
}

#[derive(Clone, Copy, Debug)]
pub enum Trigger {
    Tim6,
    Tim3_8,
    Tim7,
    Tim15,
    Tim2,
    Tim4, // Not available on DAC 2.
    Exti9,
    Swtrig,
}

impl Trigger {
    pub fn bits(&self) -> u8 {
        match self {
            Self::Tim6 => 0b000,
            Self::Tim3_8 => 0b001,
            Self::Tim7 => 0b010,
            Self::Tim15 => 0b011,
            Self::Tim2 => 0b100,
            Self::Tim4 => 0b101,
            Self::Exti9 => 0b110,
            Self::Swtrig => 0b111,
        }
    }
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
        Self {
            regs,
            id,
            bits,
            vref,
        }
    }

    /// Enable the DAC.
    pub fn enable(&mut self, apb1: &mut APB1) {
        match self.id {
            DacId::One => {
                apb1.enr().modify(|_, w| w.dac1en().set_bit());
                self.regs.cr.modify(|_, w| w.en1().set_bit());
            }
            DacId::Two => {
                apb1.enr().modify(|_, w| w.dac2en().set_bit());
                self.regs.cr.modify(|_, w| w.en2().set_bit());
            }
        }
    }

    /// Disable the DAC
    pub fn disable(&mut self, apb1: &mut APB1) {
        match self.id {
            DacId::One => {
                self.regs.cr.modify(|_, w| w.en1().clear_bit());
                apb1.enr().modify(|_, w| w.dac1en().clear_bit());
            }
            DacId::Two => {
                self.regs.cr.modify(|_, w| w.en2().clear_bit());
                apb1.enr().modify(|_, w| w.dac2en().clear_bit());
            }
        }
    }

    /// Set the DAC value as an integer.
    pub fn set_value(&mut self, val: u32) {
        match self.id {
            DacId::One => match self.bits {
                DacBits::EightR => self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            DacId::Two => match self.bits {
                DacBits::EightR => self.regs.dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
            },
        }
    }

    /// Set the DAC voltage. `v` is in Volts.
    pub fn set_voltage(&mut self, volts: f32) {
        let val = match self.bits {
            DacBits::EightR => ((volts / self.vref) * 255.) as u32,
            DacBits::TwelveL => ((volts / self.vref) * 4_095.) as u32,
            DacBits::TwelveR => ((volts / self.vref) * 4_095.) as u32,
        };

        self.set_value(val);
    }

    // Select and activate a trigger. See f303 Reference manual, section 16.5.4.
    pub fn set_trigger(&mut self, trigger: Trigger) {
        match self.id {
            DacId::One => {
                self.regs.cr.modify(|_, w| w.ten1().set_bit());
                self.regs
                    .cr
                    .modify(|_, w| unsafe { w.tsel1().bits(trigger.bits()) });
            }
            DacId::Two => {
                self.regs.cr.modify(|_, w| w.ten2().set_bit());
                self.regs.cr.modify(|_, w| w.tsel2().bits(trigger.bits()));
            }
        }
    }

    /// Independent trigger with single LFSR generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_lfsr(&mut self, trigger: Trigger, data: u32) {
        // todo: This may not be correct.
        match self.id {
            DacId::One => {
                self.regs.cr.modify(|_, w| unsafe { w.wave1().bits(0b01) });
                self.regs.cr.modify(|_, w| w.mamp1().bits(0b01));
            }
            DacId::Two => {
                self.regs.cr.modify(|_, w| unsafe { w.wave2().bits(0b01) });
                self.regs.cr.modify(|_, w| w.mamp2().bits(0b01));
            }
        }
        self.set_trigger(trigger);
        self.set_value(data);
    }

    /// Independent trigger with single triangle generation
    /// See f303 Reference Manual section 16.5.2
    pub fn trigger_triangle(&mut self, trigger: Trigger, data: u32) {
        // todo: This may not be correct.
        match self.id {
            DacId::One => {
                self.regs.cr.modify(|_, w| unsafe { w.wave1().bits(0b10) });
                self.regs.cr.modify(|_, w| w.mamp1().bits(0b10));
            }
            DacId::Two => {
                self.regs.cr.modify(|_, w| unsafe { w.wave2().bits(0b10) });
                self.regs.cr.modify(|_, w| w.mamp2().bits(0b10));
            }
        }
        self.set_trigger(trigger);
        self.set_value(data);
    }
}

impl fmt::Debug for Dac {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Dac")
            .field("id", &self.id)
            .field("bits", &self.bits)
            .field("vret", &self.vref)
            .finish()
    }
}

pub struct DacError {}

impl SingleChannelDac for Dac {
    type Error = DacError;
    type Word = u32;

    /// Set the DAC value as an integer.
    fn try_set_value(&mut self, val: u32) -> Result<(), DacError> {
        self.set_value(val);
        Ok(())
    }
}
