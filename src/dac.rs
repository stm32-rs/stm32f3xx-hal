//! Configure the internal DAC on the stm32f3xx.
//! Incomplete, but includes basic operation.

use crate::{
    gpio::{
        gpioa::{PA4, PA5},
        Analog,
    },
    pac,
    rcc::APB1,
};
use core::convert::Infallible;

/// Trait representing a single-channel digital-to-analog converter (DAC).
pub trait SingleChannelDac<Word> {
    /// Error type returned by DAC methods
    type Error;

    /// Output a constant signal, given a bit word.
    fn try_set_value(&mut self, value: Word) -> Result<(), Self::Error>;
}

/// This is an abstraction to select the correct DAC channel for a given
/// (analog) output pin.
pub trait Pin {
    const CHANNEL: Channel;
}

impl Pin for PA4<Analog> {
    const CHANNEL: Channel = Channel::One;
}

impl Pin for PA5<Analog> {
    const CHANNEL: Channel = Channel::Two;
}

#[derive(Clone, Copy)]
/// Select the channel
pub enum Channel {
    /// Channel 1
    One,
    /// Channel 2
    Two,
}

#[derive(Clone, Copy)]
/// Three options are available to set DAC precision.
pub enum DacBits {
    /// Eight bit precision, right-aligned.
    EightR,
    /// 12-bit precision, left-aligned.
    TwelveL,
    /// 12-bit precision, right-aligned.
    TwelveR,
}

#[derive(Clone, Copy)]
/// Select an external event to trigger the DAC.
pub enum Trigger {
    Timer2,
    Timer3Or8,
    Timer4,
    Timer6,
    Timer7,
    Timer15,
    Exti9,
    SoftwareTrigger,
}

impl Trigger {
    fn bits(&self) -> u8 {
        match self {
            Self::Timer6 => 0b000,
            Self::Timer3Or8 => 0b001,
            Self::Timer7 => 0b010,
            Self::Timer15 => 0b011,
            Self::Timer2 => 0b100,
            Self::Timer4 => 0b101,
            Self::Exti9 => 0b110,
            Self::SoftwareTrigger => 0b111,
        }
    }
}

pub struct Dac {
    regs: pac::DAC,
    channel: Channel,
    bits: DacBits,
    vref: f32,
}

impl Dac {
    /// Create a new DAC instance
    pub fn new<P: Pin>(regs: pac::DAC, _pin: P, bits: DacBits, vref: f32) -> Self {
        Self {
            regs,
            channel: P::CHANNEL,
            bits,
            vref,
        }
    }

    /// Enable the DAC.
    pub fn enable(&mut self, apb1: &mut APB1) {
        apb1.enr().modify(|_, w| w.dac1en().enabled());
        self.regs.cr.modify(|_, w| match self.channel {
            Channel::One => w.en1().enabled(),
            Channel::Two => w.en2().enabled(),
        });
    }

    /// Disable the DAC
    pub fn disable(&mut self, apb1: &mut APB1) {
        self.regs.cr.modify(|_, w| match self.channel {
            Channel::One => w.en1().disabled(),
            Channel::Two => w.en2().disabled(),
        });
        apb1.enr().modify(|_, w| w.dac1en().disabled());
    }

    /// Set the DAC value as an integer.
    pub fn set_value(&mut self, val: u32) {
        match self.channel {
            Channel::One => match self.bits {
                DacBits::EightR => self.regs.dhr8r1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l1.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r1.modify(|_, w| unsafe { w.bits(val) }),
            },
            Channel::Two => match self.bits {
                DacBits::EightR => self.regs.dhr8r2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveL => self.regs.dhr12l2.modify(|_, w| unsafe { w.bits(val) }),
                DacBits::TwelveR => self.regs.dhr12r2.modify(|_, w| unsafe { w.bits(val) }),
            },
        }
    }

    /// Set the DAC voltage.
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
        self.regs.cr.modify(|_, w| match self.channel {
            Channel::One => {
                w.ten1().enabled();
                unsafe { w.tsel1().bits(trigger.bits()) }
            }
            Channel::Two => {
                w.ten2().enabled();
                w.tsel2().bits(trigger.bits())
            }
        });
    }

    /// Takes the value stored via set_value or set_voltage and converts it to output on the Pin.
    pub fn trigger_software_trigger(&mut self) {
        self.regs.swtrigr.write(|w| match self.channel {
            Channel::One => w.swtrig1().enabled(),
            Channel::Two => w.swtrig2().enabled(),
        });
    }
}

impl SingleChannelDac<u32> for Dac {
    type Error = Infallible;

    /// Set the DAC value as an integer.
    fn try_set_value(&mut self, val: u32) -> Result<(), Infallible> {
        self.set_value(val);
        Ok(())
    }
}
