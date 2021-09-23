//Based on stm32hal by David-OConnor

use cortex_m::asm;

use crate::{
    gpio::{self, Analog},
    pac::{dac1, DAC1},
    rcc::{Rcc, Clocks, AHB, APB1},
};

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
use crate::pac::DMA1::{self};

use crate::pac::{self, rcc, RCC};

pub enum DacChannel {
    One,
    Two,
}

/// Represents a Digital to Analog Converter (DAC) peripheral.
pub struct Dac {
    regs: DAC1,
}

// todo: Calculate the VDDA vref, as you do with onboard ADCs!
impl Dac {
    /// Initialize a DAC peripheral, including  enabling and resetting

    pub fn new(regs: DAC1, abp1: &mut APB1) -> Self {
    
        abp1.enr().modify(|_, w| w.dac1en().set_bit());
        abp1.rstr().modify(|_, w| w.dac1rst().set_bit());
        abp1.rstr().modify(|_, w| w.dac1rst().clear_bit());

        Self { regs }
    }

    pub fn enable_channel(&mut self, channel: DacChannel) {
        let cr = &self.regs.cr;

        cr.modify(|_, w| match channel {
            DacChannel::One => w.en1().set_bit(),
            DacChannel::Two => w.en2().set_bit(),
        });
    }

    /// the data parameter MUST be a 12-bit value -- values outside that range will result in misbehavior
    pub fn write_data(&mut self, data: u16) {
        
        self.regs.dhr12r1.write(|w| w.dacc1dhr().bits(data))
    }
}
