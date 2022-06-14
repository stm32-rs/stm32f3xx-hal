//! Digital-to-Analog Converter

// Based on stm32hal by David-OConnor

use crate::rcc::{Enable, Reset, APB1};

#[cfg(any(feature = "svd-f302",))]
use crate::pac::DAC;

#[cfg(any(
    feature = "svd-f301",
    feature = "svd-f303",
    feature = "svd-f373",
    feature = "svd-f3x4",
))]
use crate::pac::DAC1 as DAC;

/// Represents a Digital to Analog Converter (DAC) peripheral.
pub struct Dac {
    regs: DAC,
}

impl Dac {
    /// Initializes the DAC peripheral.
    pub fn new(regs: DAC, apb1: &mut APB1) -> Self {
        DAC::enable(apb1);
        DAC::reset(apb1);

        // Enable channel 1.
        regs.cr.modify(|_, w| w.en1().set_bit());

        Self { regs }
    }

    /// Writes a sample to the Channel 1 output register.
    ///
    /// Only the low 12 bits of `data` will be used, the rest is ignored.
    pub fn write_data(&mut self, data: u16) {
        self.regs.dhr12r1.write(|w| {
            #[allow(unused_unsafe)]
            unsafe {
                w.dacc1dhr().bits(data)
            }
        })
    }
}
