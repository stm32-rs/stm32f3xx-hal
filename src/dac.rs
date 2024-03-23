//! Digital-to-Analog Converter

// Based on stm32hal by David-OConnor

use crate::rcc::{Enable, Reset, APB1};

use crate::pac::DAC1;

/// Represents a Digital to Analog Converter (DAC) peripheral.
pub struct Dac {
    regs: DAC1,
}

impl Dac {
    /// Initializes the DAC peripheral.
    pub fn new(regs: DAC1, apb1: &mut APB1) -> Self {
        DAC1::enable(apb1);
        DAC1::reset(apb1);

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
            // SAFETY: Direct write to register for easier sharing between different stm32f3xx svd
            // generated API
            unsafe {
                w.dacc1dhr().bits(data)
            }
        });
    }
}
