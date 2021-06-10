//! # Delays
//!
//! Implementations for the [`DelayMs`] and [`DelayUs`] traits
//!
//! [DelayMs]: embedded_hal::blocking::delay::DelayMs
//! [DelayUs]: embedded_hal::blocking::delay::DelayUs

use core::convert::From;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::hal::blocking::delay::{DelayMs, DelayUs};
use crate::rcc::Clocks;

/// System timer (SysTick) as a delay provider
pub struct Delay {
    clocks: Clocks,
    syst: SYST,
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    ///
    /// # Limitations
    ///
    /// Depending on the core clock, this delay provider
    /// can delay between 1 minute (for 72 Mhz) up to almost 9 minutes (for 8 Mhz).
    /// Higher input values will be capped to these limits.
    ///
    /// For accuracy purposes and because this is a blocking, busy-waiting function,
    /// if delays in the second to minute range are needed, use timers instead.
    pub fn new(mut syst: SYST, clocks: Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);

        Delay { clocks, syst }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms.saturating_mul(1_000));
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32::from(ms));
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32::from(ms));
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // The RVR register is 24 bits wide, as SysTick is based on a 24 bit counter
        const MAX_RVR: u32 = 1 << 24;

        // Depending on hclk (core clock), this 32 bit value allows
        // delays between 1 min to 9 min.
        //
        // (((32^2) - 1) / 72) µs ~ 59.6 seconds
        // (((32^2) - 1) / 8) µs  ~ 536.8 seconds
        let mut total_rvr = us.saturating_mul(self.clocks.hclk().0 / 1_000_000);

        // Use the full 32 bit range to allow longer delays
        //
        // Like dividing total_rvr / MAX_RVR
        // and delaying by MAX_RVR * (fraction).
        while total_rvr != 0 {
            let current_rvr = if total_rvr < MAX_RVR {
                total_rvr
            } else {
                MAX_RVR
            };
            total_rvr -= current_rvr;

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32::from(us))
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32::from(us))
    }
}
