//! # Watchdog
//!
//! ## Examples
//!
//! A usage example of the watchdog can be found at [examples/can.rs]
//!
//! [examples/can.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.8.2/examples/can.rs

use core::fmt;
use embedded_time::fixed_point::FixedPoint;

use crate::hal::watchdog::{Watchdog, WatchdogEnable};

use crate::pac::{iwdg::pr::PR_A, DBGMCU, IWDG};
use crate::time::duration::Milliseconds;
use crate::time::rate::Kilohertz;

/// Frequency of the watchdog peripheral clock
const LSI: Kilohertz = Kilohertz(40);
// const MAX_PRESCALER: u8 = 0b0111;
const MAX_PRESCALER: PR_A = PR_A::DIVIDEBY256;
const MAX_RELOAD: u32 = 0x0FFF;

/// Independent Watchdog Peripheral
pub struct IndependentWatchDog {
    iwdg: IWDG,
}

#[cfg(feature = "defmt")]
impl defmt::Format for IndependentWatchDog {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "IndependentWatchDog {{ iwdg: IWDG }}");
    }
}

impl fmt::Debug for IndependentWatchDog {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("IndependentWatchDog")
            .field("iwdg", &"IWDG")
            .finish()
    }
}

fn into_division_value(psc: PR_A) -> u32 {
    match psc {
        PR_A::DIVIDEBY4 => 4,
        PR_A::DIVIDEBY8 => 8,
        PR_A::DIVIDEBY16 => 16,
        PR_A::DIVIDEBY32 => 32,
        PR_A::DIVIDEBY64 => 64,
        PR_A::DIVIDEBY128 => 128,
        PR_A::DIVIDEBY256 | PR_A::DIVIDEBY256BIS => 256,
    }
}

impl IndependentWatchDog {
    /// Creates a new [`IndependentWatchDog`] without starting it.
    ///
    /// Call [`start`](WatchdogEnable::start) to start the watchdog.
    ///
    /// See [`WatchdogEnable`] and [`Watchdog`] for more info.
    pub fn new(iwdg: IWDG) -> Self {
        IndependentWatchDog { iwdg }
    }

    /// Set the watchdog to stop when a breakpoint is hit while debugging
    pub fn stop_on_debug(&self, dbg: &DBGMCU, stop: bool) {
        dbg.apb1_fz.modify(|_, w| w.dbg_iwdg_stop().bit(stop));
    }

    /// Find and setup the next best prescaler and reload value for the selected timeout
    fn setup(&self, timeout: Milliseconds) {
        let mut reload: u32 =
            timeout.integer() * LSI.integer() / into_division_value(PR_A::DIVIDEBY4);

        // Reload is potentially to high to be stored in the register.
        // The goal of this loop is to find the maximum possible reload value,
        // which can be stored in the register, while still guaranteeing the wanted timeout.
        //
        // This is achived by increasing the prescaler value.
        let mut psc = 0;
        loop {
            if psc >= MAX_PRESCALER as u8 {
                // ceil the value to the maximum allow reload value
                if reload > MAX_RELOAD {
                    reload = MAX_RELOAD;
                }
                break;
            }
            if reload <= MAX_RELOAD {
                break;
            }
            psc += 1;
            // When the precaler value incresed, the reload value has to be halfed
            // so that the timeout stays the same.
            reload /= 2;
        }

        self.access_registers(|iwdg| {
            iwdg.pr.modify(|_, w| w.pr().bits(psc));
            iwdg.rlr.modify(|_, w| w.rl().bits(reload as u16));
        });

        // NOTE: As the watchdog can not be stopped once started,
        // a free method is not provided.
        // pub fn free(self) -> IWDG {}
    }

    /// Get access to the underlying register block.
    ///
    /// # Safety
    ///
    /// This function is not _memory_ unsafe per se, but does not guarantee
    /// anything about assumptions of invariants made in this implementation.
    ///
    /// Changing specific options can lead to un-expected behavior and nothing
    /// is guaranteed.
    pub unsafe fn peripheral(&mut self) -> &mut IWDG {
        &mut self.iwdg
    }

    /// Returns the currently set interval
    pub fn interval(&self) -> Milliseconds {
        // If the prescaler was changed wait until the change procedure is finished.
        while self.iwdg.sr.read().pvu().bit() {}

        let psc = self.iwdg.pr.read().pr().variant();
        let reload = self.iwdg.rlr.read().rl().bits();

        Milliseconds((into_division_value(psc) * u32::from(reload)) / LSI.integer())
    }

    fn access_registers<A, F: FnMut(&IWDG) -> A>(&self, mut f: F) -> A {
        // Unprotect write access to registers
        self.iwdg.kr.write(|w| w.key().enable());
        let a = f(&self.iwdg);

        // Protect again
        self.iwdg.kr.write(|w| w.key().reset());
        a
    }
}

impl WatchdogEnable for IndependentWatchDog {
    type Time = Milliseconds;

    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        self.setup(period.into());

        self.iwdg.kr.write(|w| w.key().start());
    }
}

impl Watchdog for IndependentWatchDog {
    fn feed(&mut self) {
        self.iwdg.kr.write(|w| w.key().reset());
    }
}
