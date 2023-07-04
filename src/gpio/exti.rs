use super::{marker, Edge, Pin};
use crate::pac::{Interrupt, EXTI};
use crate::Switch;

/// Return an EXTI register for the current CPU
#[cfg(feature = "svd-f373")]
macro_rules! reg_for_cpu {
    ($exti:expr, $xr:ident) => {
        $exti.$xr
    };
}

/// Return an EXTI register for the current CPU
#[cfg(not(feature = "svd-f373"))]
macro_rules! reg_for_cpu {
    ($exti:expr, $xr:ident) => {
        paste::paste! {
            $exti.[<$xr 1>]
        }
    };
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// NVIC interrupt number of interrupt from this pin
    ///
    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
    /// This is also useful for all other [`cortex_m::peripheral::NVIC`] functions.
    pub const fn interrupt(&self) -> Interrupt {
        match N {
            0 => Interrupt::EXTI0,
            1 => Interrupt::EXTI1,
            #[cfg(feature = "svd-f373")]
            2 => Interrupt::EXTI2_TS,
            #[cfg(not(feature = "svd-f373"))]
            2 => Interrupt::EXTI2_TSC,
            3 => Interrupt::EXTI3,
            4 => Interrupt::EXTI4,
            #[cfg(feature = "svd-f373")]
            5..=9 => Interrupt::EXTI5_9,
            #[cfg(not(feature = "svd-f373"))]
            5..=9 => Interrupt::EXTI9_5,
            10..=15 => Interrupt::EXTI15_10,
            _ => panic!("Unsupported pin number"),
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::Interruptable,
{
    /// Generate interrupt on rising edge, falling edge, or both
    pub fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        const BITWIDTH: u8 = 1;
        let (rise, fall) = match edge {
            Edge::Rising => (true as u32, false as u32),
            Edge::Falling => (false as u32, true as u32),
            Edge::RisingFalling => (true as u32, true as u32),
        };
        // SAFETY: Unguarded write to the register, but behind a &mut
        unsafe {
            crate::modify_at!(reg_for_cpu!(exti, rtsr), BITWIDTH, N, rise);
            crate::modify_at!(reg_for_cpu!(exti, ftsr), BITWIDTH, N, fall);
        }
    }

    /// Configure external interrupts from this pin
    ///
    /// # Note
    ///
    /// Remeber to also configure the interrupt pin on
    /// the SysCfg site, with [`crate::syscfg::SysCfg::select_exti_interrupt_source()`]
    pub fn configure_interrupt(&mut self, exti: &mut EXTI, enable: impl Into<Switch>) {
        const BITWIDTH: u8 = 1;

        let enable: Switch = enable.into();
        let enable: bool = enable.into();

        let value = u32::from(enable);
        // SAFETY: Unguarded write to the register, but behind a &mut
        unsafe { crate::modify_at!(reg_for_cpu!(exti, imr), BITWIDTH, N, value) };
    }

    /// Enable external interrupts from this pin
    ///
    /// # Note
    ///
    /// Remeber to also configure the interrupt pin on
    /// the SysCfg site, with [`crate::syscfg::SysCfg::select_exti_interrupt_source()`]
    pub fn enable_interrupt(&mut self, exti: &mut EXTI) {
        self.configure_interrupt(exti, Switch::On)
    }

    /// Disable external interrupts from this pin
    pub fn disable_interrupt(&mut self, exti: &mut EXTI) {
        self.configure_interrupt(exti, Switch::Off)
    }

    /// Clear the interrupt pending bit for this pin
    pub fn clear_interrupt(&mut self) {
        // SAFETY: Atomic write to register without side-effects.
        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).write(|w| w.bits(1 << N)) };
    }

    /// Reads the interrupt pending bit for this pin
    pub fn is_interrupt_pending(&self) -> bool {
        // SAFETY: Atomic write to register without side-effects.
        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).read().bits() & (1 << N) != 0 }
    }
}
