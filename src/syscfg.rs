//! # System configuration controller

use core::fmt;
use core::ops::Deref;

use crate::gpio::{marker, Pin};
use crate::{
    pac::SYSCFG,
    rcc::{Enable, APB2},
};

/// Extension trait that constrains the `SYSCFG` peripheral
pub trait SysCfgExt {
    /// Constrains the `SYSCFG` peripheral so it plays nicely with the other abstractions
    fn constrain(self, apb2: &mut APB2) -> SysCfg;
}

impl SysCfgExt for SYSCFG {
    fn constrain(self, apb2: &mut APB2) -> SysCfg {
        SYSCFG::enable(apb2);

        SysCfg(self)
    }
}

/// Constrained SYSCFG peripheral
///
/// An instance of this struct is acquired by calling the
/// [`constrain`](SysCfgExt::constrain) function on the
/// [`SYSCFG`](crate::pac::SYSCFG) struct.
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let mut rcc = dp.RCC.constrain();
/// let syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
/// ```
pub struct SysCfg(SYSCFG);

#[cfg(feature = "defmt")]
impl defmt::Format for SysCfg {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "SysCfg(SYSCFG)");
    }
}

impl fmt::Debug for SysCfg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SysCfg").finish()
    }
}

impl Deref for SysCfg {
    type Target = SYSCFG;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SysCfg {
    /// Make corresponding EXTI (external interrupt) line sensitive to the selected pin.
    ///
    /// # Note
    ///
    /// Only **one** Pin index of all banks can be activated
    /// for interrupts at the same time.
    ///
    /// This means, that only on of `PA1`, `PB1`, `PC1`, ... can be activated.
    ///
    /// For example, if first [`crate::gpio::gpioa::PA1`] and than [`crate::gpio::gpiob::PB1`]
    /// would be configured, the former configuration would be overwritten.
    ///
    /// But configuring `PA1` and and `PB2` works!
    #[doc(alias = "enable_interrupt")]
    pub fn select_exti_interrupt_source<Gpio, Index, Mode>(&mut self, pin: &Pin<Gpio, Index, Mode>)
    where
        Gpio: marker::Gpio,
        Index: marker::Index,
    {
        const BITWIDTH: u8 = 4;
        let index = pin.index.index() % 4;
        let extigpionr = pin.gpio.port_index() as u32;
        match pin.index.index() {
            // SAFETY: These are all unguarded writes directly to the register,
            // without leveraging the safety of stm32f3 generated values.
            0..=3 => unsafe { crate::modify_at!(self.exticr1, BITWIDTH, index, extigpionr) },
            4..=7 => unsafe { crate::modify_at!(self.exticr2, BITWIDTH, index, extigpionr) },
            8..=11 => unsafe { crate::modify_at!(self.exticr3, BITWIDTH, index, extigpionr) },
            12..=15 => unsafe { crate::modify_at!(self.exticr4, BITWIDTH, index, extigpionr) },
            _ => crate::unreachable!(),
        };
    }
}
