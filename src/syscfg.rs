//! # System configuration controller

use core::ops::Deref;

use crate::{pac::SYSCFG, rcc::APB2};

/// Extension trait that constrains the `SYSCFG` peripheral
pub trait SysCfgExt {
    /// Constrains the `SYSCFG` peripheral so it plays nicely with the other abstractions
    fn constrain(self, apb2: &mut APB2) -> SysCfg;
}

impl SysCfgExt for SYSCFG {
    fn constrain(self, apb2: &mut APB2) -> SysCfg {
        apb2.enr().modify(|_, w| w.syscfgen().enabled());

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

impl Deref for SysCfg {
    type Target = SYSCFG;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
