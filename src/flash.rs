//! # Flash memory
//!
//! Abstractions of the internal flash module.

use crate::pac::{flash, FLASH};

impl crate::private::Sealed for FLASH {}

/// Extension trait to constrain the [`FLASH`] peripheral
#[allow(clippy::module_name_repetitions)]
pub trait FlashExt: crate::private::Sealed {
    /// Constrains the [`FLASH`] peripheral.
    ///
    /// Consumes the [`pac::FLASH`] peripheral and converts it to a [`HAL`] internal type
    /// constraining it's public access surface to fit the design of the [`HAL`].
    ///
    /// [`pac::FLASH`]: `crate::pac::FLASH`
    /// [`HAL`]: `crate`
    fn constrain(self) -> Parts;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Parts {
        Parts {
            acr: ACR { _0: () },
        }
    }
}

/// Constrained FLASH peripheral
pub struct Parts {
    /// Opaque ACR register
    pub acr: ACR,
}

/// Opaque ACR register
pub struct ACR {
    _0: (),
}

impl ACR {
    #[allow(clippy::unused_self)]
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // SAFETY: This proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }
}
