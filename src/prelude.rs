//! Prelude

pub use crate::flash::FlashExt as _stm32f3xx_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32f3xx_hal_gpio_GpioExt;
#[cfg(feature = "unproven")]
pub use crate::hal::digital::v2::InputPin as _embedded_hal_digital_InputPin;
#[cfg(feature = "unproven")]
pub use crate::hal::digital::v2::OutputPin as _embedded_hal_digital_OutputPin;
#[cfg(feature = "unproven")]
pub use crate::hal::digital::v2::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
#[cfg(feature = "unproven")]
pub use crate::hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
pub use crate::hal::prelude::*;
pub use crate::rcc::RccExt as _stm32f3xx_hal_rcc_RccExt;
pub use crate::time::U32Ext as _stm32f3xx_hal_time_U32Ext;
