//! Prelude

#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
pub use crate::dma::DmaExt as _stm32f3xx_hal_dma_DmaExt;
pub use crate::flash::FlashExt as _stm32f3xx_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32f3xx_hal_gpio_GpioExt;
pub use crate::hal::prelude::*;
pub use crate::rcc::RccExt as _stm32f3xx_hal_rcc_RccExt;
pub use crate::syscfg::SysCfgExt as _stm32f3xx_hal_syscfg_SysCfgExt;
pub use crate::time::duration::Extensions as _stm32f3xx_hal_time_time_Extensions;
pub use crate::time::rate::Extensions as _stm32f3xx_hal_time_rate_Extensions;
#[cfg(feature = "unproven")]
pub use crate::{
    hal::digital::v2::InputPin as _embedded_hal_digital_InputPin,
    hal::digital::v2::OutputPin as _embedded_hal_digital_OutputPin,
    hal::digital::v2::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin,
    hal::digital::v2::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin,
};
