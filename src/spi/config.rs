//! Types for configuring a spi interface.

use crate::time::rate::{self, Extensions};
use core::fmt;

use crate::hal::spi::{self, Mode};
use crate::time::rate::Generic;

/// Configuration struct for [`Spi`](super::Spi) providing all
/// communication-related / parameters.
///
/// The default configruation can be obtain by:
///
/// ```
/// # use stm32f3xx_hal::spi::config::Config;
/// let config = Config::default();
/// ````
///
/// [`Spi`](super::Spi) defaults to [`spi::MODE_0`] and a frequency of 1 MHz.
///
/// ```
/// # use stm32f3xx_hal::spi::config::Config;
/// let config = Config::default();
/// ```
///
/// Create a configuration by using `default` in combination with the
/// builder methods. The following snippet shows creating a configuration
/// for 19,200 Baud, 8N1 by deriving it from the default value:
/// ```
/// # use stm32f3xx_hal::serial::config::*;
/// # use stm32f3xx_hal::time::rate::{Baud, Extensions};
/// let config = Config::default().baudrate(19_200.Bd());
///
/// assert!(config.baudrate == 19_200.Bd());
/// assert!(config.parity == Parity::None);
/// assert!(config.stopbits == StopBits::STOP1);
/// ```
#[derive(Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct Config {
    /// Operating frequency of the SPI peripheral.
    pub frequency: rate::Generic<u32>,
    /// Operation Mode as defined by the [`embedded-hal`]
    pub mode: Mode,
}

impl Config {
    /// Set the operating frequency of the SPI
    pub fn frequency(mut self, frequency: impl Into<Generic<u32>>) -> Self {
        self.frequency = frequency.into();
        self
    }
    /// Set the Operation Mode
    pub fn mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }
}

impl fmt::Debug for Config {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mode = if self.mode == spi::MODE_0 {
            0
        } else if self.mode == spi::MODE_1 {
            1
        } else if self.mode == spi::MODE_2 {
            2
        } else {
            3
        };

        f.debug_struct("Config")
            .field("frequency", &format_args!("{:?}", self.frequency))
            .field("mode", &format_args!("MODE_{}", mode))
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Config {
    fn format(&self, f: defmt::Formatter) {
        let mode = if self.mode == spi::MODE_0 {
            0
        } else if self.mode == spi::MODE_1 {
            1
        } else if self.mode == spi::MODE_2 {
            2
        } else {
            3
        };

        defmt::write!(
            f,
            "Config {{ frequency: {} Hz, mode: MODE_{}}}",
            self.frequency.integer() * *self.frequency.scaling_factor(),
            mode,
        );
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 1.MHz().into(),
            mode: spi::MODE_0,
        }
    }
}

impl<T: Into<rate::Generic<u32>>> From<T> for Config {
    fn from(f: T) -> Config {
        Config {
            frequency: f.into(),
            ..Default::default()
        }
    }
}
