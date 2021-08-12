//! Types for configuring a serial interface.

use crate::pac::usart1::cr2::STOP_A;
use crate::time::rate::{Baud, Extensions};

/// Stop Bit configuration parameter for serial.
///
/// Wrapper around [`STOP_A`]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    /// 0.5 stop bit
    Stop0P5,
    /// 1 stop bit
    Stop1,
    /// 1.5 stop bit
    Stop1P5,
    /// 2 stop bit
    Stop2,
}

impl From<StopBits> for STOP_A {
    fn from(stopbit: StopBits) -> Self {
        match stopbit {
            StopBits::Stop0P5 => STOP_A::STOP0P5,
            StopBits::Stop1 => STOP_A::STOP1,
            StopBits::Stop1P5 => STOP_A::STOP1P5,
            StopBits::Stop2 => STOP_A::STOP2,
        }
    }
}

impl From<STOP_A> for StopBits {
    fn from(stopbit: STOP_A) -> Self {
        match stopbit {
            STOP_A::STOP0P5 => StopBits::Stop0P5,
            STOP_A::STOP1 => StopBits::Stop1,
            STOP_A::STOP1P5 => StopBits::Stop1P5,
            STOP_A::STOP2 => StopBits::Stop2,
        }
    }
}

/// Parity generation and checking. If odd or even parity is selected, the
/// underlying USART will be configured to send/receive the parity bit in
/// addtion to the data bits.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Parity {
    /// No parity bit will be added/checked.
    None,
    /// The MSB transmitted/received will be generated/checked to have a
    /// even number of bits set.
    Even,
    /// The MSB transmitted/received will be generated/checked to have a
    /// odd number of bits set.
    Odd,
}

/// Configuration struct for [`Serial`](super::Serial) providing all
/// communication-related / parameters. `Serial` always uses eight data
/// bits plus the parity bit - if selected.
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
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct Config {
    /// Serial interface baud rate
    pub baudrate: Baud,
    /// Whether and how to generate/check a parity bit
    pub parity: Parity,
    /// The number of stop bits to follow the last data bit or the parity
    /// bit
    pub stopbits: StopBits,
}

impl Config {
    /// Sets the given baudrate.
    pub fn baudrate(mut self, baudrate: impl Into<Baud>) -> Self {
        self.baudrate = baudrate.into();
        self
    }

    /// Sets the given parity.
    pub fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    /// Sets the stop bits to `stopbits`.
    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl Default for Config {
    /// Creates a new configuration with typically used parameters: 115,200
    /// Baud 8N1.
    fn default() -> Config {
        Config {
            baudrate: 115_200.Bd(),
            parity: Parity::None,
            stopbits: StopBits::Stop1,
        }
    }
}

impl<T: Into<Baud>> From<T> for Config {
    fn from(b: T) -> Config {
        Config {
            baudrate: b.into(),
            ..Default::default()
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Config {
    fn format(&self, f: defmt::Formatter) {
        // Omitting pins makes it:
        // 1. Easier.
        // 2. Not to specialized to use it ergonimically for users
        //    even in a generic context.
        // 3. Not require specialization.
        defmt::write!(
            f,
            "Serial {{ baudrate: {} Bd , parity: {} , stopbits: {} }}",
            self.baudrate.0,
            self.parity,
            self.stopbits,
        );
    }
}
