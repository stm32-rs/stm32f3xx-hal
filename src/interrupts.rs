//! Common Interrupt interface defintions shared between peipherals.

/// A common interrupt number interface, which returns the associated interrupt
/// of the peripheral.
///
/// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
/// This is useful for all `cortex_m::peripheral::INTERRUPT` functions.
pub trait InterruptNumber {
    /// The type used to represent the Interrupt Number.
    ///
    /// This type of interrupt should be compatible with [`cortex_m::peripheral::NVIC`].
    type Interrupt;

    /// The assocaited constant of the interrupt
    const INTERRUPT: Self::Interrupt;
}
