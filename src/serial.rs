//! Serial

use crate::{
    gpio::{gpioa, gpiob, gpioc, AF7},
    hal::{blocking, serial},
    pac::{USART1, USART2, USART3},
    rcc::{Clocks, APB1, APB2},
    time::Bps,
};
use core::{convert::Infallible, marker::PhantomData, ptr};
use nb;

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::gpio::gpiod;
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::gpio::gpioe;

#[cfg(feature = "stm32f303")]
mod dma_imports {
    pub use crate::dma;
    pub use as_slice::{AsMutSlice, AsSlice};
    pub use core::ops::{Deref, DerefMut};
    pub use stable_deref_trait::StableDeref;
}

#[cfg(feature = "stm32f303")]
use dma_imports::*;

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
}

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// TX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait TxPin<USART> {}

/// RX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait RxPin<USART> {}

unsafe impl TxPin<USART1> for gpioa::PA9<AF7> {}
unsafe impl TxPin<USART1> for gpiob::PB6<AF7> {}
unsafe impl TxPin<USART1> for gpioc::PC4<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl TxPin<USART1> for gpioe::PE0<AF7> {}

unsafe impl RxPin<USART1> for gpioa::PA10<AF7> {}
unsafe impl RxPin<USART1> for gpiob::PB7<AF7> {}
unsafe impl RxPin<USART1> for gpioc::PC5<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl RxPin<USART1> for gpioe::PE1<AF7> {}

unsafe impl TxPin<USART2> for gpioa::PA2<AF7> {}
// unsafe impl TxPin<USART2> for gpioa::PA14<AF7> {}
// unsafe impl TxPin<USART2> for gpiob::PB3<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl TxPin<USART2> for gpiod::PD5<AF7> {}

unsafe impl RxPin<USART2> for gpioa::PA3<AF7> {}
// unsafe impl RxPin<USART2> for gpioa::PA15<AF7> {}
// unsafe impl RxPin<USART2> for gpiob::PB4<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl RxPin<USART2> for gpiod::PD6<AF7> {}

unsafe impl TxPin<USART3> for gpiob::PB10<AF7> {}
unsafe impl TxPin<USART3> for gpioc::PC10<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl TxPin<USART3> for gpiod::PD8<AF7> {}

#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl RxPin<USART3> for gpiob::PB11<AF7> {}
unsafe impl RxPin<USART3> for gpioc::PC11<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl RxPin<USART3> for gpiod::PD9<AF7> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl RxPin<USART3> for gpioe::PE15<AF7> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $USARTX:ident: ($usartX:ident, $APB:ident, $usartXen:ident, $usartXrst:ident, $pclkX:ident),
    )+) => {
        $(
            impl<TX, RX> Serial<$USARTX, (TX, RX)> {
                /// Configures a USART peripheral to provide serial communication
                pub fn $usartX(
                    usart: $USARTX,
                    pins: (TX, RX),
                    baud_rate: Bps,
                    clocks: Clocks,
                    apb: &mut $APB,
                ) -> Self
                where
                    TX: TxPin<$USARTX>,
                    RX: RxPin<$USARTX>,
                {
                    // enable or reset $USARTX
                    apb.enr().modify(|_, w| w.$usartXen().set_bit());
                    apb.rstr().modify(|_, w| w.$usartXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$usartXrst().clear_bit());

                    let brr = clocks.$pclkX().0 / baud_rate.0;
                    assert!(brr >= 16, "impossible baud rate");
                    // NOTE(write): uses all bits of this register.
                    usart.brr.write(|w| unsafe { w.bits(brr) });

                    usart.cr3.modify(|_, w| {
                        w.dmar().enabled();  // enable DMA for reception
                        w.dmat().enabled()   // enable DMA for transmission
                    });

                    usart.cr1.modify(|_, w| {
                        w.ue().enabled();  // enable USART
                        w.re().enabled();  // enable receiver
                        w.te().enabled()   // enable transmitter
                    });

                    Serial { usart, pins }
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                        },
                    }
                }

                /// Starts listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                        },
                    }
                }

                /// Splits the `Serial` abstraction into a transmitter and a receiver half
                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }

                /// Releases the USART peripheral and associated pins
                pub fn free(self) -> ($USARTX, (TX, RX)) {
                    (self.usart, self.pins)
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    // NOTE(unsafe, write) write accessor for atomic writes with no side effects
                    let icr = unsafe { &(*$USARTX::ptr()).icr };

                    Err(if isr.pe().bit_is_set() {
                        icr.write(|w| w.pecf().clear());
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        icr.write(|w| w.fecf().clear());
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        icr.write(|w| w.ncf().clear());
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        icr.write(|w| w.orecf().clear());
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe {
                            ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl serial::Write<u8> for Tx<$USARTX> {
                // NOTE(Infallible) See section "29.7 USART interrupts"; the only possible errors during
                // transmission are: clear to send (which is disabled in this case) errors and
                // framing errors (which only occur in SmartCard mode); neither of these apply to
                // our hardware configuration
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Infallible> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl blocking::serial::write::Default<u8> for Tx<$USARTX> {}

            #[cfg(feature = "stm32f303")]
            impl Rx<$USARTX> {
                /// Fill the buffer with received data using DMA.
                pub fn read_exact<B, C>(
                    self,
                    mut buffer: B,
                    mut channel: C
                ) -> dma::Transfer<B, C, Self>
                where
                    Self: dma::Target<C>,
                    B: DerefMut + StableDeref + 'static,
                    B::Target: AsMutSlice<Element = u8>,
                    C: dma::Channel,
                {
                    // NOTE(unsafe) taking the address of a register
                    let pa = unsafe { &(*$USARTX::ptr()).rdr } as *const _ as u32;
                    channel.set_peripheral_address(pa, dma::Increment::Disable);

                    let slice = buffer.as_mut_slice();
                    let (ma, len) = (slice.as_mut_ptr() as u32, slice.len());
                    channel.set_memory_address(ma, dma::Increment::Enable);
                    channel.set_transfer_length(len);
                    channel.set_word_size(dma::WordSize::Bits8);

                    channel.set_direction(dma::Direction::FromPeripheral);

                    unsafe { dma::Transfer::start(buffer, channel, self) }
                }
            }

            #[cfg(feature = "stm32f303")]
            impl Tx<$USARTX> {
                /// Transmit all data in the buffer using DMA.
                pub fn write_all<B, C>(
                    self,
                    buffer: B,
                    mut channel: C
                ) -> dma::Transfer<B, C, Self>
                where
                    Self: dma::Target<C>,
                    B: Deref + StableDeref + 'static,
                    B::Target: AsSlice<Element = u8>,
                    C: dma::Channel,
                {
                    // NOTE(unsafe) taking the address of a register
                    let pa = unsafe { &(*$USARTX::ptr()).tdr } as *const _ as u32;
                    channel.set_peripheral_address(pa, dma::Increment::Disable);

                    let slice = buffer.as_slice();
                    let (ma, len) = (slice.as_ptr() as u32, slice.len());
                    channel.set_memory_address(ma, dma::Increment::Enable);
                    channel.set_transfer_length(len);
                    channel.set_word_size(dma::WordSize::Bits8);

                    channel.set_direction(dma::Direction::FromMemory);

                    unsafe { dma::Transfer::start(buffer, channel, self) }
                }
            }
        )+
    }
}

#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f318",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
hal! {
    USART1: (usart1, APB2, usart1en, usart1rst, pclk2),
    USART2: (usart2, APB1, usart2en, usart2rst, pclk1),
    USART3: (usart3, APB1, usart3en, usart3rst, pclk1),
}

#[cfg(any(feature = "stm32f302", feature = "stm32f334"))]
hal! {
    USART1: (usart1, APB2, usart1en, usart1rst, pclk2),
    USART2: (usart2, APB1, usart2en, usart2rst, pclk1),
}
