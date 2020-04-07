//! Direct memory access (DMA) controller

// Currently DMA is only supported for STM32F303 MCUs.
// Remove these `allow`s once all models have support.
#![cfg_attr(not(feature = "stm32f303"), allow(unused_imports, unused_macros))]

use crate::{
    rcc::AHB,
    serial,
    stm32::{self, dma1::ch::cr},
};
use cast::u16;
use core::sync::atomic::{self, Ordering};
use stable_deref_trait::StableDeref;

/// Extension trait to split a DMA peripheral into independent channels
pub trait DmaExt {
    /// The type to split the DMA into
    type Channels;

    /// Split the DMA into independent channels
    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

/// An in-progress one-shot DMA transfer
pub struct Transfer<B, C: Channel, T> {
    // This is always a `Some` outside of `drop`.
    inner: Option<TransferInner<B, C, T>>,
}

impl<B, C: Channel, T> Transfer<B, C, T> {
    /// Start a DMA transfer.
    ///
    /// # Safety
    ///
    /// Callers must ensure that the DMA channel is configured correctly for
    /// the given target and buffer.
    pub unsafe fn start(buffer: B, mut channel: C, target: T) -> Self
    where
        B: StableDeref + 'static,
        T: Target<C>,
    {
        assert!(!channel.is_enabled());

        atomic::compiler_fence(Ordering::Release);

        channel.enable();

        Self {
            inner: Some(TransferInner {
                buffer,
                channel,
                target,
            }),
        }
    }

    /// Is this transfer complete?
    pub fn is_complete(&self) -> bool {
        let inner = self.inner.as_ref().unwrap();
        inner.channel.event_occured(Event::TransferComplete)
    }

    /// Stop this transfer and return ownership over its parts
    pub fn stop(mut self) -> (B, C, T) {
        let mut inner = self.inner.take().unwrap();
        inner.stop();

        (inner.buffer, inner.channel, inner.target)
    }

    /// Block until this transfer is done and return ownership over its parts
    pub fn wait(self) -> (B, C, T) {
        while !self.is_complete() {}

        self.stop()
    }
}

impl<B, C: Channel, T> Drop for Transfer<B, C, T> {
    fn drop(&mut self) {
        if let Some(inner) = self.inner.as_mut() {
            inner.stop();
        }
    }
}

/// This only exists so we can implement `Drop` for `Transfer`.
struct TransferInner<B, C, T> {
    buffer: B,
    channel: C,
    target: T,
}

impl<B, C: Channel, T> TransferInner<B, C, T> {
    /// Stop this transfer
    fn stop(&mut self) {
        self.channel.disable();
        atomic::compiler_fence(Ordering::Acquire);
    }
}

/// DMA address increment mode
pub enum Increment {
    /// Enable increment
    Enable,
    /// Disable increment
    Disable,
}

impl From<Increment> for cr::PINC_A {
    fn from(inc: Increment) -> Self {
        match inc {
            Increment::Enable => cr::PINC_A::ENABLED,
            Increment::Disable => cr::PINC_A::DISABLED,
        }
    }
}

/// DMA word size
pub enum WordSize {
    /// 8 bits
    Bits8,
    /// 16 bits
    Bits16,
    /// 32 bits
    Bits32,
}

impl From<WordSize> for cr::PSIZE_A {
    fn from(size: WordSize) -> Self {
        match size {
            WordSize::Bits8 => cr::PSIZE_A::BITS8,
            WordSize::Bits16 => cr::PSIZE_A::BITS16,
            WordSize::Bits32 => cr::PSIZE_A::BITS32,
        }
    }
}

/// Channel priority level
pub enum Priority {
    /// Low
    Low,
    /// Medium
    Medium,
    /// High
    High,
    /// Very high
    VeryHigh,
}

impl From<Priority> for cr::PL_A {
    fn from(prio: Priority) -> Self {
        match prio {
            Priority::Low => cr::PL_A::LOW,
            Priority::Medium => cr::PL_A::MEDIUM,
            Priority::High => cr::PL_A::HIGH,
            Priority::VeryHigh => cr::PL_A::VERYHIGH,
        }
    }
}

/// DMA transfer direction
pub enum Direction {
    /// From memory to peripheral
    FromMemory,
    /// From peripheral to memory
    FromPeripheral,
}

impl From<Direction> for cr::DIR_A {
    fn from(dir: Direction) -> Self {
        match dir {
            Direction::FromMemory => cr::DIR_A::FROMMEMORY,
            Direction::FromPeripheral => cr::DIR_A::FROMPERIPHERAL,
        }
    }
}

/// DMA events
pub enum Event {
    /// First half of a transfer is done
    HalfTransfer,
    /// Transfer is complete
    TransferComplete,
    /// A transfer error occurred
    TransferError,
    /// Any of the above events occurred
    Any,
}

/// Trait implemented by all DMA channels
pub trait Channel: private::Channel {
    /// Is the interrupt flag for the given event set?
    fn event_occured(&self, event: Event) -> bool;
    /// Clear the interrupt flag for the given event
    fn clear_event(&mut self, event: Event);

    /// Reset the control registers of this channel.
    /// This stops any ongoing transfers.
    fn reset(&mut self) {
        self.ch().cr.reset();
        self.ch().ndtr.reset();
        self.ch().par.reset();
        self.ch().mar.reset();
        self.clear_event(Event::Any);
    }

    /// Set the base address of the peripheral data register from/to which the
    /// data will be read/written.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    fn set_peripheral_address(&mut self, address: u32, inc: Increment) {
        assert!(!self.is_enabled());

        self.ch().par.write(|w| w.pa().bits(address));
        self.ch().cr.modify(|_, w| w.pinc().variant(inc.into()));
    }

    /// Set the base address of the memory area from/to which
    /// the data will be read/written.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    fn set_memory_address(&mut self, address: u32, inc: Increment) {
        assert!(!self.is_enabled());

        self.ch().mar.write(|w| w.ma().bits(address));
        self.ch().cr.modify(|_, w| w.minc().variant(inc.into()));
    }

    /// Set the number of words to transfer.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    fn set_transfer_length(&mut self, len: usize) {
        assert!(!self.is_enabled());

        let len = u16(len).expect("DMA transfer length too large");
        self.ch().ndtr.write(|w| w.ndt().bits(len));
    }

    /// Set the word size
    fn set_word_size(&mut self, size: WordSize) {
        let psize = size.into();
        self.ch().cr.modify(|_, w| {
            w.psize().variant(psize);
            w.msize().variant(psize)
        });
    }

    /// Set the priority level of this channel
    fn set_priority_level(&mut self, priority: Priority) {
        let pl = priority.into();
        self.ch().cr.modify(|_, w| w.pl().variant(pl));
    }

    /// Set the transfer direction
    fn set_direction(&mut self, direction: Direction) {
        let dir = direction.into();
        self.ch().cr.modify(|_, w| w.dir().variant(dir));
    }

    /// Enable the interrupt for the given event
    fn listen(&mut self, event: Event) {
        use Event::*;
        match event {
            HalfTransfer => self.ch().cr.modify(|_, w| w.htie().enabled()),
            TransferComplete => self.ch().cr.modify(|_, w| w.tcie().enabled()),
            TransferError => self.ch().cr.modify(|_, w| w.teie().enabled()),
            Any => self.ch().cr.modify(|_, w| {
                w.htie().enabled();
                w.tcie().enabled();
                w.teie().enabled()
            }),
        }
    }

    /// Disable the interrupt for the given event
    fn unlisten(&mut self, event: Event) {
        use Event::*;
        match event {
            HalfTransfer => self.ch().cr.modify(|_, w| w.htie().disabled()),
            TransferComplete => self.ch().cr.modify(|_, w| w.tcie().disabled()),
            TransferError => self.ch().cr.modify(|_, w| w.teie().disabled()),
            Any => self.ch().cr.modify(|_, w| {
                w.htie().disabled();
                w.tcie().disabled();
                w.teie().disabled()
            }),
        }
    }

    /// Start a transfer
    fn enable(&mut self) {
        self.clear_event(Event::Any);
        self.ch().cr.modify(|_, w| w.en().enabled());
    }

    /// Stop the current transfer
    fn disable(&mut self) {
        self.ch().cr.modify(|_, w| w.en().disabled());
    }

    /// Is there a transfer in progress on this channel?
    fn is_enabled(&self) -> bool {
        self.ch().cr.read().en().is_enabled()
    }
}

mod private {
    use crate::stm32;

    /// Channel methods private to this module
    pub trait Channel {
        /// Return the register block for this channel
        fn ch(&self) -> &stm32::dma1::CH;
    }
}

macro_rules! dma {
    (
        $DMAx:ident, $dmax:ident, $dmaxen:ident,
        channels: {
            $( $Ci:ident: (
                $chi:ident,
                $htifi:ident, $tcifi:ident, $teifi:ident, $gifi:ident,
                $chtifi:ident, $ctcifi:ident, $cteifi:ident, $cgifi:ident
            ), )+
        },
    ) => {
        pub mod $dmax {
            use super::*;
            use crate::stm32::$DMAx;

            impl DmaExt for $DMAx {
                type Channels = Channels;

                fn split(self, ahb: &mut AHB) -> Channels {
                    ahb.enr().modify(|_, w| w.$dmaxen().set_bit());

                    let mut channels = Channels {
                        $( $chi: $Ci { _0: () }, )+
                    };

                    channels.reset();
                    channels
                }
            }

            /// DMA channels
            pub struct Channels {
                $( pub $chi: $Ci, )+
            }

            impl Channels {
                /// Reset the control registers of all channels.
                /// This stops any ongoing transfers.
                fn reset(&mut self) {
                    $( self.$chi.reset(); )+
                }
            }

            $(
                /// Singleton that represents a DMA channel
                pub struct $Ci {
                    _0: (),
                }

                impl private::Channel for $Ci {
                    fn ch(&self) -> &stm32::dma1::CH {
                        // NOTE(unsafe) $Ci grants exclusive access to this register
                        unsafe { &(*$DMAx::ptr()).$chi }
                    }
                }

                impl Channel for $Ci {
                    fn event_occured(&self, event: Event) -> bool {
                        use Event::*;

                        // NOTE(unsafe) atomic read
                        let flags = unsafe { (*$DMAx::ptr()).isr.read() };
                        match event {
                            HalfTransfer => flags.$htifi().bit_is_set(),
                            TransferComplete => flags.$tcifi().bit_is_set(),
                            TransferError => flags.$teifi().bit_is_set(),
                            Any => flags.$gifi().bit_is_set(),
                        }
                    }

                    fn clear_event(&mut self, event: Event) {
                        use Event::*;

                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe {
                            &(*$DMAx::ptr()).ifcr.write(|w| match event {
                                HalfTransfer => w.$chtifi().set_bit(),
                                TransferComplete => w.$ctcifi().set_bit(),
                                TransferError => w.$cteifi().set_bit(),
                                Any => w.$cgifi().set_bit(),
                            });
                        }
                    }
                }
            )+
        }
    };
}

#[cfg(feature = "stm32f303")]
dma!(
    DMA1, dma1, dma1en,
    channels: {
        C1: (ch1, htif1, tcif1, teif1, gif1, chtif1, ctcif1, cteif1, cgif1),
        C2: (ch2, htif2, tcif2, teif2, gif2, chtif2, ctcif2, cteif2, cgif2),
        C3: (ch3, htif3, tcif3, teif3, gif3, chtif3, ctcif3, cteif3, cgif3),
        C4: (ch4, htif4, tcif4, teif4, gif4, chtif4, ctcif4, cteif4, cgif4),
        C5: (ch5, htif5, tcif5, teif5, gif5, chtif5, ctcif5, cteif5, cgif5),
        C6: (ch6, htif6, tcif6, teif6, gif6, chtif6, ctcif6, cteif6, cgif6),
        C7: (ch7, htif7, tcif7, teif7, gif7, chtif7, ctcif7, cteif7, cgif7),
    },
);

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe"
))]
dma!(
    DMA2, dma2, dma2en,
    channels: {
        C1: (ch1, htif1, tcif1, teif1, gif1, chtif1, ctcif1, cteif1, cgif1),
        C2: (ch2, htif2, tcif2, teif2, gif2, chtif2, ctcif2, cteif2, cgif2),
        C3: (ch3, htif3, tcif3, teif3, gif3, chtif3, ctcif3, cteif3, cgif3),
        C4: (ch4, htif4, tcif4, teif4, gif4, chtif4, ctcif4, cteif4, cgif4),
        C5: (ch5, htif5, tcif5, teif5, gif5, chtif5, ctcif5, cteif5, cgif5),
    },
);

/// Marker trait for DMA targets and their channels
pub unsafe trait Target<C: Channel> {}

macro_rules! targets {
    (
        $dma:ident,
        $( $target:ty => $C:ident, )+
    ) => {
        $( unsafe impl Target<$dma::$C> for $target {} )+
    };
}

#[cfg(feature = "stm32f303")]
targets!(dma1,
    serial::Rx<stm32::USART1> => C5,
    serial::Tx<stm32::USART1> => C4,
    serial::Rx<stm32::USART2> => C6,
    serial::Tx<stm32::USART2> => C7,
    serial::Rx<stm32::USART3> => C3,
    serial::Tx<stm32::USART3> => C2,
);
