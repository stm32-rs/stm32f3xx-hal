//! # Direct memory access (DMA) controller.
//!
//! Currently DMA is only supported for STM32F303 or STM32F302 MCUs.
//!
//! ## Examples
//!
//! An example how to use DMA for serial, can be found at [examples/serial_dma.rs]
//!
//! [examples/serial_dma.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.10.0/examples/serial_dma.rs

// To learn about most of the ideas implemented here, check out the DMA section
// of the Embedonomicon: https://docs.rust-embedded.org/embedonomicon/dma.html

pub use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::{
    pac::{self, dma1::ch::cr},
    rcc::AHB,
    serial,
};
use core::{
    convert::TryFrom,
    mem,
    sync::atomic::{self, Ordering},
};

#[cfg(feature = "enumset")]
use enumset::EnumSetType;

/// Extension trait to split a DMA peripheral into independent channels
#[allow(clippy::module_name_repetitions)]
pub trait DmaExt {
    /// The type to split the DMA into
    type Channels;

    /// Split the DMA into independent channels
    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

/// Trait implemented by DMA targets.
pub trait Target {
    /// Enable DMA on the target
    fn enable_dma(&mut self) {}
    /// Disable DMA on the target
    fn disable_dma(&mut self) {}
}

/// An in-progress one-shot DMA transfer
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Transfer<'t, B, C, T>
where
    // C: Channel,
    T: 't + Target,
{
    // This is always a `Some` outside of `drop`.
    inner: Option<TransferInner<'t, B, C, T>>,
}

impl<'t, B, C: Channel, T: Target> Transfer<'t, B, C, T> {
    /// Start a DMA write transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    pub fn start_write(mut buffer: B, mut channel: C, target: &'t mut T) -> Self
    where
        B: WriteBuffer + 'static,
        T: OnChannel<C>,
    {
        // SAFETY: We don't know the concrete type of `buffer` here, all
        // we can use are its `WriteBuffer` methods. Hence the only `&mut self`
        // method we can call is `write_buffer`, which is allowed by
        // `WriteBuffer`'s safety requirements.
        let (ptr, len) = unsafe { buffer.write_buffer() };
        let len = crate::expect!(u16::try_from(len).ok(), "buffer is too large");

        // SAFETY: We are using the address of a 'static WriteBuffer here,
        // which is guaranteed to be safe for DMA.
        unsafe { channel.set_memory_address(ptr as u32, Increment::Enable) };
        channel.set_transfer_length(len);
        channel.set_word_size::<B::Word>();
        channel.set_direction(Direction::FromPeripheral);

        // SAFTEY: we take ownership of the buffer, which is 'static as well, so it lives long
        // enough (at least longer that the DMA transfer itself)
        #[allow(clippy::undocumented_unsafe_blocks)]
        unsafe {
            Self::start(buffer, channel, target)
        }
    }

    /// Start a DMA read transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    pub fn start_read(buffer: B, mut channel: C, target: &'t mut T) -> Self
    where
        B: ReadBuffer + 'static,
        T: OnChannel<C>,
    {
        // SAFETY: We don't know the concrete type of `buffer` here, all
        // we can use are its `ReadBuffer` methods. Hence there are no
        // `&mut self` methods we can call, so we are safe according to
        // `ReadBuffer`'s safety requirements.
        let (ptr, len) = unsafe { buffer.read_buffer() };
        let len = crate::expect!(u16::try_from(len).ok(), "buffer is too large");

        // SAFETY: We are using the address of a 'static ReadBuffer here,
        // which is guaranteed to be safe for DMA.
        unsafe { channel.set_memory_address(ptr as u32, Increment::Enable) };
        channel.set_transfer_length(len);
        channel.set_word_size::<B::Word>();
        channel.set_direction(Direction::FromMemory);

        // SAFTEY: We take ownership of the buffer, which is 'static as well, so it lives long
        // enough (at least longer that the DMA transfer itself)
        #[allow(clippy::undocumented_unsafe_blocks)]
        unsafe {
            Self::start(buffer, channel, target)
        }
    }

    /// # Safety
    ///
    /// Callers must ensure that:
    ///
    /// - the given buffer will be valid for the duration of the transfer
    /// - the DMA channel is configured correctly for the given target and buffer
    unsafe fn start(buffer: B, mut channel: C, target: &'t mut T) -> Self
    where
        T: OnChannel<C>,
    {
        crate::assert!(!channel.is_enabled());

        atomic::compiler_fence(Ordering::Release);

        target.enable_dma();
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
    ///
    /// # Panics
    ///
    /// Panics if no transfer is ongoing.
    pub fn is_complete(&self) -> bool {
        let inner = crate::unwrap!(self.inner.as_ref());
        inner.channel.is_event_triggered(Event::TransferComplete)
    }

    /// Stop this transfer and return ownership over its parts
    ///
    /// # Panics
    ///
    /// Panics no transfer is ongoing.
    pub fn stop(mut self) -> (B, C) {
        let mut inner = crate::unwrap!(self.inner.take());
        inner.stop();

        (inner.buffer, inner.channel)
    }

    /// Block until this transfer is done and return ownership over its parts
    pub fn wait(self) -> (B, C) {
        while !self.is_complete() {}

        self.stop()
    }
}

impl<'t, BR, BW, CR, CW, T> Transfer<'t, (BR, BW), (CR, CW), T>
where
    CR: Channel, /*  + ReceiveChannel */
    CW: Channel, /*  + TransmitChannel */
    T: Target,
{
    /// Start a DMA write transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    // TODO: sort arguments (target at first, write at first (see embedded hal?))
    pub fn start_transfer(
        buffer_read: BR,
        mut buffer_write: BW,
        mut channel_read: CR,
        mut channel_write: CW,
        target: &'t mut T,
    ) -> Self
    where
        BR: ReadBuffer + 'static,
        BW: WriteBuffer + 'static,
        T: OnChannel<CR> + OnChannel<CW>,
    {
        // TODO: ensure same buffer length
        // TODO: ensure same nord type

        // SAFETY: We don't know the concrete type of `buffer` here, all
        // we can use are its `WriteBuffer` methods. Hence the only `&mut self`
        // method we can call is `write_buffer`, which is allowed by
        // `WriteBuffer`'s safety requirements.
        let (ptr, len) = unsafe { buffer_write.write_buffer() };
        let len = crate::expect!(u16::try_from(len).ok(), "buffer is too large");

        // SAFETY: We are using the address of a 'static WriteBuffer here,
        // which is guaranteed to be safe for DMA.
        unsafe { channel_write.set_memory_address(ptr as u32, Increment::Enable) };
        channel_write.set_transfer_length(len);
        channel_write.set_word_size::<BW::Word>();
        channel_write.set_direction(Direction::FromPeripheral);

        // SAFETY: We don't know the concrete type of `buffer` here, all
        // we can use are its `ReadBuffer` methods. Hence there are no
        // `&mut self` methods we can call, so we are safe according to
        // `ReadBuffer`'s safety requirements.
        let (ptr, len) = unsafe { buffer_read.read_buffer() };
        let len = crate::expect!(u16::try_from(len).ok(), "buffer is too large");

        // SAFETY: We are using the address of a 'static ReadBuffer here,
        // which is guaranteed to be safe for DMA.
        unsafe { channel_read.set_memory_address(ptr as u32, Increment::Enable) };
        channel_read.set_transfer_length(len);
        channel_read.set_word_size::<BR::Word>();
        channel_read.set_direction(Direction::FromMemory);

        crate::assert!(!channel_read.is_enabled() && !channel_write.is_enabled());

        atomic::compiler_fence(Ordering::Release);

        target.enable_dma();
        channel_read.enable();
        channel_write.enable();

        Self {
            inner: Some(TransferInner {
                buffer: (buffer_read, buffer_write),
                channel: (channel_read, channel_write),
                target,
            }),
        }
    }

    /// Is this transfer complete?
    ///
    /// # Panics
    ///
    /// Panics if no transfer is ongoing.
    pub fn is_complete(&self) -> bool {
        let inner = crate::unwrap!(self.inner.as_ref());
        defmt::dbg!(inner.channel.0.is_event_triggered(Event::Any));
        defmt::dbg!(inner.channel.1.is_event_triggered(Event::Any));
        inner.channel.0.is_event_triggered(Event::TransferComplete)
            && inner.channel.1.is_event_triggered(Event::TransferComplete)
    }

    /// Stop this transfer and return ownership over its parts
    ///
    /// # Panics
    ///
    /// Panics no transfer is ongoing.
    pub fn stop(mut self) -> ((BR, BW), (CR, CW)) {
        let mut inner = crate::unwrap!(self.inner.take());
        inner.stop();

        (inner.buffer, inner.channel)
    }

    /// Block until this transfer is done and return ownership over its parts
    pub fn wait(self) -> ((BR, BW), (CR, CW)) {
        defmt::dbg!(self.inner.as_ref().unwrap().channel.0.is_enabled());
        defmt::dbg!(self.inner.as_ref().unwrap().channel.1.is_enabled());
        while !self.is_complete() {}

        self.stop()
    }
}

// TODO: Use different drops in case of one or both channels
// impl<'t, B, C, T> Drop for Transfer<'t, B, C, T>
// where
//     C: Channel,
//     T: Target,
// {
//     fn drop(&mut self) {
//         if let Some(inner) = self.inner.as_mut() {
//             inner.stop();
//         }
//     }
// }

/// This only exists so we can implement `Drop` for `Transfer`.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct TransferInner<'t, B, C, T> {
    buffer: B,
    channel: C,
    target: &'t mut T,
}

impl<'t, B, C: Channel, T: Target> TransferInner<'t, B, C, T> {
    /// Stop this transfer
    fn stop(&mut self) {
        self.channel.disable();
        self.target.disable_dma();

        atomic::compiler_fence(Ordering::SeqCst);
    }
}

impl<'t, B, CR, CW, T> TransferInner<'t, B, (CR, CW), T>
where
    CR: Channel,
    CW: Channel,
    T: Target,
{
    /// Stop this transfer
    fn stop(&mut self) {
        self.channel.0.disable();
        self.channel.1.disable();
        self.target.disable_dma();

        atomic::compiler_fence(Ordering::SeqCst);
    }
}

/// DMA address increment mode
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Increment {
    /// Enable increment
    Enable,
    /// Disable increment
    Disable,
}

impl From<Increment> for cr::PINC_A {
    fn from(inc: Increment) -> Self {
        match inc {
            Increment::Enable => cr::PINC_A::Enabled,
            Increment::Disable => cr::PINC_A::Disabled,
        }
    }
}

/// Channel priority level
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
            Priority::Low => cr::PL_A::Low,
            Priority::Medium => cr::PL_A::Medium,
            Priority::High => cr::PL_A::High,
            Priority::VeryHigh => cr::PL_A::VeryHigh,
        }
    }
}

/// DMA transfer direction
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// From memory to peripheral
    FromMemory,
    /// From peripheral to memory
    FromPeripheral,
}

impl From<Direction> for cr::DIR_A {
    fn from(dir: Direction) -> Self {
        match dir {
            Direction::FromMemory => cr::DIR_A::FromMemory,
            Direction::FromPeripheral => cr::DIR_A::FromPeripheral,
        }
    }
}

/// DMA events
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
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
    fn is_event_triggered(&self, event: Event) -> bool;

    /// Clear the interrupt flag for the given event.
    ///
    /// Passing `Event::Any` clears all interrupt flags.
    ///
    /// Note that the the global interrupt flag is not automatically cleared
    /// even when all other flags are cleared. The only way to clear it is to
    /// call this method with `Event::Any`.
    fn clear_event(&mut self, event: Event);

    /// Clear **all** interrupt event flags
    fn clear_events(&mut self) {
        self.clear_event(Event::Any);
    }

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
    ///
    /// # Safety
    ///
    /// Callers must ensure the given address is the address of a peripheral
    /// register that supports DMA.
    unsafe fn set_peripheral_address(&mut self, address: u32, inc: Increment) {
        crate::assert!(!self.is_enabled());

        // SAFETY: If the caller does ensure, that address is valid address, this should be safe
        unsafe {
            self.ch().par.write(|w| w.pa().bits(address));
        }
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
    ///
    /// # Safety
    ///
    /// Callers must ensure the given address is a valid memory address
    /// that will remain valid as long as at is used by DMA.
    unsafe fn set_memory_address(&mut self, address: u32, inc: Increment) {
        crate::assert!(!self.is_enabled());

        // SAFETY: If the caller does ensure, that address is valid address, this should be safe
        unsafe {
            self.ch().mar.write(|w| w.ma().bits(address));
        }
        self.ch().cr.modify(|_, w| w.minc().variant(inc.into()));
    }

    /// Set the number of words to transfer.
    ///
    /// Only call this method on disabled channels.
    ///
    /// # Panics
    ///
    /// Panics if this channel is enabled.
    fn set_transfer_length(&mut self, len: u16) {
        crate::assert!(!self.is_enabled());

        self.ch().ndtr.write(|w| w.ndt().bits(len));
    }

    /// Set the word size.
    ///
    /// # Panics
    ///
    /// Panics if the word size is not one of 8, 16, or 32 bits.
    fn set_word_size<W>(&mut self) {
        use cr::PSIZE_A::{Bits16, Bits32, Bits8};

        let psize = match mem::size_of::<W>() {
            1 => Bits8,
            2 => Bits16,
            4 => Bits32,
            #[cfg(not(feature = "defmt"))]
            s => core::panic!("unsupported word size: {:?}", s),
            #[cfg(feature = "defmt")]
            _ => defmt::panic!("unsupported word size"),
        };

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

    /// Enable or disable the interrupt for the specified [`Event`].
    fn configure_interrupt(&mut self, event: Event, enable: bool) {
        match event {
            Event::HalfTransfer => self.ch().cr.modify(|_, w| w.htie().bit(enable)),
            Event::TransferComplete => self.ch().cr.modify(|_, w| w.tcie().bit(enable)),
            Event::TransferError => self.ch().cr.modify(|_, w| w.teie().bit(enable)),
            Event::Any => self.ch().cr.modify(|_, w| {
                w.htie().bit(enable);
                w.tcie().bit(enable);
                w.teie().bit(enable)
            }),
        }
    }

    /// Enable the interrupt for the given [`Event`].
    fn enable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, true);
    }

    /// Disable the interrupt for the given [`Event`].
    fn disable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, false);
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
    use crate::pac;

    /// Channel methods private to this module
    pub trait Channel {
        /// Return the register block for this channel
        fn ch(&self) -> &pac::dma1::CH;
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
        paste::paste! {
            #[doc = "All associated types, traits and methods of the `" $DMAx "` peripheral."]
            pub mod $dmax {
                use super::*;
                use crate::pac::$DMAx;
                use crate::rcc::Enable;

                impl DmaExt for $DMAx {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB) -> Channels {
                        <$DMAx>::enable(ahb);

                        let mut channels = Channels {
                            $( $chi: $Ci { _0: () }, )+
                        };

                        channels.reset();
                        channels
                    }
                }

                /// DMA channels
                #[derive(Debug)]
                #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                pub struct Channels {
                    $(
                        /// Channel
                        pub $chi: $Ci,
                    )+
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
                    #[derive(Debug)]
                    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
                    pub struct $Ci {
                        _0: (),
                    }

                    impl private::Channel for $Ci {
                        fn ch(&self) -> &pac::dma1::CH {
                            // SAFETY: $Ci grants exclusive access to this register
                            unsafe { &(*$DMAx::ptr()).$chi }
                        }
                    }

                    impl Channel for $Ci {
                        fn is_event_triggered(&self, event: Event) -> bool {
                            // SAFETY: atomic read
                            let flags = unsafe { (*$DMAx::ptr()).isr.read() };
                            match event {
                                Event::HalfTransfer => flags.$htifi().bit_is_set(),
                                Event::TransferComplete => flags.$tcifi().bit_is_set(),
                                Event::TransferError => flags.$teifi().bit_is_set(),
                                Event::Any => flags.$gifi().bit_is_set(),
                            }
                        }

                        fn clear_event(&mut self, event: Event) {
                            // SAFETY: atomic write to a stateless register
                            unsafe {
                                (*$DMAx::ptr()).ifcr.write(|w| match event {
                                    Event::HalfTransfer => w.$chtifi().set_bit(),
                                    Event::TransferComplete => w.$ctcifi().set_bit(),
                                    Event::TransferError => w.$cteifi().set_bit(),
                                    Event::Any => w.$cgifi().set_bit(),
                                });
                            }
                        }
                    }
                )+
            }
        }
    };

    ( $X:literal: {$($C:literal),+} ) => {
        paste::paste! {
            dma!(
                [<DMA $X>], [<dma $X>], [<dma $X en>],
                channels: {
                    $(
                        [<C $C>]:
			(
                            [<ch $C>],
                            [<htif $C>],
                            [<tcif $C>],
                            [<teif $C>],
                            [<gif $C>],
                            [<chtif $C>],
                            [<ctcif $C>],
                            [<cteif $C>],
                            [<cgif $C>]
                        ),
                    )+
                },
            );
        }
    };
}

dma!( 1: { 1,2,3,4,5,6,7 } );

#[cfg(any(feature = "gpio-f303", feature = "gpio-f303e",))]
dma!( 2: { 1,2,3,4,5 } );

/// Marker trait mapping DMA targets to their channels
pub trait OnChannel<C: Channel>: Target + crate::private::Sealed {}
// TODO: put this in serial mod
// TODO: the naming is switched
pub trait TransmitChannel: crate::private::Sealed {}
pub trait ReceiveChannel: crate::private::Sealed {}

macro_rules! on_channel {
    (
        $(
            $dma:ident: [$(($USART:ty, ($TxChannel:ident, $RxChannel:ident)),)+],
        ),+
    ) => {
        $(
            $(
                impl<Tx, Rx> crate::private::Sealed for serial::Serial<$USART, (Tx, Rx)> {}
                impl<Tx, Rx> OnChannel<$dma::$TxChannel> for serial::Serial<$USART, (Tx, Rx)> {}
                impl<Tx, Rx> OnChannel<$dma::$RxChannel> for serial::Serial<$USART, (Tx, Rx)> {}

                impl crate::private::Sealed for $dma::$TxChannel {}
                impl TransmitChannel for $dma::$TxChannel {}
                impl crate::private::Sealed for $dma::$RxChannel {}
                impl ReceiveChannel for $dma::$RxChannel {}
            )+
        )+
    };
}

// See mapping details in RM0316 13.4.7 Fig 47 onwards
on_channel!(
    dma1: [
        (pac::USART1, (C4, C5)),
        (pac::USART2, (C7, C6)),
        (pac::USART3, (C2, C3)),
    ],
);

#[cfg(any(feature = "gpio-f303", feature = "gpio-f303e",))]
on_channel!(
    dma2: [
        (pac::UART4, (C5, C3)),
    ],
);
