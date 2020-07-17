//! Direct memory access (DMA) controller
//!
//! Currently DMA is only supported for STM32F303 MCUs.

// To learn about most of the ideas implemented here, check out the DMA section
// of the Embedonomicon: https://docs.rust-embedded.org/embedonomicon/dma.html

use crate::{
    pac::{self, dma1::ch::cr},
    rcc::AHB,
    serial,
};
use cast::u16;
use core::{
    mem::{self, MaybeUninit},
    ops::{Deref, DerefMut},
    sync::atomic::{self, Ordering},
};
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
    /// Start a DMA write transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    pub fn start_write(mut buffer: B, mut channel: C, target: T) -> Self
    where
        B: WriteBuffer + 'static,
        T: Target<C>,
    {
        // NOTE(unsafe) cannot call `&mut self` methods on `buffer` because its
        // concrete type is unknown here
        let (ptr, len) = unsafe { buffer.write_buffer() };
        let len = u16(len).expect("buffer is too large");

        channel.set_memory_address(ptr as u32, Increment::Enable);
        channel.set_transfer_length(len);
        channel.set_word_size::<B::Word>();
        channel.set_direction(Direction::FromPeripheral);

        unsafe { Self::start(buffer, channel, target) }
    }

    /// Start a DMA read transfer.
    ///
    /// # Panics
    ///
    /// Panics if the buffer is longer than 65535 words.
    pub fn start_read(buffer: B, mut channel: C, target: T) -> Self
    where
        B: ReadBuffer + 'static,
        T: Target<C>,
    {
        // NOTE(unsafe) cannot call `&mut self` methods on `buffer` because its
        // concrete type is unknown here
        let (ptr, len) = unsafe { buffer.read_buffer() };
        let len = u16(len).expect("buffer is too large");

        channel.set_memory_address(ptr as u32, Increment::Enable);
        channel.set_transfer_length(len);
        channel.set_word_size::<B::Word>();
        channel.set_direction(Direction::FromMemory);

        unsafe { Self::start(buffer, channel, target) }
    }

    /// # Safety
    ///
    /// Callers must ensure that:
    ///
    /// - the given buffer will be valid for the duration of the transfer
    /// - the DMA channel is configured correctly for the given target and buffer
    unsafe fn start(buffer: B, mut channel: C, target: T) -> Self
    where
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
        inner.channel.event_occurred(Event::TransferComplete)
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

/// Trait for buffers that can be given to DMA for reading.
///
/// # Safety
///
/// The implementing type must be safe to use for DMA reads. This means:
///
/// - It must be a pointer that references the actual buffer.
/// - The following requirements must be fulfilled by `read_buffer`:
///   - The function must always return the same values, if called multiple
///     times.
///   - The memory specified by the returned pointer and size must not be
///     freed as long as `self` is not dropped.
pub unsafe trait ReadBuffer {
    type Word;

    /// Provide a buffer usable for DMA reads.
    ///
    /// The return value is:
    ///
    /// - pointer to the start of the buffer
    /// - buffer size in words
    ///
    /// # Safety
    ///
    /// Once this method has been called, it is unsafe to call any `&mut self`
    /// methods on this object as long as the returned value is used (for DMA).
    unsafe fn read_buffer(&self) -> (*const Self::Word, usize);
}

/// Trait for buffers that can be given to DMA for writing.
///
/// # Safety
///
/// The implementing type must be safe to use for DMA writes. This means:
///
/// - It must be a pointer that references the actual buffer.
/// - `Target` must be a type that is valid for any possible byte pattern.
/// - The following requirements must be fulfilled by `write_buffer`:
///   - The function must always return the same values, if called multiple
///     times.
///   - The memory specified by the returned pointer and size must not be
///     freed as long as `self` is not dropped.
pub unsafe trait WriteBuffer {
    type Word;

    /// Provide a buffer usable for DMA writes.
    ///
    /// The return value is:
    ///
    /// - pointer to the start of the buffer
    /// - buffer size in words
    ///
    /// # Safety
    ///
    /// Once this method has been called, it is unsafe to call any `&mut self`
    /// methods on this object as long as the returned value is used (for DMA)..
    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize);
}

// Blanked implementations for common DMA buffer types.

unsafe impl<B, T> ReadBuffer for B
where
    B: Deref<Target = T> + StableDeref,
    T: ReadTarget + ?Sized,
{
    type Word = T::Word;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        self.as_read_buffer()
    }
}

unsafe impl<B, T> WriteBuffer for B
where
    B: DerefMut<Target = T> + StableDeref,
    T: WriteTarget + ?Sized,
{
    type Word = T::Word;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        self.as_write_buffer()
    }
}

/// Trait for DMA word types used by the blanket DMA buffer impls.
///
/// # Safety
///
/// Types that implement this trait must be valid for every possible byte
/// pattern. This is to ensure that, whatever DMA writes into the buffer,
/// we won't get UB due to invalid values.
pub unsafe trait Word {}

unsafe impl Word for u8 {}
unsafe impl Word for u16 {}
unsafe impl Word for u32 {}

/// Trait for `Deref` targets used by the blanket `DmaReadBuffer` impl.
///
/// This trait exists solely to work around
/// https://github.com/rust-lang/rust/issues/20400.
///
/// # Safety
///
/// - `as_read_buffer` must adhere to the safety requirements
///   documented for `DmaReadBuffer::dma_read_buffer`.
pub unsafe trait ReadTarget {
    type Word: Word;

    fn as_read_buffer(&self) -> (*const Self::Word, usize) {
        let ptr = self as *const _ as *const Self::Word;
        let len = mem::size_of_val(self) / mem::size_of::<Self::Word>();
        (ptr, len)
    }
}

/// Trait for `DerefMut` targets used by the blanket `DmaWriteBuffer` impl.
///
/// This trait exists solely to work around
/// https://github.com/rust-lang/rust/issues/20400.
///
/// # Safety
///
/// - `as_write_buffer` must adhere to the safety requirements
///   documented for `DmaWriteBuffer::dma_write_buffer`.
pub unsafe trait WriteTarget {
    type Word: Word;

    fn as_write_buffer(&mut self) -> (*mut Self::Word, usize) {
        let ptr = self as *mut _ as *mut Self::Word;
        let len = mem::size_of_val(self) / mem::size_of::<Self::Word>();
        (ptr, len)
    }
}

unsafe impl<W: Word> ReadTarget for W {
    type Word = W;
}

unsafe impl<W: Word> WriteTarget for W {
    type Word = W;
}

unsafe impl<T: ReadTarget> ReadTarget for [T] {
    type Word = T::Word;
}

unsafe impl<T: WriteTarget> WriteTarget for [T] {
    type Word = T::Word;
}

macro_rules! dma_target_array_impls {
    ( $( $i:expr, )+ ) => {
        $(
            unsafe impl<T: ReadTarget> ReadTarget for [T; $i] {
                type Word = T::Word;
            }

            unsafe impl<T: WriteTarget> WriteTarget for [T; $i] {
                type Word = T::Word;
            }
        )+
    };
}

#[rustfmt::skip]
dma_target_array_impls!(
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,
    10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
    20,  21,  22,  23,  24,  25,  26,  27,  28,  29,
    30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,
    50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
    60,  61,  62,  63,  64,  65,  66,  67,  68,  69,
    70,  71,  72,  73,  74,  75,  76,  77,  78,  79,
    80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
    90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
   100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
   110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
   120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
   130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
   140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
   150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
   160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
   170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
   180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
   190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
   200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
   210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
   220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
   230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
   240, 241, 242, 243, 244, 245, 246, 247, 248, 249,
   250, 251, 252, 253, 254, 255, 256,

   1 <<  9,
   1 << 10,
   1 << 11,
   1 << 12,
   1 << 13,
   1 << 14,
   1 << 15,
   1 << 16,
);

unsafe impl<T: WriteTarget> WriteTarget for MaybeUninit<T> {
    type Word = T::Word;
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
    fn event_occurred(&self, event: Event) -> bool;

    /// Clear the interrupt flag for the given event.
    ///
    /// Passing `Event::Any` clears all interrupt flags.
    ///
    /// Note that the the global interrupt flag is not automatically cleared
    /// even when all other flags are cleared. The only way to clear it is to
    /// call this method with `Event::Any`.
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
    fn set_transfer_length(&mut self, len: u16) {
        assert!(!self.is_enabled());

        self.ch().ndtr.write(|w| w.ndt().bits(len));
    }

    /// Set the word size.
    ///
    /// # Panics
    ///
    /// Panics if the word size is not one of 8, 16, or 32 bits.
    fn set_word_size<W>(&mut self) {
        use cr::PSIZE_A::*;

        let psize = match mem::size_of::<W>() {
            1 => BITS8,
            2 => BITS16,
            4 => BITS32,
            s => panic!("unsupported word size: {}", s),
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
        pub mod $dmax {
            use super::*;
            use crate::pac::$DMAx;

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
                    fn ch(&self) -> &pac::dma1::CH {
                        // NOTE(unsafe) $Ci grants exclusive access to this register
                        unsafe { &(*$DMAx::ptr()).$chi }
                    }
                }

                impl Channel for $Ci {
                    fn event_occurred(&self, event: Event) -> bool {
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
    serial::Rx<pac::USART1> => C5,
    serial::Tx<pac::USART1> => C4,
    serial::Rx<pac::USART2> => C6,
    serial::Tx<pac::USART2> => C7,
    serial::Rx<pac::USART3> => C3,
    serial::Tx<pac::USART3> => C2,
);
