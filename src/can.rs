//! Controller Area Network
//!
//! CAN is currently not enabled by default, and
//! can be enabled by the `can` feature.
//!
//! It is a implementation of the [`embedded_hal_can`][can] traits.
//!
//! [can]: embedded_hal_can
//!
//! A usage example of the can peripheral can be found at [examples/can.rs]
//!
//! [examples/can.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.0/examples/can.rs

pub use embedded_hal_can::{self, Filter, Frame, Id, Receiver, Transmitter};

use crate::gpio::gpioa;
use crate::gpio::AF9;
use crate::rcc::APB1;
use crate::stm32;
use nb::{self, Error};

use core::sync::atomic::{AtomicU8, Ordering};

const EXID_MASK: u32 = 0b1_1111_1111_1100_0000_0000_0000_0000;
const MAX_EXTENDED_ID: u32 = 0x1FFF_FFFF;

/// A CAN identifier, which can be either 11 or 27 (extended) bits.
/// u16 and u32 respectively are used here despite the fact that the upper bits are unused.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum CanId {
    /// Standard 11bit Identifier (0..=0x7FF)
    BaseId(u16),
    /// Extended 29bit Identifier (0..=0x1FFF_FFFF)
    ExtendedId(u32),
}

/// A CAN frame consisting of a destination ID and up to 8 bytes of data.
///
/// Currently, we always allocate a fixed size array for each frame regardless
/// of actual size, but this could be improved in the future using const-generics.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct CanFrame {
    /// CAN Frame ID
    id: CanId,
    /// Data Length Code (range 0..=8)
    dlc: usize,
    /// Data Frame
    data: [u8; 8],
}

/// Can Frame Filter Mode
///
/// Represents the operating mode of a CAN filter, which can either contain a
/// list of identifiers, or a mask to match on.
pub enum FilterMode {
    /// Filter on a given Mask
    Mask,
    /// Filter on a list of identifiers
    List,
}

/// A fully specified CAN filter with its associated list of of IDs or mask.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum CanFilterData {
    /// Can Frame Identifier Filter
    IdFilter(CanId),
    /// Filter for an identifier with a applied mask
    MaskFilter(u16, u16),
    /// Filter for an extended identifier with a applied mask
    ExtendedMaskFilter(u32, u32),
    /// Do not filter
    AcceptAll,
}

/// CAN Filter type
///
/// Used to specify the filter behavior
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct CanFilter {
    data: CanFilterData,
    index: Option<u8>,
}

static FILTER_INDEX: AtomicU8 = AtomicU8::new(0);

/// Controll Area Network (CAN) Peripheral
pub struct Can {
    can: stm32::CAN,
    _rx: gpioa::PA11<AF9>,
    _tx: gpioa::PA12<AF9>,
}

/// A CAN FIFO which is used to receive and buffer messages from the CAN
/// network that match on of the assigned filters.
pub struct CanFifo {
    idx: usize,
}

/// A CAN transmitter which is used to send messages to the CAN network.
pub struct CanTransmitter {
    _can: stm32::CAN,
    _rx: gpioa::PA11<AF9>,
    _tx: gpioa::PA12<AF9>,
}

// TODO Use wrapper type around interal pac definition
/// CAN Interrupt events
pub enum Event {
    /// FIFO 0 message pending interrupt
    Fifo0Fmp,
    /// FIFO 1 message pending interrupt
    Fifo1Fmp,
    /// FIFO 0 full interupt enable
    Fifo0Full,
    /// FIFO 1 full interupt enable
    Fifo1Full,
    /// FIFO 0 overrun interrupt enable
    Fifo0Ovr,
    /// FIFO 1 overrun interrupt enable
    Fifo1Ovr,
    /// Transmit mailbox empty interrupt
    Txe,
}

impl embedded_hal_can::Id for CanId {
    type BaseId = u16;
    type ExtendedId = u32;

    fn base_id(&self) -> Option<<CanId as Id>::BaseId> {
        match self {
            CanId::BaseId(v) => Some(*v),
            CanId::ExtendedId(_) => None,
        }
    }

    fn extended_id(&self) -> Option<<CanId as Id>::ExtendedId> {
        match self {
            CanId::BaseId(_) => None,
            CanId::ExtendedId(v) => Some(*v),
        }
    }
}

impl embedded_hal_can::Frame for CanFrame {
    type Id = CanId;

    #[inline(always)]
    fn is_remote_frame(&self) -> bool {
        self.data().is_none()
    }

    #[inline(always)]
    fn is_data_frame(&self) -> bool {
        self.data().is_some()
    }

    fn id(&self) -> <CanFrame as Frame>::Id {
        self.id
    }

    #[inline(always)]
    fn data(&self) -> Option<&[u8]> {
        if self.dlc > 0 {
            Some(&self.data)
        } else {
            None
        }
    }
}

impl embedded_hal_can::Filter for CanFilter {
    type Id = CanId;

    /// Construct a filter which filters messages for a specific identified
    fn from_id(id: Self::Id) -> Self {
        CanFilter::new(CanFilterData::IdFilter(id))
    }

    /// Construct an "empty" filter which will accept all messages
    fn accept_all() -> Self {
        CanFilter::new(CanFilterData::AcceptAll)
    }

    // TODO: Constructing filters like this is fairly limiting because ideally
    // we would have the full "filter state" available, so for non-extended
    // filters this could be 2 masks and filters or 4 ids for id lists

    /// Constuct a mask filter. This method accepts two parameters, the mask which designates which
    /// bits are actually matched againts and the filter, with the actual bits to match.
    fn from_mask(mask: u32, filter: u32) -> Self {
        crate::assert!(
            mask < MAX_EXTENDED_ID,
            "Mask cannot have bits higher than 29"
        );

        // If the mask has any significant (i.e. 1) bits in the upper bits, this filter is extended
        if (mask & EXID_MASK) > 0 {
            CanFilter::new(CanFilterData::ExtendedMaskFilter(mask, filter))
        } else {
            CanFilter::new(CanFilterData::MaskFilter(mask as u16, filter as u16))
        }
    }
}

impl CanFilter {
    /// Create a new filter with no assigned index.
    ///
    /// To actually active the filter call
    /// [`Receiver::set_filter`], which will assign an index.
    pub fn new(data: CanFilterData) -> CanFilter {
        CanFilter { data, index: None }
    }

    /// Create a new filter with a specified filter index.
    pub fn new_with_index(index: u8, data: CanFilterData) -> CanFilter {
        CanFilter {
            data,
            index: Some(index),
        }
    }
}

impl CanFilterData {
    fn fr1(&self) -> u32 {
        match self {
            CanFilterData::AcceptAll => 0,
            CanFilterData::ExtendedMaskFilter(filter, _) => filter << 3,
            CanFilterData::MaskFilter(filter, mask) => {
                // Only use lower 16 bits
                let shifted_filter = ((*filter as u32) << 5) & (u16::max_value() as u32);
                let shifted_mask = ((*mask as u32) << 5) << 16;

                shifted_filter | shifted_mask
            }
            CanFilterData::IdFilter(id) => match id {
                CanId::BaseId(_base_id) => todo!("Shift base ID correctly!"),
                CanId::ExtendedId(ext_id) => ext_id << 3,
            },
        }
    }

    fn fr2(&self) -> Option<u32> {
        match self {
            CanFilterData::AcceptAll => Some(0),
            CanFilterData::ExtendedMaskFilter(_, mask) => Some(mask << 3),
            // TODO: We should be able to fill this register with a second filter/mask pair
            CanFilterData::MaskFilter(_, _mask) => None,
            // TODO: This sucks, we need more info here to figure out the correct value of fr2
            CanFilterData::IdFilter(_id) => None,
        }
    }
}

impl Can {
    /// Initialize the CAN Peripheral
    pub fn new(
        can: stm32::CAN,
        rx: gpioa::PA11<AF9>,
        tx: gpioa::PA12<AF9>,
        apb1: &mut APB1,
    ) -> Self {
        apb1.enr().modify(|_, w| w.canen().enabled());
        can.mcr.modify(|_, w| w.sleep().clear_bit());
        can.mcr.modify(|_, w| w.inrq().set_bit());

        // Wait for INAK to confirm we have entered initialization mode
        while !can.msr.read().inak().bit_is_set() {}

        // TODO: actually calculate baud params

        // Our baud rate calc here is aiming for roughly 4000uS total bit time or about 250kbps
        // Though we actually allow closer to 5500uS total given the sjw setting
        // Calculations for timing value from http://www.bittiming.can-wiki.info/#bxCAN

        // Baud rate prescaler defines time quanta
        // tq = (BRP[9:0]+1) x tPCLK
        let brp: u16 = 4;

        // Resynchronization jump width: number of quanta segments may be expanded to resync
        // tRJW = tq x (SJW[1:0] + 1)
        let sjw = 0;

        // Time seg 2
        // tBS2 = tq x (TS2[2:0] + 1)
        let ts2 = 3;

        // Time seg 1
        // tBS1 = tq x (TS1[3:0] + 1)
        let ts1 = 10;

        can.btr.modify(|_, w| unsafe {
            w.brp()
                .bits(brp)
                .sjw()
                .bits(sjw)
                .ts1()
                .bits(ts1)
                .ts2()
                .bits(ts2)
            //.lbkm()
            //.set_bit()
        });

        // Leave initialization mode by clearing INRQ and switch to normal mode
        can.mcr.modify(|_, w| w.inrq().clear_bit());
        // Wait for INAK again to confirm the mode switch
        while !can.msr.read().inak().bit_is_clear() {}

        Can {
            can,
            _rx: rx,
            _tx: tx,
        }
    }

    /// Enable CAN event interrupts for `Event`
    pub fn listen(&mut self, event: Event) {
        self.can.ier.modify(|_, w| match event {
            Event::Fifo0Fmp => w.fmpie0().set_bit(),
            Event::Fifo0Full => w.ffie0().set_bit(),
            Event::Fifo0Ovr => w.fovie0().set_bit(),
            Event::Txe => w.tmeie().set_bit(),
            _ => unimplemented!(),
        });
    }

    /// Split the CAN peripheral into a transmitter and associated FIFOs.
    pub fn split(self) -> (CanTransmitter, CanFifo, CanFifo) {
        let fifo0 = CanFifo { idx: 0 };
        let fifo1 = CanFifo { idx: 1 };

        let transmitter = CanTransmitter {
            _can: self.can,
            _rx: self._rx,
            _tx: self._tx,
        };

        (transmitter, fifo0, fifo1)
    }

    /// Release owned peripherals
    pub fn free(self) -> (stm32::CAN, gpioa::PA11<AF9>, gpioa::PA12<AF9>) {
        (self.can, self._rx, self._tx)
    }
}

impl embedded_hal_can::Interface for CanTransmitter {
    type Id = CanId;
    type Frame = CanFrame;
    type Error = ();
    type Filter = CanFilter;
}

impl embedded_hal_can::Interface for CanFifo {
    type Id = CanId;
    type Frame = CanFrame;
    type Error = ();
    type Filter = CanFilter;
}

impl embedded_hal_can::Transmitter for CanTransmitter {
    fn transmit(
        &mut self,
        frame: &Self::Frame,
    ) -> Result<Option<Self::Frame>, nb::Error<Self::Error>> {
        let can = unsafe { &*stm32::CAN::ptr() };

        for tx_idx in 0..3 {
            let free = match tx_idx {
                0 => can.tsr.read().tme0().bit_is_set(),
                1 => can.tsr.read().tme1().bit_is_set(),
                2 => can.tsr.read().tme2().bit_is_set(),
                _ => crate::unreachable!(),
            };

            if !free {
                continue;
            }

            let tx = &can.tx[tx_idx];

            match frame.id {
                CanId::BaseId(id) => tx.tir.modify(|_, w| unsafe { w.stid().bits(id) }),
                CanId::ExtendedId(id) => tx.tir.modify(|_, w| unsafe {
                    let hb = ((id & EXID_MASK) >> 18) as u16;
                    w.exid().bits(id).stid().bits(hb).ide().set_bit()
                }),
            }

            if let Some(data) = frame.data() {
                for (i, d) in data.iter().enumerate() {
                    // NOTE(unsafe): full 8bit write is unsafe via the svd2rust api
                    unsafe {
                        match i {
                            0 => tx.tdlr.modify(|_, w| w.data0().bits(*d)),
                            1 => tx.tdlr.modify(|_, w| w.data1().bits(*d)),
                            2 => tx.tdlr.modify(|_, w| w.data2().bits(*d)),
                            3 => tx.tdlr.modify(|_, w| w.data3().bits(*d)),
                            4 => tx.tdhr.modify(|_, w| w.data4().bits(*d)),
                            5 => tx.tdhr.modify(|_, w| w.data5().bits(*d)),
                            6 => tx.tdhr.modify(|_, w| w.data6().bits(*d)),
                            7 => tx.tdhr.modify(|_, w| w.data7().bits(*d)),
                            _ => crate::unreachable!(),
                        }
                    }
                }

                // NOTE(unsafe): full 8bit write is unsafe via the svd2rust api
                tx.tdtr
                    .modify(|_, w| unsafe { w.dlc().bits(data.len() as u8) });

                tx.tir.modify(|_, w| w.rtr().clear_bit());
            } else {
                tx.tir.modify(|_, w| w.rtr().set_bit());
            }

            // Request the frame be sent!
            tx.tir.modify(|_, w| w.txrq().set_bit());

            return Ok(None);
        }

        Err(nb::Error::WouldBlock)
    }
}

impl Receiver for CanFifo {
    fn receive(&mut self) -> Result<Self::Frame, Error<Self::Error>> {
        let can = unsafe { &*stm32::CAN::ptr() };

        let rx = &can.rx[self.idx];
        if can.rfr[self.idx].read().fmp().bits() > 0 {
            let mut data: [u8; 8] = [0; 8];

            let len: usize = rx.rdtr.read().dlc().bits().into();

            let data_low = rx.rdlr.read();
            let data_high = rx.rdhr.read();

            for i in 0..len {
                match i {
                    0 => data[0] = data_low.data0().bits(),
                    1 => data[1] = data_low.data1().bits(),
                    2 => data[2] = data_low.data2().bits(),
                    3 => data[3] = data_low.data3().bits(),
                    4 => data[4] = data_high.data4().bits(),
                    5 => data[5] = data_high.data5().bits(),
                    6 => data[6] = data_high.data6().bits(),
                    7 => data[7] = data_high.data7().bits(),
                    _ => crate::unreachable!(),
                }
            }

            let rcv_id = match rx.rir.read().ide().bit_is_set() {
                true => {
                    let lower = rx.rir.read().exid().bits();
                    let upper = (rx.rir.read().stid().bits() as u32) << 18;
                    CanId::ExtendedId(upper | lower)
                }
                false => CanId::BaseId(rx.rir.read().stid().bits()),
            };

            // Release the mailbox
            can.rfr[self.idx].modify(|_, w| w.rfom().set_bit());

            let frame = CanFrame {
                id: rcv_id,
                dlc: len,
                data,
            };

            return Ok(frame);
        }

        Err(nb::Error::WouldBlock)
    }

    /// Sets a filter in the next open filter register.
    fn set_filter(&mut self, filter: Self::Filter) {
        cortex_m::interrupt::free(|_cs| {
            let can = unsafe { &*stm32::CAN::ptr() };

            // Filter init mode
            can.fmr.modify(|_, w| w.finit().set_bit());

            can.fm1r.modify(|_, w| match filter.data {
                CanFilterData::MaskFilter(_, _) => w.fbm0().clear_bit(),
                CanFilterData::ExtendedMaskFilter(_, _) => w.fbm0().clear_bit(),
                CanFilterData::AcceptAll => w.fbm0().clear_bit(),
                CanFilterData::IdFilter(_) => w.fbm0().set_bit(),
            });

            can.fs1r.modify(|_, w| match filter.data {
                CanFilterData::MaskFilter(_, _) => w.fsc0().clear_bit(),
                CanFilterData::ExtendedMaskFilter(_, _) => w.fsc0().set_bit(),
                CanFilterData::AcceptAll => w.fsc0().set_bit(),
                CanFilterData::IdFilter(id) => match id {
                    CanId::BaseId(_) => w.fsc0().clear_bit(),
                    CanId::ExtendedId(_) => w.fsc0().set_bit(),
                },
            });

            // Assign filter to this FIFO
            can.ffa1r.modify(|_, w| match self.idx {
                0 => w.ffa0().clear_bit(),
                1 => w.ffa0().set_bit(),
                _ => crate::unreachable!(),
            });

            let index = filter
                .index
                .unwrap_or_else(|| FILTER_INDEX.fetch_add(1, Ordering::Acquire))
                as usize;

            crate::assert!(index < 28, "Filter index out of range");

            can.fb[index]
                .fr1
                .modify(|_, w| unsafe { w.bits(filter.data.fr1()) });

            let fr2 = filter.data.fr2().unwrap_or(0);
            can.fb[index].fr2.modify(|_, w| unsafe { w.bits(fr2) });

            can.fa1r.modify(|_, w| {
                // TODO: the rest of these
                match index {
                    0 => w.fact0().set_bit(),
                    1 => w.fact1().set_bit(),
                    2 => w.fact2().set_bit(),
                    3 => w.fact3().set_bit(),
                    4 => w.fact4().set_bit(),
                    5 => w.fact5().set_bit(),
                    _ => unimplemented!(),
                }
            });

            // Disable init mode
            can.fmr.modify(|_, w| w.finit().clear_bit());
        });
    }

    fn clear_filter(&mut self) {
        self.set_filter(CanFilter::new(CanFilterData::AcceptAll))
    }
}

impl CanFrame {
    /// Create a new Can Frame
    ///
    /// `src` content is copied into internal buffer
    /// and handled internally.
    ///
    /// # Panics
    ///
    /// This function will panic if length of `data` is greater than `8`
    pub fn new_data(id: CanId, data: &[u8]) -> CanFrame {
        crate::assert!((0..8).contains(&data.len()));

        let mut frame = Self {
            id,
            dlc: data.len(),
            data: [0; 8],
        };
        frame.data[0..data.len()].copy_from_slice(data);
        frame
    }

    /// Crate a new remote Can Frame
    ///
    /// # Panics
    ///
    /// This function will panic if `dlc` is not inside the vliad range `0..=8`.
    pub fn new_remote(id: CanId, dlc: usize) -> CanFrame {
        assert!((0..=8).contains(&dlc));

        CanFrame {
            id,
            dlc,
            data: [0; 8],
        }
    }
}
