<<<<<<< HEAD
pub use canbus::{self, Filter, Frame, Id, Receiver, Transmitter};
=======
//! Controller Area Network
pub use embedded_hal_can::{self, Filter, Frame, Id, Receiver, Transmitter};
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19

use crate::gpio::gpioa;
use crate::gpio::AF9;
use crate::rcc::APB1;
use crate::stm32;
<<<<<<< HEAD
use nb;
use nb::Error;

use core::sync::atomic::{AtomicU8, Ordering};
use stm32f3::stm32f302::CAN;
=======
use heapless::{consts::U8, Vec};
use nb::{self, Error};

use core::sync::atomic::{AtomicU8, Ordering};
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19

const EXID_MASK: u32 = 0b11111_11111100_00000000_00000000;
const MAX_EXTENDED_ID: u32 = 0x1FFF_FFFF;

<<<<<<< HEAD
#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
=======
/// A CAN identifier, which can be either 11 or 27 (extended) bits. u16 and u32 respectively are used here despite the fact that the upper bits are unused.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub enum CanId {
    BaseId(u16),
    ExtendedId(u32),
}

<<<<<<< HEAD
#[derive(Debug, Clone, Hash, Eq, PartialEq)]
pub struct CanFrame {
    pub id: CanId,
    pub data: [u8; 8],
    pub length: usize,
}

=======
/// A CAN frame consisting of a destination ID and up to 8 bytes of data.
///
/// Currently, we always allocate a fixed size array for each frame regardless of actual size, but this could be improved in the future using const-generics.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct CanFrame {
    pub id: CanId,
    pub data: Vec<u8, U8>,
}

/// Represents the operating mode of a CAN filter, which can either contain a list of identifiers, or a mask to match on.
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub enum FilterMode {
    Mask,
    List,
}

<<<<<<< HEAD
#[derive(Copy, Clone, Debug, Hash, Eq, PartialEq)]
=======
/// A fully specified CAN filter with its associated list of of IDs or mask.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub enum CanFilterData {
    IdFilter(CanId),
    MaskFilter(u16, u16),
    ExtendedMaskFilter(u32, u32),
    AcceptAll,
}

<<<<<<< HEAD
#[derive(Copy, Clone, Debug, Hash, Eq, PartialEq)]
=======
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub struct CanFilter {
    data: CanFilterData,
    index: Option<u8>,
}

static FILTER_INDEX: AtomicU8 = AtomicU8::new(0);

pub struct Can {
    can: stm32::CAN,
    _rx: gpioa::PA11<AF9>,
    _tx: gpioa::PA12<AF9>,
}

<<<<<<< HEAD
=======
/// A CAN FIFO which is used to receive and buffer messages from the CAN network that match on of the assigned filters.
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub struct CanFifo {
    idx: usize,
}

<<<<<<< HEAD
=======
/// A CAN transmitter which is used to send messages to the CAN network.
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub struct CanTransmitter {
    _can: stm32::CAN,
    _rx: gpioa::PA11<AF9>,
    _tx: gpioa::PA12<AF9>,
}

<<<<<<< HEAD
=======
/// CAN Interrupt events
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
pub enum Event {
    Fifo0Fmp,
    Fifo1Fmp,
    Fifo0Full,
    Fifo1Full,
    Fifo0Ovr,
    Fifo1Ovr,
    Txe,
}

<<<<<<< HEAD
impl canbus::Id for CanId {
=======
impl embedded_hal_can::Id for CanId {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
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

<<<<<<< HEAD
impl canbus::Frame for CanFrame {
=======
impl embedded_hal_can::Frame for CanFrame {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
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
<<<<<<< HEAD
        if self.length > 0 {
            Some(&self.data[0..self.length])
=======
        if self.data.len() > 0 {
            Some(&self.data)
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
        } else {
            None
        }
    }
}

<<<<<<< HEAD
impl canbus::Filter for CanFilter {
=======
impl embedded_hal_can::Filter for CanFilter {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    type Id = CanId;

    /// Construct a filter which filters messages for a specific identified
    fn from_id(id: Self::Id) -> Self {
        CanFilter::new(CanFilterData::IdFilter(id))
    }

    /// Construct an "empty" filter which will accept all messages
    fn accept_all() -> Self {
        CanFilter::new(CanFilterData::AcceptAll)
    }

    // TODO: Constructing filters like this is fairly limiting because ideally we would have the full "filter state" available, so for non-extended filters this could be 2 masks and filters or 4 ids for id lists

    /// Constuct a mask filter. This method accepts two parameters, the mask which designates which bits are actually matched againts and the filter, with the actual bits to match.
    fn from_mask(mask: u32, filter: u32) -> Self {
        assert!(
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
<<<<<<< HEAD
=======
    /// Create a new filter with no assigned index. To actually active the filter call `Receiver::set_filter`, which will assign an index.
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    pub fn new(data: CanFilterData) -> CanFilter {
        CanFilter { data, index: None }
    }

<<<<<<< HEAD
=======
    /// Create a new filter with a specified filter index.
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
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
<<<<<<< HEAD
            CanFilterData::MaskFilter(_filter, _) => todo!("Shift filter and mask correctly"),
=======
            CanFilterData::MaskFilter(filter, mask) => {
                let shifted_filter = ((*filter as u32) << 5) & (u16::max_value() as u32); // Only use lower 16 bits
                let shifted_mask = ((*mask as u32) << 5) << 16;

                shifted_filter | shifted_mask
            }
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
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
<<<<<<< HEAD
            CanFilterData::MaskFilter(_, _mask) => todo!("Shift filter and mask correctly"),
=======
            CanFilterData::MaskFilter(_, _mask) => None, // TODO: We should be able to fill this register with a second filter/mask pair
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
            CanFilterData::IdFilter(_id) => None, // TODO: This sucks, we need more info here to figure out the correct value of fr2
        }
    }
}

impl Can {
<<<<<<< HEAD
    pub fn new(
=======
    pub fn can(
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
        can: stm32::CAN,
        rx: gpioa::PA11<AF9>,
        tx: gpioa::PA12<AF9>,
        apb1: &mut APB1,
    ) -> Can {
<<<<<<< HEAD
        apb1.enr().modify(|_, w| w.canen().set_bit());
=======
        apb1.enr().modify(|_, w| w.canen().enabled());
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
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

<<<<<<< HEAD
=======
    /// Enable CAN event interrupts for `Event`
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    pub fn listen(&mut self, event: Event) {
        self.can.ier.modify(|_, w| match event {
            Event::Fifo0Fmp => w.fmpie0().set_bit(),
            Event::Fifo0Full => w.ffie0().set_bit(),
            Event::Fifo0Ovr => w.fovie0().set_bit(),
            Event::Txe => w.tmeie().set_bit(),
            _ => unimplemented!(),
        });
    }

<<<<<<< HEAD
=======
    /// Split the CAN peripheral into a transmitter and associated FIFOs.
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    pub fn split(self) -> (CanTransmitter, CanFifo, CanFifo) {
        let fifo0 = CanFifo { idx: 0 };
        let fifo1 = CanFifo { idx: 1 };

        let transmitter = CanTransmitter {
            _can: self.can,
            _rx: self._rx,
            _tx: self._tx,
        };

        return (transmitter, fifo0, fifo1);
    }

    pub fn free(self) -> (stm32::CAN, gpioa::PA11<AF9>, gpioa::PA12<AF9>) {
        (self.can, self._rx, self._tx)
    }
}

<<<<<<< HEAD
impl canbus::Interface for CanTransmitter {
=======
impl embedded_hal_can::Interface for CanTransmitter {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    type Id = CanId;
    type Frame = CanFrame;
    type Error = ();
    type Filter = CanFilter;
}

<<<<<<< HEAD
impl canbus::Interface for CanFifo {
=======
impl embedded_hal_can::Interface for CanFifo {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    type Id = CanId;
    type Frame = CanFrame;
    type Error = ();
    type Filter = CanFilter;
}

<<<<<<< HEAD
impl canbus::Transmitter for CanTransmitter {
=======
impl embedded_hal_can::Transmitter for CanTransmitter {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    fn transmit(
        &mut self,
        frame: &Self::Frame,
    ) -> Result<Option<Self::Frame>, nb::Error<Self::Error>> {
<<<<<<< HEAD
        if let Some(data) = frame.data() {
            assert!(
                data.len() < 8,
                "CanFrame cannot contain more than 8 bytes of data"
            );
        }

        let can = unsafe { &*CAN::ptr() };

        for i in 0..3 {
            let free = match i {
=======
        let can = unsafe { &*stm32::CAN::ptr() };

        for tx_idx in 0..3 {
            let free = match tx_idx {
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
                0 => can.tsr.read().tme0().bit_is_set(),
                1 => can.tsr.read().tme1().bit_is_set(),
                2 => can.tsr.read().tme2().bit_is_set(),
                _ => unreachable!(),
            };

            if !free {
                continue;
            }

<<<<<<< HEAD
            let tx = &can.tx[i];
=======
            let tx = &can.tx[tx_idx];
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19

            match frame.id {
                CanId::BaseId(id) => tx.tir.modify(|_, w| unsafe { w.stid().bits(id) }),
                CanId::ExtendedId(id) => tx.tir.modify(|_, w| unsafe {
                    let hb = ((id & EXID_MASK) >> 18) as u16;
                    w.exid().bits(id).stid().bits(hb).ide().set_bit()
                }),
            }

<<<<<<< HEAD
            match frame.data() {
                Some(d) => unsafe {
                    d.iter().enumerate().for_each(|(j, val)| match j {
                        0 => tx.tdlr.modify(|_, w| w.data0().bits(*val)),
                        1 => tx.tdlr.modify(|_, w| w.data1().bits(*val)),
                        2 => tx.tdlr.modify(|_, w| w.data2().bits(*val)),
                        3 => tx.tdlr.modify(|_, w| w.data3().bits(*val)),
                        4 => tx.tdhr.modify(|_, w| w.data4().bits(*val)),
                        5 => tx.tdhr.modify(|_, w| w.data5().bits(*val)),
                        6 => tx.tdhr.modify(|_, w| w.data6().bits(*val)),
                        7 => tx.tdhr.modify(|_, w| w.data7().bits(*val)),
                        _ => unreachable!(),
                    });

                    tx.tdtr.modify(|_, w| w.dlc().bits(d.len() as u8));

                    tx.tir.modify(|_, w| w.rtr().clear_bit());
                },
                None => tx.tir.modify(|_, w| w.rtr().set_bit()),
=======
            if let Some(_) = frame.data() {
                for j in 0..frame.data.len() {
                    let val = &frame.data[j];

                    // NOTE(unsafe): full 8bit write is unsafe via the svd2rust api
                    unsafe {
                        match j {
                            0 => tx.tdlr.modify(|_, w| w.data0().bits(*val)),
                            1 => tx.tdlr.modify(|_, w| w.data1().bits(*val)),
                            2 => tx.tdlr.modify(|_, w| w.data2().bits(*val)),
                            3 => tx.tdlr.modify(|_, w| w.data3().bits(*val)),
                            4 => tx.tdhr.modify(|_, w| w.data4().bits(*val)),
                            5 => tx.tdhr.modify(|_, w| w.data5().bits(*val)),
                            6 => tx.tdhr.modify(|_, w| w.data6().bits(*val)),
                            7 => tx.tdhr.modify(|_, w| w.data7().bits(*val)),
                            _ => unreachable!(),
                        }
                    }
                }

                // NOTE(unsafe): full 8bit write is unsafe via the svd2rust api
                tx.tdtr
                    .modify(|_, w| unsafe { w.dlc().bits(frame.data.len() as u8) });

                tx.tir.modify(|_, w| w.rtr().clear_bit());
            } else {
                tx.tir.modify(|_, w| w.rtr().set_bit());
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
            }

            // Request the frame be sent!
            tx.tir.modify(|_, w| w.txrq().set_bit());

<<<<<<< HEAD
            // TODO: we can prob not wait for txok?
            while match i {
                0 => can.tsr.read().txok0().bit_is_clear(),
                1 => can.tsr.read().txok1().bit_is_clear(),
                2 => can.tsr.read().txok2().bit_is_clear(),
                _ => unreachable!(),
            } {}

=======
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
            return Ok(None);
        }

        Err(nb::Error::WouldBlock)
    }
}

impl Receiver for CanFifo {
    fn receive(&mut self) -> Result<Self::Frame, Error<Self::Error>> {
<<<<<<< HEAD
        let can = unsafe { (&*CAN::ptr()) };

        let rx = &can.rx[self.idx];
        if (can).rfr[self.idx].read().fmp().bits() > 0 {
            let mut data: [u8; 8] = [0u8; 8];
=======
        let can = unsafe { &*stm32::CAN::ptr() };

        let rx = &can.rx[self.idx];
        if can.rfr[self.idx].read().fmp().bits() > 0 {
            let mut data = Vec::<_, U8>::new();
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19

            let len = rx.rdtr.read().dlc().bits() as usize;

            let data_low = rx.rdlr.read();
            let data_high = rx.rdhr.read();

            for i in 0..len {
                match i {
<<<<<<< HEAD
                    0 => data[i] = data_low.data0().bits(),
                    1 => data[i] = data_low.data1().bits(),
                    2 => data[i] = data_low.data2().bits(),
                    3 => data[i] = data_low.data3().bits(),
                    4 => data[i] = data_high.data4().bits(),
                    5 => data[i] = data_high.data5().bits(),
                    6 => data[i] = data_high.data6().bits(),
                    7 => data[i] = data_high.data7().bits(),
=======
                    0 => data.push(data_low.data0().bits()).unwrap(),
                    1 => data.push(data_low.data1().bits()).unwrap(),
                    2 => data.push(data_low.data2().bits()).unwrap(),
                    3 => data.push(data_low.data3().bits()).unwrap(),
                    4 => data.push(data_high.data4().bits()).unwrap(),
                    5 => data.push(data_high.data5().bits()).unwrap(),
                    6 => data.push(data_high.data6().bits()).unwrap(),
                    7 => data.push(data_high.data7().bits()).unwrap(),
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
                    _ => unreachable!(),
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

<<<<<<< HEAD
            let frame = CanFrame::data_frame(rcv_id, &data);
=======
            let frame = CanFrame { id: rcv_id, data };
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19

            return Ok(frame);
        }

        Err(nb::Error::WouldBlock)
    }

    /// Sets a filter in the next open filter register.
    fn set_filter(&mut self, filter: Self::Filter) {
<<<<<<< HEAD
        // TODO: this likely needs to be in a critical section, is that OK?

        let can = unsafe { (&*CAN::ptr()) };

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
            _ => unreachable!(),
        });

        let index = filter
            .index
            .unwrap_or_else(|| FILTER_INDEX.fetch_add(1, Ordering::Acquire))
            as usize;

        assert!(index < 28, "Filter index out of range");

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
=======
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
                _ => unreachable!(),
            });

            let index = filter
                .index
                .unwrap_or_else(|| FILTER_INDEX.fetch_add(1, Ordering::Acquire))
                as usize;

            assert!(index < 28, "Filter index out of range");

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
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    }

    fn clear_filter(&mut self) {
        self.set_filter(CanFilter::new(CanFilterData::AcceptAll))
    }
}

impl CanFrame {
<<<<<<< HEAD
    pub fn data_frame(id: CanId, data: &[u8]) -> CanFrame {
        let mut frame_data = [0u8; 8];
        frame_data.clone_from_slice(data);

        CanFrame {
            id,
            data: frame_data,
            length: data.len(),
        }
=======
    pub fn new_with_len(id: CanId, src: &[u8], length: usize) -> CanFrame {
        assert!(length <= 8, "CAN Frames can have at most 8 data bytes");

        let mut data = Vec::<u8, U8>::new();

        // The vector is always empty and the data size has alreay been checked, this will always succeed
        data.extend_from_slice(src).unwrap();

        CanFrame { id, data }
    }

    pub fn data_frame(id: CanId, data: &[u8]) -> CanFrame {
        CanFrame::new_with_len(id, data, data.len())
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
    }

    pub fn remote_frame(id: CanId) -> CanFrame {
        CanFrame {
            id,
<<<<<<< HEAD
            data: [0u8; 8],
            length: 0,
=======
            data: Vec::<_, _>::new(),
>>>>>>> 74dc692b7c3a820da84d6d29c841e71d7b34ee19
        }
    }
}
