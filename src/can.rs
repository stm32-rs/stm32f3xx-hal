use crate::gpio::gpioa;
use crate::gpio::AF9;
use crate::rcc::APB1;
use crate::stm32;
use nb;
use void::Void;

const EXID_MASK: u32 = 0b11111111111000000000000000000;

#[derive(Debug, Copy, Clone)]
pub enum CanId {
    BaseId(u16),
    ExtendedId(u32),
}

#[derive(Debug, Clone)]
pub struct CanFrame<'a> {
    pub id: CanId,
    pub data: Option<&'a [u8]>,
}

pub enum CanFifo {
    Fifo0,
    Fifo1,
}

pub enum FilterMode {
    Mask,
    List,
}

pub enum FilterScale {
    Standard,
    Extended,
}

pub trait CanFilter {
    fn mode(&self) -> FilterMode;
    fn scale(&self) -> FilterScale;
    fn fr1(&self) -> u32;
    fn fr2(&self) -> u32;
}

pub struct CanExtendedMaskFilter {
    pub filter: u32,
    pub mask: u32,
}

pub struct Can {
    can: stm32::CAN,
    rx: gpioa::PA11<AF9>,
    tx: gpioa::PA12<AF9>,
}

pub enum Event {
    Fifo0Fmp,
    Fifo1Fmp,
    Fifo0Full,
    Fifo1Full,
    Fifo0Ovr,
    Fifo1Ovr,
    Txe,
}

impl Can {
    pub fn new(
        can: stm32::CAN,
        rx: gpioa::PA11<AF9>,
        tx: gpioa::PA12<AF9>,
        apb1: &mut APB1,
    ) -> Can {
        apb1.enr().modify(|_, w| w.canen().set_bit());
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

        Can { can, rx, tx }
    }

    pub fn listen(&mut self, event: Event) {
        self.can.ier.modify(|_, w| match event {
            Event::Fifo0Fmp => w.fmpie0().set_bit(),
            Event::Fifo0Full => w.ffie0().set_bit(),
            Event::Fifo0Ovr => w.fovie0().set_bit(),
            Event::Txe => w.tmeie().set_bit(),
            _ => unimplemented!(),
        });
    }

    pub fn transmit_frame(&mut self, frame: CanFrame) -> nb::Result<(), Void> {
        if let Some(data) = frame.data {
            assert!(
                data.len() < 8,
                "CanFrame cannot contain more than 8 bytes of data"
            );
        }

        for i in 0..3 {
            let free = match i {
                0 => self.can.tsr.read().tme0().bit_is_set(),
                1 => self.can.tsr.read().tme1().bit_is_set(),
                2 => self.can.tsr.read().tme2().bit_is_set(),
                _ => unreachable!(),
            };

            if !free {
                continue;
            }

            match frame.id {
                CanId::BaseId(id) => self.can.tx[i]
                    .tir
                    .modify(|_, w| unsafe { w.stid().bits(id) }),
                CanId::ExtendedId(id) => self.can.tx[i].tir.modify(|_, w| unsafe {
                    let hb = ((id & EXID_MASK) >> 18) as u16;
                    w.exid().bits(id).stid().bits(hb).ide().set_bit()
                }),
            }

            match frame.data {
                Some(d) => unsafe {
                    d.iter().enumerate().for_each(|(j, val)| match j {
                        0 => self.can.tx[i].tdlr.modify(|_, w| w.data0().bits(*val)),
                        1 => self.can.tx[i].tdlr.modify(|_, w| w.data1().bits(*val)),
                        2 => self.can.tx[i].tdlr.modify(|_, w| w.data2().bits(*val)),
                        3 => self.can.tx[i].tdlr.modify(|_, w| w.data3().bits(*val)),
                        4 => self.can.tx[i].tdhr.modify(|_, w| w.data4().bits(*val)),
                        5 => self.can.tx[i].tdhr.modify(|_, w| w.data5().bits(*val)),
                        6 => self.can.tx[i].tdhr.modify(|_, w| w.data6().bits(*val)),
                        7 => self.can.tx[i].tdhr.modify(|_, w| w.data7().bits(*val)),
                        _ => unreachable!(),
                    });

                    self.can.tx[i]
                        .tdtr
                        .modify(|_, w| w.dlc().bits(d.len() as u8));

                    self.can.tx[i].tir.modify(|_, w| w.rtr().clear_bit());
                },
                None => self.can.tx[i].tir.modify(|_, w| w.rtr().set_bit()),
            }

            // Request the frame be sent!
            self.can.tx[i].tir.modify(|_, w| w.txrq().set_bit());

            // TODO: we can prob not wait for txok?
            while match i {
                0 => self.can.tsr.read().txok0().bit_is_clear(),
                1 => self.can.tsr.read().txok1().bit_is_clear(),
                2 => self.can.tsr.read().txok2().bit_is_clear(),
                _ => unreachable!(),
            } {}

            return Ok(());
        }

        Err(nb::Error::WouldBlock)
    }

    // TODO: Enable extended filters
    pub fn enabled_filter<F: CanFilter>(&mut self, fifo: CanFifo, filter: F, _index: u8) {
        // TODO: support more than the 0th filter
        // Filter init mode
        self.can.fmr.modify(|_, w| w.finit().set_bit());

        self.can.fm1r.modify(|_, w| match filter.mode() {
            FilterMode::Mask => w.fbm0().clear_bit(),
            FilterMode::List => w.fbm0().set_bit(),
        });

        self.can.fs1r.modify(|_, w| match filter.scale() {
            FilterScale::Extended => w.fsc0().set_bit(),
            FilterScale::Standard => w.fsc0().clear_bit(),
        });

        // Assign filter to selected FIFO
        self.can.ffa1r.modify(|_, w| match fifo {
            CanFifo::Fifo0 => w.ffa0().clear_bit(),
            CanFifo::Fifo1 => w.ffa0().set_bit(),
        });

        self.can.fb[0]
            .fr1
            .modify(|_, w| unsafe { w.bits(filter.fr1()) });

        self.can.fb[0]
            .fr2
            .modify(|_, w| unsafe { w.bits(filter.fr2()) });

        self.can.fa1r.modify(|_, w| w.fact0().set_bit());

        // Disable init mode
        self.can.fmr.modify(|_, w| w.finit().clear_bit());
    }

    pub fn read_into<'a>(
        &mut self,
        buffer: &'a mut [u8; 8],
    ) -> nb::Result<Option<(CanId, &'a [u8])>, Void> {
        if self.can.rfr[0].read().fmp().bits() > 0 {
            let len = self.can.rx[0].rdtr.read().dlc().bits() as usize;
            for i in 0..len {
                match i {
                    0 => buffer[i] = self.can.rx[0].rdlr.read().data0().bits(),
                    1 => buffer[i] = self.can.rx[0].rdlr.read().data1().bits(),
                    2 => buffer[i] = self.can.rx[0].rdlr.read().data2().bits(),
                    3 => buffer[i] = self.can.rx[0].rdlr.read().data3().bits(),
                    4 => buffer[i] = self.can.rx[0].rdhr.read().data4().bits(),
                    5 => buffer[i] = self.can.rx[0].rdhr.read().data5().bits(),
                    6 => buffer[i] = self.can.rx[0].rdhr.read().data6().bits(),
                    7 => buffer[i] = self.can.rx[0].rdhr.read().data7().bits(),
                    _ => unreachable!(),
                }
            }

            let rcv_id = match self.can.rx[0].rir.read().ide().bit_is_set() {
                true => {
                    let lower = self.can.rx[0].rir.read().exid().bits();
                    let upper = (self.can.rx[0].rir.read().stid().bits() as u32) << 18;
                    CanId::ExtendedId(upper | lower)
                }
                false => CanId::BaseId(self.can.rx[0].rir.read().stid().bits()),
            };

            // Release the mailbox
            self.can.rfr[0].modify(|_, w| w.rfom().set_bit());

            return Ok(Some((rcv_id, &buffer[..len])));
        }

        Ok(None)
    }

    pub fn release(self) -> stm32::CAN {
        self.can
    }
}

impl CanFilter for CanExtendedMaskFilter {
    fn mode(&self) -> FilterMode {
        FilterMode::Mask
    }

    fn scale(&self) -> FilterScale {
        FilterScale::Extended
    }

    fn fr1(&self) -> u32 {
        self.filter << 3
    }

    fn fr2(&self) -> u32 {
        self.mask << 3
    }
}

impl<'a> CanFrame<'a> {
    pub fn data_frame<'b>(id: CanId, data: &'b [u8]) -> CanFrame<'b> {
        CanFrame {
            id,
            data: Some(data),
        }
    }

    pub fn remote_frame(id: CanId) -> CanFrame<'static> {
        CanFrame { id, data: None }
    }
}
