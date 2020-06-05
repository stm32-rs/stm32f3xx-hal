/*!  Interface to the real time clock */

use crate::rcc::{APB1, BDCR, CR, CSR};
use crate::stm32::{PWR, RTC};
use rtcc::{Datelike, Hours, NaiveDate, NaiveDateTime, NaiveTime, Rtcc, Timelike};

/// Invalid input error
// #[derive(Clone, Copy)]
#[derive(Debug)]
pub enum Error {
    InvalidInputData,
}

/// RTC clock input source
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum RTCSrc {
    LSE = 0b01,
    LSI = 0b10,
    HSE = 0b11,
}

/// RTC configuration struct
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct cfgRTC {
    src: RTCSrc,
    prediv_s: u16,
    prediv_a: u8,
}

/// LSI clock source with prescalers
/// set clock frequency to 1 hz
impl Default for cfgRTC {
    fn default() -> Self {
        cfgRTC {
            src: RTCSrc::LSI,
            prediv_s: 311,
            prediv_a: 127,
        }
    }
}

impl cfgRTC {
    pub fn new(source: RTCSrc, prescaler_sync: u16, prescaler_async: u8) -> cfgRTC {
        let config = cfgRTC {
            src: source,
            prediv_s: prescaler_sync,
            prediv_a: prescaler_async,
        };
        config
    }
}

pub struct Rtc {
    regs: RTC,
}

impl Rtc {
    pub fn rtc(
        regs: RTC,
        apb1: &mut APB1,
        bdcr: &mut BDCR,
        cr: &mut CR,
        csr: &mut CSR,
        cfg_rtc: cfgRTC,
    ) -> Self {
        match &cfg_rtc.src {
            RTCSrc::LSI => Rtc::enable_lsi(csr),
            RTCSrc::HSE => Rtc::enable_hse(cr, false),
            RTCSrc::LSE => Rtc::enable_lse(bdcr, false),
        }
        Rtc::unlock_rtc(apb1);
        Rtc::enable_rtc(bdcr, &cfg_rtc);

        let mut result = Self { regs };
        result.set_12h_fmt();
        result.modify(|regs| {
            regs.prer.write(|w| unsafe {
                w.prediv_s()
                    .bits(cfg_rtc.prediv_s)
                    .prediv_a()
                    .bits(cfg_rtc.prediv_a)
            });
        });
        result
    }

    fn enable_lsi(csr: &mut CSR) {
        csr.csr().write(|w| w.lsion().set_bit());
        while csr.csr().read().lsirdy().bit_is_clear() {}
    }

    fn enable_hse(cr: &mut CR, bypass: bool) {
        cr.cr().write(|w| w.hseon().set_bit().hsebyp().bit(bypass));
        while cr.cr().read().hserdy().bit_is_clear() {}
    }

    fn enable_lse(bdcr: &mut BDCR, bypass: bool) {
        bdcr.bdcr()
            .write(|w| w.lseon().set_bit().lsebyp().bit(bypass));
        while bdcr.bdcr().read().lserdy().bit_is_clear() {}
    }

    fn unlock_rtc(apb1: &mut APB1) {
        let pwr = unsafe { &(*PWR::ptr()) };
        apb1.enr().modify(|_, w| {
            w
                // Enable the backup interface by setting PWREN
                .pwren()
                .set_bit()
        });
        pwr.cr.modify(|_, w| {
            w
                // Enable access to the backup registers
                .dbp()
                .set_bit()
        });

        while pwr.cr.read().dbp().bit_is_clear() {}
    }

    fn enable_rtc(bdcr: &mut BDCR, cfg_rtc: &cfgRTC) {
        bdcr.bdcr().modify(|_, w| {
            w
                // RTC Backup Domain reset bit set high
                .bdrst()
                .set_bit()
        });

        bdcr.bdcr().modify(|_, w| {
            w
                // RTC clock source selection
                .rtcsel()
                .bits(cfg_rtc.src as u8)
                // Enable RTC
                .rtcen()
                .set_bit()
                // RTC backup Domain reset bit set low
                .bdrst()
                .clear_bit()
        });
    }

    /// Sets calendar clock to 24 hr format
    pub fn set_24h_fmt(&mut self) {
        self.regs.cr.modify(|_, w| w.fmt().set_bit());
    }
    /// Sets calendar clock to 12 hr format
    pub fn set_12h_fmt(&mut self) {
        self.regs.cr.modify(|_, w| w.fmt().clear_bit());
    }

    /// Reads current hour format selection
    pub fn is_24h_fmt(&self) -> bool {
        self.regs.cr.read().fmt().bit()
    }

    /// As described in Section 27.3.7 in RM0316,
    /// this function is used to disable write protection
    /// when modifying an RTC register
    fn modify<F>(&mut self, mut closure: F)
    where
        F: FnMut(&mut RTC) -> (),
    {
        // Disable write protection
        self.regs.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.regs.wpr.write(|w| unsafe { w.bits(0x53) });
        // Enter init mode
        let isr = self.regs.isr.read();
        if isr.initf().bit_is_clear() {
            self.regs.isr.write(|w| w.init().set_bit());
            while self.regs.isr.read().initf().bit_is_clear() {}
        }
        // Invoke closure
        closure(&mut self.regs);
        // Exit init mode
        self.regs.isr.write(|w| w.init().clear_bit());
        // wait for last write to be done
        while !self.regs.isr.read().initf().bit_is_clear() {}
    }
}

impl Rtcc for Rtc {
    type Error = Error;

    /// set time using NaiveTime (ISO 8601 time without timezone)
    /// Hour format is 24h
    fn set_time(&mut self, time: &NaiveTime) -> Result<(), Self::Error> {
        self.set_24h_fmt();
        let (ht, hu) = bcd2_encode(time.hour());
        let (mnt, mnu) = bcd2_encode(time.minute());
        let (st, su) = bcd2_encode(time.second());
        Ok(self.modify(|regs| {
            regs.tr.write(|w| unsafe {
                w.ht()
                    .bits(ht)
                    .hu()
                    .bits(hu)
                    .mnt()
                    .bits(mnt)
                    .mnu()
                    .bits(mnu)
                    .st()
                    .bits(st)
                    .su()
                    .bits(su)
                    .pm()
                    .clear_bit()
            });
        }))
    }

    fn set_seconds(&mut self, seconds: u8) -> Result<(), Self::Error> {
        if seconds > 59 {
            return Err(Error::InvalidInputData);
        }
        let (st, su) = bcd2_encode(seconds as u32);
        Ok(self.modify(|regs| regs.tr.write(|w| unsafe { w.st().bits(st).su().bits(su) })))
    }

    fn set_minutes(&mut self, minutes: u8) -> Result<(), Self::Error> {
        if minutes > 59 {
            return Err(Error::InvalidInputData);
        }
        let (mnt, mnu) = bcd2_encode(minutes as u32);
        Ok(self.modify(|regs| {
            regs.tr
                .write(|w| unsafe { w.mnt().bits(mnt).mnu().bits(mnu) })
        }))
    }

    fn set_hours(&mut self, hours: rtcc::Hours) -> Result<(), Self::Error> {
        let (ht, hu) = hours_to_register(hours)?;
        match hours {
            Hours::H24(_h) => {
                self.set_24h_fmt();
                Ok(self.modify(|regs| regs.tr.write(|w| unsafe { w.ht().bits(ht).hu().bits(hu) })))
            }
            Hours::AM(_h) | Hours::PM(_h) => {
                self.set_12h_fmt();
                Ok(self.modify(|regs| regs.tr.write(|w| unsafe { w.ht().bits(ht).hu().bits(hu) })))
            }
        }
    }

    fn set_weekday(&mut self, weekday: u8) -> Result<(), Self::Error> {
        if (weekday < 1) | (weekday > 7) {
            return Err(Error::InvalidInputData);
        }
        Ok(self.modify(|regs| regs.dr.write(|w| unsafe { w.wdu().bits(weekday) })))
    }

    fn set_day(&mut self, day: u8) -> Result<(), Self::Error> {
        if (day < 1) | (day > 31) {
            return Err(Error::InvalidInputData);
        }
        let (dt, du) = bcd2_encode(day as u32);
        Ok(self.modify(|regs| regs.dr.write(|w| unsafe { w.dt().bits(dt).du().bits(du) })))
    }

    fn set_month(&mut self, month: u8) -> Result<(), Self::Error> {
        if (month < 1) | (month > 12) {
            return Err(Error::InvalidInputData);
        }
        let (mt, mu) = bcd2_encode(month as u32);
        Ok(self.modify(|regs| {
            regs.dr
                .write(|w| unsafe { w.mt().bit(mt > 0).mu().bits(mu) })
        }))
    }

    fn set_year(&mut self, year: u16) -> Result<(), Self::Error> {
        if (year < 1970) | (year > 2038) {
            return Err(Error::InvalidInputData);
        }
        let (yt, yu) = bcd2_encode(year as u32);
        Ok(self.modify(|regs| regs.dr.write(|w| unsafe { w.yt().bits(yt).yu().bits(yu) })))
    }

    /// set date using NaiveDate (ISO 8601 calendar date without timezone)
    /// WeekDay is set using set_weekday method
    fn set_date(&mut self, date: &NaiveDate) -> Result<(), Self::Error> {
        let (yt, yu) = bcd2_encode((date.year() - 1970) as u32);
        let (mt, mu) = bcd2_encode(date.month());
        let (dt, du) = bcd2_encode(date.day());

        Ok(self.modify(|regs| {
            regs.dr.write(|w| unsafe {
                w.dt()
                    .bits(dt)
                    .du()
                    .bits(du)
                    .mt()
                    .bit(mt > 0)
                    .mu()
                    .bits(mu)
                    .yt()
                    .bits(yt)
                    .yu()
                    .bits(yu)
            });
        }))
    }

    fn set_datetime(&mut self, date: &NaiveDateTime) -> Result<(), Self::Error> {
        // Check if unsigned integer affects encoding to bcd

        self.set_24h_fmt();
        let (yt, yu) = bcd2_encode((date.year() - 1970) as u32);
        let (mt, mu) = bcd2_encode(date.month());
        let (dt, du) = bcd2_encode(date.day());

        let (ht, hu) = bcd2_encode(date.hour());
        let (mnt, mnu) = bcd2_encode(date.minute());
        let (st, su) = bcd2_encode(date.second());

        self.modify(|regs| {
            regs.dr.write(|w| unsafe {
                w.dt()
                    .bits(dt)
                    .du()
                    .bits(du)
                    .mt()
                    .bit(mt > 0)
                    .mu()
                    .bits(mu)
                    .yt()
                    .bits(yt)
                    .yu()
                    .bits(yu)
            });
        });
        Ok(self.modify(|regs| {
            regs.tr.write(|w| unsafe {
                w.ht()
                    .bits(ht)
                    .hu()
                    .bits(hu)
                    .mnt()
                    .bits(mnt)
                    .mnu()
                    .bits(mnu)
                    .st()
                    .bits(st)
                    .su()
                    .bits(su)
                    .pm()
                    .clear_bit()
            });
        }))
    }

    fn get_seconds(&mut self) -> Result<u8, Self::Error> {
        let tr = self.regs.tr.read();
        let seconds = bcd2_decode(tr.st().bits(), tr.su().bits());
        Ok(seconds as u8)
    }

    fn get_minutes(&mut self) -> Result<u8, Self::Error> {
        let tr = self.regs.tr.read();
        let minutes = bcd2_decode(tr.mnt().bits(), tr.mnu().bits());
        Ok(minutes as u8)
    }

    fn get_hours(&mut self) -> Result<rtcc::Hours, Self::Error> {
        let tr = self.regs.tr.read();
        let hours = bcd2_decode(tr.ht().bits(), tr.hu().bits());
        if self.is_24h_fmt() {
            return Ok(rtcc::Hours::H24(hours as u8));
        }
        if !tr.pm().bit() {
            return Ok(rtcc::Hours::AM(hours as u8));
        }
        Ok(rtcc::Hours::PM(hours as u8))
    }

    fn get_time(&mut self) -> Result<NaiveTime, Self::Error> {
        self.set_24h_fmt();
        let seconds = self.get_seconds().unwrap();
        let minutes = self.get_minutes().unwrap();
        let hours = hours_to_u8(self.get_hours().unwrap());

        Ok(NaiveTime::from_hms(
            hours.into(),
            minutes.into(),
            seconds.into(),
        ))
    }

    fn get_weekday(&mut self) -> Result<u8, Self::Error> {
        let dr = self.regs.dr.read();
        let weekday = bcd2_decode(dr.wdu().bits(), 0x00);
        Ok(weekday as u8)
    }

    fn get_day(&mut self) -> Result<u8, Self::Error> {
        let dr = self.regs.dr.read();
        let day = bcd2_decode(dr.dt().bits(), dr.du().bits());
        Ok(day as u8)
    }

    fn get_month(&mut self) -> Result<u8, Self::Error> {
        let dr = self.regs.dr.read();
        let mt: u8 = if dr.mt().bit() { 1 } else { 0 };
        let month = bcd2_decode(mt, dr.mu().bits());
        Ok(month as u8)
    }

    fn get_year(&mut self) -> Result<u16, Self::Error> {
        let dr = self.regs.dr.read();
        let year = bcd2_decode(dr.yt().bits(), dr.yu().bits());
        Ok(year as u16)
    }

    fn get_date(&mut self) -> Result<NaiveDate, Self::Error> {
        let day = self.get_day().unwrap();
        let month = self.get_month().unwrap();
        let year = self.get_year().unwrap();

        Ok(NaiveDate::from_ymd(year.into(), month.into(), day.into()))
    }

    fn get_datetime(&mut self) -> Result<NaiveDateTime, Self::Error> {
        self.set_24h_fmt();

        let day = self.get_day().unwrap();
        let month = self.get_month().unwrap();
        let year = self.get_year().unwrap();

        let seconds = self.get_seconds().unwrap();
        let minutes = self.get_minutes().unwrap();
        let hours = hours_to_u8(self.get_hours().unwrap());

        Ok(
            NaiveDate::from_ymd(year.into(), month.into(), day.into()).and_hms(
                hours.into(),
                minutes.into(),
                seconds.into(),
            ),
        )
    }
}

// Two 32-bit registers (RTC_TR and RTC_DR) contain the seconds, minutes, hours (12- or 24-hour format), day (day
// of week), date (day of month), month, and year, expressed in binary coded decimal format
// (BCD). The sub-seconds value is also available in binary format.
//
// The following helper functions are encode into BCD format from integer and
// decode to an integer from a BCD value respectively.
fn bcd2_encode(word: u32) -> (u8, u8) {
    let mut value = word as u8;
    let mut bcd_high: u8 = 0;
    while value >= 10 {
        bcd_high += 1;
        value -= 10;
    }
    // let bcd_low = ((bcd_high << 4) | value) as u8;
    let bcd_low = value as u8;
    (bcd_high, bcd_low)
}

fn bcd2_decode(fst: u8, snd: u8) -> u32 {
    let value = snd | fst << 4;
    let value = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;
    value as u32
}

fn hours_to_register(hours: Hours) -> Result<(u8, u8), Error> {
    match hours {
        Hours::H24(h) if h > 23 => Err(Error::InvalidInputData),
        Hours::H24(h) => Ok(bcd2_encode(h as u32)),
        Hours::AM(h) if h < 1 || h > 12 => Err(Error::InvalidInputData),
        Hours::AM(h) => Ok(bcd2_encode(h as u32)),
        Hours::PM(h) if h < 1 || h > 12 => Err(Error::InvalidInputData),
        Hours::PM(h) => Ok(bcd2_encode(h as u32)),
    }
}

fn hours_to_u8(_hours: rtcc::Hours) -> u8 {
    if let rtcc::Hours::H24(h) = _hours {
        h
    } else {
        panic!("_hours could not be destructured into rtc::Hours::H24(h)");
    }
}
