//! # Real Time Clock
//!
//! Interface to the real time clock. See STM32F303 reference manual, section 27.
//! For more details, see [ST AN4759][].
//!
//! [ST AN4759]: https:/www.st.com%2Fresource%2Fen%2Fapplication_note%2Fdm00226326-using-the-hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-stm32-microcontrollers-stmicroelectronics.pdf&usg=AOvVaw3PzvL2TfYtwS32fw-Uv37h

use crate::pac::{PWR, RTC};
use crate::rcc::{Enable, APB1, BDCR};
use core::convert::TryInto;
use core::fmt;
use rtcc::{DateTimeAccess, Datelike, Hours, NaiveDate, NaiveDateTime, NaiveTime, Rtcc, Timelike};

/// RTC error type
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Invalid input error
    InvalidInputData,
}

/// Real Time Clock peripheral
pub struct Rtc {
    /// RTC Peripheral register definition
    rtc: RTC,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Rtc {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Rtc {{ rtc: RTC }}");
    }
}

impl fmt::Debug for Rtc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Rtc").field("rtc", &"RTC").finish()
    }
}

impl Rtc {
    /// Create and enable a new RTC, and configure its clock source and prescalers.
    /// From AN4759, Table 7, when using the LSE (The only clock source this module
    /// supports currently), set `prediv_s` to 255, and `prediv_a` to 127 to get a
    /// calendar clock of 1Hz.
    /// The `bypass` argument is `true` if you're using an external oscillator that
    /// doesn't connect to `OSC32_IN`, such as a MEMS resonator.
    pub fn new(
        rtc: RTC,
        prediv_s: u16,
        prediv_a: u8,
        bypass: bool,
        apb1: &mut APB1,
        bdcr: &mut BDCR,
        pwr: &mut PWR,
    ) -> Self {
        let mut result = Self { rtc };

        enable_lse(bdcr, bypass);
        unlock(apb1, pwr);
        enable(bdcr);
        result.set_24h_fmt();

        result.rtc.prer.modify(|_, w| {
            w.prediv_s().bits(prediv_s);
            w.prediv_a().bits(prediv_a)
        });

        result
    }

    /// Sets calendar clock to 24 hr format
    pub fn set_24h_fmt(&mut self) {
        self.rtc.cr.modify(|_, w| w.fmt().set_bit());
    }
    /// Sets calendar clock to 12 hr format
    pub fn set_12h_fmt(&mut self) {
        self.rtc.cr.modify(|_, w| w.fmt().clear_bit());
    }

    /// Reads current hour format selection
    pub fn is_24h_fmt(&self) -> bool {
        self.rtc.cr.read().fmt().bit()
    }

    /// Get access to the underlying register block.
    ///
    /// # Safety
    ///
    /// This function is not _memory_ unsafe per se, but does not guarantee
    /// anything about assumptions of invariants made in this implementation.
    ///
    /// Changing specific options can lead to un-expected behavior and nothing
    /// is guaranteed.
    pub unsafe fn peripheral(&mut self) -> &mut RTC {
        &mut self.rtc
    }

    /// Release the RTC peripheral
    pub fn free(self) -> RTC {
        // TODO(Sh3Rm4n): Disable peripheral before releasing it.
        self.rtc
    }

    /// As described in Section 27.3.7 in RM0316,
    /// this function is used to disable write protection
    /// when modifying an RTC register
    fn modify<F>(&mut self, mut closure: F)
    where
        F: FnMut(&mut RTC),
    {
        // Disable write protection
        self.rtc.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.rtc.wpr.write(|w| unsafe { w.bits(0x53) });
        // Enter init mode
        let isr = self.rtc.isr.read();
        if isr.initf().bit_is_clear() {
            self.rtc.isr.modify(|_, w| w.init().set_bit());
            while self.rtc.isr.read().initf().bit_is_clear() {}
        }
        // Invoke closure
        closure(&mut self.rtc);
        // Exit init mode
        self.rtc.isr.modify(|_, w| w.init().clear_bit());
        // wait for last write to be done
        while !self.rtc.isr.read().initf().bit_is_clear() {}
    }
}

impl DateTimeAccess for Rtc {
    type Error = Error;

    fn set_datetime(&mut self, date: &NaiveDateTime) -> Result<(), Self::Error> {
        if date.year() < 1970 {
            return Err(Error::InvalidInputData);
        }

        self.set_24h_fmt();
        let (yt, yu) = bcd2_encode((date.year() - 1970) as u32)?;
        let (mt, mu) = bcd2_encode(date.month())?;
        let (dt, du) = bcd2_encode(date.day())?;

        let (ht, hu) = bcd2_encode(date.hour())?;
        let (mnt, mnu) = bcd2_encode(date.minute())?;
        let (st, su) = bcd2_encode(date.second())?;

        self.rtc.dr.write(|w| {
            w.dt().bits(dt);
            w.du().bits(du);
            w.mt().bit(mt > 0);
            w.mu().bits(mu);
            w.yt().bits(yt);
            w.yu().bits(yu)
        });

        self.rtc.tr.write(|w| {
            w.ht().bits(ht);
            w.hu().bits(hu);
            w.mnt().bits(mnt);
            w.mnu().bits(mnu);
            w.st().bits(st);
            w.su().bits(su);
            w.pm().clear_bit()
        });

        Ok(())
    }

    fn datetime(&mut self) -> Result<NaiveDateTime, Self::Error> {
        self.set_24h_fmt();

        let day = self.day()?;
        let month = self.month()?;
        let year = self.year()?;

        let seconds = self.seconds()?;
        let minutes = self.minutes()?;
        let hours = hours_to_u8(self.hours()?)?;

        Ok(
            NaiveDate::from_ymd_opt(year.into(), month.into(), day.into())
                .unwrap()
                .and_hms_opt(hours.into(), minutes.into(), seconds.into())
                .unwrap(),
        )
    }
}

impl Rtcc for Rtc {
    /// set time using NaiveTime (ISO 8601 time without timezone)
    /// Hour format is 24h
    fn set_time(&mut self, time: &NaiveTime) -> Result<(), Self::Error> {
        self.set_24h_fmt();
        let (ht, hu) = bcd2_encode(time.hour())?;
        let (mnt, mnu) = bcd2_encode(time.minute())?;
        let (st, su) = bcd2_encode(time.second())?;
        self.rtc.tr.write(|w| {
            w.ht().bits(ht);
            w.hu().bits(hu);
            w.mnt().bits(mnt);
            w.mnu().bits(mnu);
            w.st().bits(st);
            w.su().bits(su);
            w.pm().clear_bit()
        });

        Ok(())
    }

    fn set_seconds(&mut self, seconds: u8) -> Result<(), Self::Error> {
        if seconds > 59 {
            return Err(Error::InvalidInputData);
        }
        let (st, su) = bcd2_encode(seconds as u32)?;
        self.modify(|rtc| rtc.tr.modify(|_, w| w.st().bits(st).su().bits(su)));

        Ok(())
    }

    fn set_minutes(&mut self, minutes: u8) -> Result<(), Self::Error> {
        if minutes > 59 {
            return Err(Error::InvalidInputData);
        }
        let (mnt, mnu) = bcd2_encode(minutes as u32)?;
        self.modify(|rtc| rtc.tr.modify(|_, w| w.mnt().bits(mnt).mnu().bits(mnu)));

        Ok(())
    }

    fn set_hours(&mut self, hours: Hours) -> Result<(), Self::Error> {
        let (ht, hu) = hours_to_register(hours)?;
        match hours {
            Hours::H24(_h) => self.set_24h_fmt(),
            Hours::AM(_h) | Hours::PM(_h) => self.set_12h_fmt(),
        }

        self.rtc.tr.modify(|_, w| w.ht().bits(ht).hu().bits(hu));

        Ok(())
    }

    fn set_weekday(&mut self, weekday: u8) -> Result<(), Self::Error> {
        if !(1..=7).contains(&weekday) {
            return Err(Error::InvalidInputData);
        }
        self.modify(|rtc| rtc.dr.modify(|_, w| unsafe { w.wdu().bits(weekday) }));

        Ok(())
    }

    fn set_day(&mut self, day: u8) -> Result<(), Self::Error> {
        if !(1..=31).contains(&day) {
            return Err(Error::InvalidInputData);
        }
        let (dt, du) = bcd2_encode(day as u32)?;
        self.modify(|rtc| rtc.dr.modify(|_, w| w.dt().bits(dt).du().bits(du)));

        Ok(())
    }

    fn set_month(&mut self, month: u8) -> Result<(), Self::Error> {
        if !(1..=12).contains(&month) {
            return Err(Error::InvalidInputData);
        }
        let (mt, mu) = bcd2_encode(month as u32)?;
        self.modify(|rtc| rtc.dr.modify(|_, w| w.mt().bit(mt > 0).mu().bits(mu)));

        Ok(())
    }

    fn set_year(&mut self, year: u16) -> Result<(), Self::Error> {
        if !(1970..=2038).contains(&year) {
            return Err(Error::InvalidInputData);
        }
        let (yt, yu) = bcd2_encode(year as u32)?;
        self.modify(|rtc| rtc.dr.modify(|_, w| w.yt().bits(yt).yu().bits(yu)));

        Ok(())
    }

    /// Set the date using NaiveDate (ISO 8601 calendar date without timezone).
    /// WeekDay is set using the `set_weekday` method
    fn set_date(&mut self, date: &NaiveDate) -> Result<(), Self::Error> {
        if date.year() < 1970 {
            return Err(Error::InvalidInputData);
        }

        let (yt, yu) = bcd2_encode((date.year() - 1970) as u32)?;
        let (mt, mu) = bcd2_encode(date.month())?;
        let (dt, du) = bcd2_encode(date.day())?;

        self.rtc.dr.write(|w| {
            w.dt().bits(dt);
            w.du().bits(du);
            w.mt().bit(mt > 0);
            w.mu().bits(mu);
            w.yt().bits(yt);
            w.yu().bits(yu)
        });

        Ok(())
    }

    fn seconds(&mut self) -> Result<u8, Self::Error> {
        let tr = self.rtc.tr.read();
        let seconds = bcd2_decode(tr.st().bits(), tr.su().bits());
        Ok(seconds as u8)
    }

    fn minutes(&mut self) -> Result<u8, Self::Error> {
        let tr = self.rtc.tr.read();
        let minutes = bcd2_decode(tr.mnt().bits(), tr.mnu().bits());
        Ok(minutes as u8)
    }

    fn hours(&mut self) -> Result<Hours, Self::Error> {
        let tr = self.rtc.tr.read();
        let hours = bcd2_decode(tr.ht().bits(), tr.hu().bits());
        if self.is_24h_fmt() {
            return Ok(Hours::H24(hours as u8));
        }
        if !tr.pm().bit() {
            return Ok(Hours::AM(hours as u8));
        }
        Ok(Hours::PM(hours as u8))
    }

    fn time(&mut self) -> Result<NaiveTime, Self::Error> {
        self.set_24h_fmt();
        let seconds = self.seconds()?;
        let minutes = self.minutes()?;
        let hours = hours_to_u8(self.hours()?)?;

        Ok(NaiveTime::from_hms_opt(hours.into(), minutes.into(), seconds.into()).unwrap())
    }

    fn weekday(&mut self) -> Result<u8, Self::Error> {
        let dr = self.rtc.dr.read();
        let weekday = bcd2_decode(dr.wdu().bits(), 0x00);
        Ok(weekday as u8)
    }

    fn day(&mut self) -> Result<u8, Self::Error> {
        let dr = self.rtc.dr.read();
        let day = bcd2_decode(dr.dt().bits(), dr.du().bits());
        Ok(day as u8)
    }

    fn month(&mut self) -> Result<u8, Self::Error> {
        let dr = self.rtc.dr.read();
        let mt = u8::from(dr.mt().bit());
        let month = bcd2_decode(mt, dr.mu().bits());
        Ok(month as u8)
    }

    fn year(&mut self) -> Result<u16, Self::Error> {
        let dr = self.rtc.dr.read();
        let year = bcd2_decode(dr.yt().bits(), dr.yu().bits());
        Ok(year as u16)
    }

    fn date(&mut self) -> Result<NaiveDate, Self::Error> {
        let day = self.day()?;
        let month = self.month()?;
        let year = self.year()?;

        Ok(NaiveDate::from_ymd_opt(year.into(), month.into(), day.into()).unwrap())
    }
}

// Two 32-bit registers (RTC_TR and RTC_DR) contain the seconds, minutes, hours (12- or 24-hour format), day (day
// of week), date (day of month), month, and year, expressed in binary coded decimal format
// (BCD). The sub-seconds value is also available in binary format.
//
// The following helper functions encode into BCD format from integer and
// decode to an integer from a BCD value respectively.
fn bcd2_encode(word: u32) -> Result<(u8, u8), Error> {
    let l = match (word / 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidInputData);
        }
    };
    let r = match (word % 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidInputData);
        }
    };

    Ok((l, r))
}

fn bcd2_decode(fst: u8, snd: u8) -> u32 {
    (fst * 10 + snd).into()
}

fn hours_to_register(hours: Hours) -> Result<(u8, u8), Error> {
    match hours {
        Hours::H24(h) => Ok(bcd2_encode(h as u32))?,
        Hours::AM(h) => Ok(bcd2_encode((h - 1) as u32))?,
        Hours::PM(h) => Ok(bcd2_encode((h + 11) as u32))?,
    }
}

fn hours_to_u8(hours: Hours) -> Result<u8, Error> {
    if let Hours::H24(h) = hours {
        Ok(h)
    } else {
        Err(Error::InvalidInputData)
    }
}

/// Enable the low frequency external oscillator. This is the only mode currently
/// supported, to avoid exposing the `CR` and `CRS` registers.
fn enable_lse(bdcr: &mut BDCR, bypass: bool) {
    bdcr.bdcr()
        .modify(|_, w| w.lseon().set_bit().lsebyp().bit(bypass));
    while bdcr.bdcr().read().lserdy().bit_is_clear() {}
}

fn unlock(apb1: &mut APB1, pwr: &mut PWR) {
    // Enable the backup interface by setting PWREN
    PWR::enable(apb1);
    pwr.cr.modify(|_, w| {
        w
            // Enable access to the backup registers
            .dbp()
            .set_bit()
    });

    while pwr.cr.read().dbp().bit_is_clear() {}
}

fn enable(bdcr: &mut BDCR) {
    bdcr.bdcr().modify(|_, w| w.bdrst().enabled());
    bdcr.bdcr().modify(|_, w| {
        w.rtcsel().lse();
        w.rtcen().enabled();
        w.bdrst().disabled()
    });
}
