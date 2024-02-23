//! # Real Time Clock
//!
//! Interface to the real time clock. See STM32F303 reference manual, section 27.
//! For more details, see [ST AN4759][].
//!
//! [ST AN4759]: https:/www.st.com%2Fresource%2Fen%2Fapplication_note%2Fdm00226326-using-the-hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-stm32-microcontrollers-stmicroelectronics.pdf&usg=AOvVaw3PzvL2TfYtwS32fw-Uv37h

use crate::pac::{PWR, RTC, RCC};
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
    /// Invalid register data, failed to convert to rtcc Type
    InvalidRtcData,
}

/// RTC clock source
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RtcClockSource {
    /// Low Speed Internal Clock (LSI)
    LSI,
    /// Low Speed External Clock (LSE) - To turn off/on bypass for LSE, please use inner bool parameter LSE(false/true)
    LSE(bool),
}

pub struct RtcBuilder {
    pub(crate) rtc: RTC,
    prediv_s: u16,
    prediv_a: u8,
    clock_source: RtcClockSource,
    default: bool,
}

impl RtcBuilder {
    /// Builder for RTC using both LSI and LSE clock sources,
    /// by default LSE is used with configuration of 1Hz calendar clock.
    ///
    /// ### Example for LSE:
    /// ```
    /// use stm32f3xx_hal::{pac, rtc::{RtcBuilder, RtcClockSource}};
    /// ...
    /// let dp = pac::Peripherals::take().unwrap();
    /// let rcc = pac.RCC.constrain();
    /// let mut pwr = dp.PWR;
    ///
    /// let rtc = RtcBuilder::new(pac.RTC).build(&mut pwr, &mut rcc.apb1, &mut rcc.bdcr);
    /// /// or
    /// let rtc = RtcBuilder::new(pac.RTC)
    /// .set_clock_source(RtcClockSource::LSE(true)).build(&mut pwr, &mut rcc.apb1, &mut rcc.bdcr);
    /// ```
    /// ### Example for LSI:
    /// ````
    /// let rtc = RtcBuilder::new(pac.RTC)
    /// .set_clock_source(RtcClockSource::LSI).build(&mut pwr, &mut rcc.apb1, &mut rcc.bdcr);
    /// ````
    /// **This examples shows how to run RTC with LSI and LSE with 1Hz frequency. This means
    /// that RTC will increase by 1 second every second.**
    ///
    /// If you want to change your clock source or prescalers please use
    /// correct functions.
    pub fn new(rtc: RTC) -> Self {
        Self {
            prediv_s: 255,
            prediv_a: 127,
            clock_source: RtcClockSource::LSE(false),
            default: true,
            rtc: rtc,
        }
    }

    /// Set your prescaler "perediv_s"
    ///
    /// **Note:** Using deferent values than default will affect your RTC clock,
    /// it can slow down or speed up RTC.
    pub fn set_perediv_s(mut self, prediv_s: u16) -> Self {
        self.default = false;
        self.prediv_s = prediv_s;
        self
    }

    /// Set your prescaler "perediv_a"
    ///
    /// **Note:** Using deferent values than default will affect your RTC clock,
    /// it can slow down or speed up RTC.
    pub fn set_perediv_a(mut self, prediv_a: u8) -> Self {
        self.default = false;
        self.prediv_a = prediv_a;
        self
    }

    /// Please select your clock source. Selecting source of RTC clock will impact on
    /// clock accuracy.
    ///
    /// If your using:
    ///
    /// - ***LSE*** - your clock will be accurate, but it is external peripheral, so you might not have it.
    ///
    /// - ***LSI*** - your clock might be not accurate, but if you don't need super accurate clock you can use LSI.
    /// It is build in microcontroller clock source.
    pub fn set_clock_source(mut self, clock_source: RtcClockSource) -> Self {
        self.clock_source = clock_source;
        self
    }

    /// Build RTC ready for use
    pub fn build(mut self, pwr: &mut PWR, apb1: &mut APB1, bdcr: &mut BDCR) -> Rtc {
        match self.clock_source {
            RtcClockSource::LSI => {
                let cfg = match self.default {
                    true => {
                        Self {
                            prediv_s: 319,
                            prediv_a: 127,
                            clock_source: RtcClockSource::LSI,
                            default: true,
                            rtc: self.rtc,
                        }
                    }
                    false => {
                        self
                    }
                };
                Rtc::new_for_builder(cfg, apb1, bdcr, pwr)
            }
            RtcClockSource::LSE(bypass) => {
                Rtc::new_for_builder(self, apb1, bdcr, pwr)
            }
        }
    }
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

    /// Constructor for RTC builder
    pub(crate) fn new_for_builder(cfg: RtcBuilder, apb1: &mut APB1, bdcr: &mut BDCR, pwr: &mut PWR) -> Self {
        let mut result = Self { rtc: cfg.rtc };

        unlock(apb1, pwr);
        match cfg.clock_source {
            RtcClockSource::LSI => {
                enable_lsi();
                enable_rtc_with_lsi(bdcr);
            }
            RtcClockSource::LSE(bypass) => {
                enable_lse(bdcr, bypass);
                enable(bdcr);
            }
        }
        result.set_24h_fmt();

        result.rtc.prer.modify(|_, w| {
            w.prediv_s().bits(cfg.prediv_s);
            w.prediv_a().bits(cfg.prediv_a)
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
    #[must_use]
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
    #[must_use]
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
        self.rtc.wpr.write(|w| w.key().bits(0xCA));
        self.rtc.wpr.write(|w| w.key().bits(0x53));
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

// TODO: check conditional compilation because of Chrono
impl DateTimeAccess for Rtc {
    type Error = Error;

    fn set_datetime(&mut self, date: &NaiveDateTime) -> Result<(), Self::Error> {
        self.set_24h_fmt();
        let (year_tens, year_units) =
            bcd2_encode(u32::try_from(date.year() - 1970).map_err(|_| Error::InvalidInputData)?)?;
        let (month_tens, month_units) = bcd2_encode(date.month())?;
        let (day_tens, day_units) = bcd2_encode(date.day())?;

        let (hour_tens, hour_units) = bcd2_encode(date.hour())?;
        let (minutes_tens, minutes_units) = bcd2_encode(date.minute())?;
        let (second_tens, second_units) = bcd2_encode(date.second())?;

        self.rtc.dr.write(|w| {
            w.dt().bits(day_tens);
            w.du().bits(day_units);
            w.mt().bit(month_tens > 0);
            w.mu().bits(month_units);
            w.yt().bits(year_tens);
            w.yu().bits(year_units)
        });

        self.rtc.tr.write(|w| {
            w.ht().bits(hour_tens);
            w.hu().bits(hour_units);
            w.mnt().bits(minutes_tens);
            w.mnu().bits(minutes_units);
            w.st().bits(second_tens);
            w.su().bits(second_units);
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

        NaiveDate::from_ymd_opt(year.into(), month.into(), day.into())
            .ok_or(Error::InvalidRtcData)?
            .and_hms_opt(hours.into(), minutes.into(), seconds.into())
            .ok_or(Error::InvalidRtcData)
    }
}

impl Rtcc for Rtc {
    /// Set time using `NaiveTime` (ISO 8601 time without timezone)
    ///
    /// Hour format is 24h
    fn set_time(&mut self, time: &NaiveTime) -> Result<(), Self::Error> {
        self.set_24h_fmt();
        let (hour_tens, hour_units) = bcd2_encode(time.hour())?;
        let (minutes_tens, minutes_units) = bcd2_encode(time.minute())?;
        let (seconds_tens, seconds_units) = bcd2_encode(time.second())?;
        self.rtc.tr.write(|w| {
            w.ht().bits(hour_tens);
            w.hu().bits(hour_units);
            w.mnt().bits(minutes_tens);
            w.mnu().bits(minutes_units);
            w.st().bits(seconds_tens);
            w.su().bits(seconds_units);
            w.pm().clear_bit()
        });

        Ok(())
    }

    fn set_seconds(&mut self, seconds: u8) -> Result<(), Self::Error> {
        if seconds > 59 {
            return Err(Error::InvalidInputData);
        }
        let (seconds_tens, seconds_units) = bcd2_encode(u32::from(seconds))?;
        self.modify(|rtc| {
            rtc.tr
                .modify(|_, w| w.st().bits(seconds_tens).su().bits(seconds_units));
        });

        Ok(())
    }

    fn set_minutes(&mut self, minutes: u8) -> Result<(), Self::Error> {
        if minutes > 59 {
            return Err(Error::InvalidInputData);
        }
        let (minutes_tens, minutes_units) = bcd2_encode(u32::from(minutes))?;
        self.modify(|rtc| {
            rtc.tr
                .modify(|_, w| w.mnt().bits(minutes_tens).mnu().bits(minutes_units));
        });

        Ok(())
    }

    fn set_hours(&mut self, hours: Hours) -> Result<(), Self::Error> {
        let (hour_tens, hour_units) = hours_to_register(hours)?;
        match hours {
            Hours::H24(_) => self.set_24h_fmt(),
            Hours::AM(_) | Hours::PM(_) => self.set_12h_fmt(),
        }

        self.rtc
            .tr
            .modify(|_, w| w.ht().bits(hour_tens).hu().bits(hour_units));

        Ok(())
    }

    fn set_weekday(&mut self, weekday: u8) -> Result<(), Self::Error> {
        if !(1..=7).contains(&weekday) {
            return Err(Error::InvalidInputData);
        }
        // SAFETY: check above ensures, that the weekday number is in the valid range (0x01 - 0x07)
        self.modify(|rtc| rtc.dr.modify(|_, w| unsafe { w.wdu().bits(weekday) }));

        Ok(())
    }

    fn set_day(&mut self, day: u8) -> Result<(), Self::Error> {
        if !(1..=31).contains(&day) {
            return Err(Error::InvalidInputData);
        }
        let (day_tens, day_units) = bcd2_encode(u32::from(day))?;
        self.modify(|rtc| {
            rtc.dr
                .modify(|_, w| w.dt().bits(day_tens).du().bits(day_units));
        });

        Ok(())
    }

    fn set_month(&mut self, month: u8) -> Result<(), Self::Error> {
        if !(1..=12).contains(&month) {
            return Err(Error::InvalidInputData);
        }
        let (month_tens, month_units) = bcd2_encode(u32::from(month))?;
        self.modify(|rtc| {
            rtc.dr
                .modify(|_, w| w.mt().bit(month_tens > 0).mu().bits(month_units));
        });

        Ok(())
    }

    fn set_year(&mut self, year: u16) -> Result<(), Self::Error> {
        if !(1970..=2038).contains(&year) {
            return Err(Error::InvalidInputData);
        }
        let (year_tens, yu) = bcd2_encode(u32::from(year))?;
        self.modify(|rtc| rtc.dr.modify(|_, w| w.yt().bits(year_tens).yu().bits(yu)));

        Ok(())
    }

    /// Set the date using `NaiveDate` (ISO 8601 calendar date without timezone).
    /// `WeekDay` is set using the `set_weekday` method
    fn set_date(&mut self, date: &NaiveDate) -> Result<(), Self::Error> {
        let (year_tens, yu) =
            bcd2_encode(u32::try_from(date.year() - 1970).map_err(|_| Error::InvalidInputData)?)?;
        let (month_tens, month_units) = bcd2_encode(date.month())?;
        let (day_tens, day_units) = bcd2_encode(date.day())?;

        self.rtc.dr.write(|w| {
            w.dt().bits(day_tens);
            w.du().bits(day_units);
            w.mt().bit(month_tens > 0);
            w.mu().bits(month_units);
            w.yt().bits(year_tens);
            w.yu().bits(yu)
        });

        Ok(())
    }

    fn seconds(&mut self) -> Result<u8, Self::Error> {
        let tr = self.rtc.tr.read();
        let seconds = bcd2_decode(tr.st().bits(), tr.su().bits());
        u8::try_from(seconds).map_err(|_| Error::InvalidRtcData)
    }

    fn minutes(&mut self) -> Result<u8, Self::Error> {
        let tr = self.rtc.tr.read();
        let minutes = bcd2_decode(tr.mnt().bits(), tr.mnu().bits());
        u8::try_from(minutes).map_err(|_| Error::InvalidRtcData)
    }

    fn hours(&mut self) -> Result<Hours, Self::Error> {
        let tr = self.rtc.tr.read();
        let hours = bcd2_decode(tr.ht().bits(), tr.hu().bits());
        let hours = u8::try_from(hours).map_err(|_| Error::InvalidRtcData)?;
        if self.is_24h_fmt() {
            return Ok(Hours::H24(hours));
        }
        if !tr.pm().bit() {
            return Ok(Hours::AM(hours));
        }
        Ok(Hours::PM(hours))
    }

    fn time(&mut self) -> Result<NaiveTime, Self::Error> {
        self.set_24h_fmt();
        let seconds = self.seconds()?;
        let minutes = self.minutes()?;
        let hours = hours_to_u8(self.hours()?)?;

        NaiveTime::from_hms_opt(hours.into(), minutes.into(), seconds.into())
            .ok_or(Error::InvalidRtcData)
    }

    fn weekday(&mut self) -> Result<u8, Self::Error> {
        let dr = self.rtc.dr.read();
        let weekday = bcd2_decode(dr.wdu().bits(), 0x00);
        u8::try_from(weekday).map_err(|_| Error::InvalidRtcData)
    }

    fn day(&mut self) -> Result<u8, Self::Error> {
        let dr = self.rtc.dr.read();
        let day = bcd2_decode(dr.dt().bits(), dr.du().bits());
        u8::try_from(day).map_err(|_| Error::InvalidRtcData)
    }

    fn month(&mut self) -> Result<u8, Self::Error> {
        let dr = self.rtc.dr.read();
        let month_tens = u8::from(dr.mt().bit());
        let month = bcd2_decode(month_tens, dr.mu().bits());
        u8::try_from(month).map_err(|_| Error::InvalidRtcData)
    }

    fn year(&mut self) -> Result<u16, Self::Error> {
        let dr = self.rtc.dr.read();
        let year = bcd2_decode(dr.yt().bits(), dr.yu().bits());
        u16::try_from(year).map_err(|_| Error::InvalidRtcData)
    }

    fn date(&mut self) -> Result<NaiveDate, Self::Error> {
        let day = self.day()?;
        let month = self.month()?;
        let year = self.year()?;

        NaiveDate::from_ymd_opt(year.into(), month.into(), day.into()).ok_or(Error::InvalidRtcData)
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
            return Err(Error::InvalidRtcData);
        }
    };
    let r = match (word % 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidRtcData);
        }
    };

    Ok((l, r))
}

fn bcd2_decode(fst: u8, snd: u8) -> u32 {
    u32::from(fst) * 10 + u32::from(snd)
}

fn hours_to_register(hours: Hours) -> Result<(u8, u8), Error> {
    match hours {
        Hours::H24(h) => Ok(bcd2_encode(u32::from(h)))?,
        Hours::AM(h) => Ok(bcd2_encode(u32::from(h - 1)))?,
        Hours::PM(h) => Ok(bcd2_encode(u32::from(h + 11)))?,
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

/// Enable the low frequency internal oscillator (LSI) - potentially unsafe
fn enable_lsi() {
    let mut rcc = unsafe { &*RCC::ptr() };
    rcc.csr.modify(|_, w| w.lsion().set_bit());
    while rcc.csr.read().lsirdy().bit_is_clear() {}
}

/// Enable DBP
fn unlock(apb1: &mut APB1, pwr: &mut PWR) {
    // Enable the backup interface by setting PWREN
    PWR::enable(apb1);
    pwr.cr.modify(|_, w|
        w
            // Enable access to the backup registers
            .dbp()
            .set_bit()
    );

    while pwr.cr.read().dbp().bit_is_clear() {}
}

/// Enable RTC with Low Speed Internal clock (LSI) as clock source.
fn enable_rtc_with_lsi(bdcr: &mut BDCR) {
    bdcr.bdcr().modify(|_, w| w.bdrst().enabled());
    bdcr.bdcr().modify(|_, w| {
        w.rtcsel().lsi();
        w.rtcen().enabled();
        w.bdrst().disabled()
    });
}

fn enable(bdcr: &mut BDCR) {
    bdcr.bdcr().modify(|_, w| w.bdrst().enabled());
    bdcr.bdcr().modify(|_, w| {
        w.rtcsel().lse();
        w.rtcen().enabled();
        w.bdrst().disabled()
    });
}
