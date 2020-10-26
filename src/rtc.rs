//! Interface to the real time clock. See STM32F303 reference manual, section 27.
//! For more details, see
//! [ST AN4759](https:/www.st.com%2Fresource%2Fen%2Fapplication_note%2Fdm00226326-using-the-hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-stm32-microcontrollers-stmicroelectronics.pdf&usg=AOvVaw3PzvL2TfYtwS32fw-Uv37h)
#[cfg(any(feature = "rt"))]
#[cfg(not(any(feature = "stm32f301", feature = "stm32f373", feature = "stm32f398")))]
use crate::pac::{interrupt::RTC_WKUP, EXTI};
#[cfg(any(feature = "rt"))]
#[cfg(not(any(feature = "stm32f301", feature = "stm32f373", feature = "stm32f398")))]
use cortex_m::peripheral::NVIC;

use crate::pac::{PWR, RTC};
use crate::rcc::{APB1, BDCR};
use core::convert::TryInto;
use rtcc::{Datelike, Hours, NaiveDate, NaiveDateTime, NaiveTime, Rtcc, Timelike};

#[cfg(any(feature = "rt"))]
/// To enable RTC wakeup interrupts, run this in the main body of your program, eg:
/// `make_rtc_interrupt_handler!(RTC_WKUP);`
#[macro_export]
macro_rules! make_rtc_interrupt_handler {
    ($line:ident) => {
        #[interrupt]
        fn $line() {
            free(|cs| {
                // Reset pending bit for interrupt line
                unsafe { (*pac::EXTI::ptr()).pr1.modify(|_, w| w.pr20().bit(true)) };

                // Clear the wake up timer flag, after disabling write protections.
                unsafe { (*pac::RTC::ptr()).wpr.write(|w| w.bits(0xCA)) };
                unsafe { (*pac::RTC::ptr()).wpr.write(|w| w.bits(0x53)) };

                unsafe { (*pac::RTC::ptr()).cr.modify(|_, w| w.wute().clear_bit()) };

                unsafe { (*pac::RTC::ptr()).isr.modify(|_, w| w.init().set_bit()) };
                while unsafe { (*pac::RTC::ptr()).isr.read().initf().bit_is_clear() } {}

                unsafe { (*pac::RTC::ptr()).isr.modify(|_, w| w.wutf().clear_bit()) };

                unsafe { (*pac::RTC::ptr()).isr.modify(|_, w| w.init().clear_bit()) };
                while !unsafe { (*pac::RTC::ptr()).isr.read().initf().bit_is_clear() } {}

                unsafe { (*pac::RTC::ptr()).cr.modify(|_, w| w.wute().set_bit()) };
                unsafe { (*pac::RTC::ptr()).wpr.write(|w| w.bits(0xFF)) };
            });
        }
    };
}

/// Invalid input error
#[derive(Debug)]
pub enum Error {
    InvalidInputData,
}

pub const LSE_BITS: u8 = 0b01;

/// See ref man, section 27.6.3, or AN4769, section 2.4.2.
/// To be used with WakeupPrescaler
#[derive(Clone, Copy, Debug)]
pub enum WakeupDivision {
    Sixteen,
    Eight,
    Four,
    Two,
}

/// See AN4759, table 13.
#[derive(Clone, Copy, Debug)]
pub enum ClockConfig {
    One(WakeupDivision),
    Two,
    Three,
}

pub struct Rtc {
    pub regs: RTC,
}

impl Rtc {
    /// Create and enable a new RTC, and configure its clock source and prescalers.
    /// From AN4759, Table 7, when using the LSE (The only clock source this module
    /// supports currently), set `prediv_s` to 255, and `prediv_a` to 127 to get a
    /// calendar clock of 1Hz.
    /// The `bypass` argument is `true` if you're using an external oscillator that
    /// doesn't connect to `OSC32_IN`, such as a MEMS resonator.
    /// Note: You may need to run `dp.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());` before
    /// constraining RCC, eg before running this constructor.
    pub fn new(
        regs: RTC,
        prediv_s: u16,
        prediv_a: u8,
        bypass: bool,
        apb1: &mut APB1,
        bdcr: &mut BDCR,
        pwr: &mut PWR,
    ) -> Self {
        let mut result = Self { regs };

        reset(bdcr);
        unlock(apb1, pwr); // Must unlock before enabling LSE.
        enable_lse(bdcr, bypass);
        enable(bdcr);

        result.set_24h_fmt();

        result.regs.prer.modify(|_, w| {
            w.prediv_s().bits(prediv_s);
            w.prediv_a().bits(prediv_a)
        });

        result
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

    #[cfg(any(feature = "rt"))]
    #[cfg(not(any(feature = "stm32f301", feature = "stm32f373", feature = "stm32f398")))]
    /// Setup periodic auto-akeup interrupts. See ST AN4759, Table 11, and more broadly,
    /// section 2.4.1. See also reference manual, section 27.5.
    /// In addition to running this function, set up the interrupt handling function by
    /// adding the line `make_rtc_interrupt_handler!(RTC_WKUP);` somewhere in the body
    /// of your program.
    /// `sleep_time` is in ms.
    pub fn set_auto_wakeup(&mut self, exti: &mut EXTI, clock_cfg: ClockConfig, sleep_time: u32) {
        // Configure and enable the EXTI line corresponding to the Wakeup timer even in
        // interrupt mode and select the rising edge sensitivity.
        exti.imr1.modify(|_, w| w.mr20().unmasked());
        exti.rtsr1.modify(|_, w| w.tr20().bit(true));
        exti.ftsr1.modify(|_, w| w.tr20().bit(false));

        // Configure and Enable the RTC_WKUP IRQ channel in the NVIC.
        unsafe { NVIC::unmask(RTC_WKUP) };

        // Disable the RTC registers Write protection.
        // Write 0xCA and then 0x53 into the RTC_WPR register. RTC registers can then be modified.
        self.regs.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.regs.wpr.write(|w| unsafe { w.bits(0x53) });

        // Disable the wakeup timer. Clear WUTE bit in RTC_CR register
        self.regs.cr.modify(|_, w| w.wute().clear_bit());

        // Ensure access to Wakeup auto-reload counter and bits WUCKSEL[2:0] is allowed.
        // Poll WUTWF until it is set in RTC_ISR (RTC2)/RTC_ICSR (RTC3)
        while !self.regs.isr.read().wutwf().bit_is_set() {}

        // Program the value into the wakeup timer
        // Set WUT[15:0] in RTC_WUTR register. For RTC3 the user must also program
        // WUTOCLR bits.
        // See ref man Section 2.4.2: Maximum and minimum RTC wakeup period.
        // todo check ref man register table
        let lfe_freq = 32_768;
        let sleep_for_cycles = lfe_freq * sleep_time / 1_000;
        self.regs
            .wutr
            .modify(|_, w| w.wut().bits(sleep_for_cycles as u16));

        // Select the desired clock source. Program WUCKSEL[2:0] bits in RTC_CR register.
        // See ref man Section 2.4.2: Maximum and minimum RTC wakeup period.
        // todo: Check register docs and see what to set here.

        // See AN4759, Table 13.
        let word = match clock_cfg {
            ClockConfig::One(division) => match division {
                WakeupDivision::Sixteen => 0b000,
                WakeupDivision::Eight => 0b001,
                WakeupDivision::Four => 0b010,
                WakeupDivision::Two => 0b011,
            },
            // for 2 and 3, what does `x` mean in the docs? Best guess is it doesn't matter.
            ClockConfig::Two => 0b100,
            ClockConfig::Three => 0b110,
        };

        // 000: RTC/16 clock is selected
        // 001: RTC/8 clock is selected
        // 010: RTC/4 clock is selected
        // 011: RTC/2 clock is selected
        // 10x: ck_spre (usually 1 Hz) clock is selected
        // 11x: ck_spre (usually 1 Hz) clock is selected and 216 is added to the WUT counter value
        self.regs
            .cr
            .modify(|_, w| unsafe { w.wucksel().bits(word) });

        // Re-enable the wakeup timer. Set WUTE bit in RTC_CR register.
        // The wakeup timer restarts counting down.
        self.regs.cr.modify(|_, w| w.wute().set_bit());

        // Enable the wakeup timer interrupt.
        self.regs.cr.modify(|_, w| w.wutie().set_bit());

        // Clear the  wakeup flag.
        self.regs.isr.modify(|_, w| w.wutf().clear_bit());

        // Enable the RTC registers Write protection. Write 0xFF into the
        // RTC_WPR register. RTC registers can no more be modified.
        self.regs.wpr.write(|w| unsafe { w.bits(0xFF) });
    }

    #[cfg(any(feature = "rt"))]
    /// Clears the wakeup flag. Must be cleared manually after every RTC wakeup.
    /// Alternatively, you could handle this in the EXTI handler function.
    pub fn clear_wakeup_flag(&mut self) {
        self.modify(|regs| {
            regs.cr.modify(|_, w| w.wute().clear_bit());
            regs.isr.modify(|_, w| w.wutf().clear_bit());
            regs.cr.modify(|_, w| w.wute().set_bit());
        });
    }

    /// As described in Section 27.3.7 in RM0316,
    /// this function is used to disable write protection
    /// when modifying an RTC register.
    fn modify<F>(&mut self, mut closure: F)
    where
        F: FnMut(&mut RTC),
    {
        // Disable write protection
        self.regs.wpr.write(|w| unsafe { w.bits(0xCA) });
        self.regs.wpr.write(|w| unsafe { w.bits(0x53) });
        // Enter init mode
        let isr = self.regs.isr.read();
        if isr.initf().bit_is_clear() {
            self.regs.isr.modify(|_, w| w.init().set_bit());
            while self.regs.isr.read().initf().bit_is_clear() {}
        }
        // Invoke closure
        closure(&mut self.regs);
        // Exit init mode
        self.regs.isr.modify(|_, w| w.init().clear_bit());
        // wait for last write to be done
        while !self.regs.isr.read().initf().bit_is_clear() {}
        self.regs.wpr.write(|w| unsafe { w.bits(0xFF) });
    }
}

impl Rtcc for Rtc {
    type Error = Error;

    /// set time using NaiveTime (ISO 8601 time without timezone)
    /// Hour format is 24h
    fn set_time(&mut self, time: &NaiveTime) -> Result<(), Self::Error> {
        self.set_24h_fmt();
        let (ht, hu) = bcd2_encode(time.hour())?;
        let (mnt, mnu) = bcd2_encode(time.minute())?;
        let (st, su) = bcd2_encode(time.second())?;

        self.modify(|regs| {
            regs.tr.write(|w| {
                w.ht().bits(ht);
                w.hu().bits(hu);
                w.mnt().bits(mnt);
                w.mnu().bits(mnu);
                w.st().bits(st);
                w.su().bits(su);
                w.pm().clear_bit()
            })
        });

        Ok(())
    }

    fn set_seconds(&mut self, seconds: u8) -> Result<(), Self::Error> {
        if seconds > 59 {
            return Err(Error::InvalidInputData);
        }
        let (st, su) = bcd2_encode(seconds as u32)?;
        self.modify(|regs| regs.tr.modify(|_, w| w.st().bits(st).su().bits(su)));

        Ok(())
    }

    fn set_minutes(&mut self, minutes: u8) -> Result<(), Self::Error> {
        if minutes > 59 {
            return Err(Error::InvalidInputData);
        }
        let (mnt, mnu) = bcd2_encode(minutes as u32)?;
        self.modify(|regs| regs.tr.modify(|_, w| w.mnt().bits(mnt).mnu().bits(mnu)));

        Ok(())
    }

    fn set_hours(&mut self, hours: Hours) -> Result<(), Self::Error> {
        let (ht, hu) = hours_to_register(hours)?;
        match hours {
            Hours::H24(_h) => self.set_24h_fmt(),
            Hours::AM(_h) | Hours::PM(_h) => self.set_12h_fmt(),
        }

        self.regs.tr.modify(|_, w| w.ht().bits(ht).hu().bits(hu));

        Ok(())
    }

    fn set_weekday(&mut self, weekday: u8) -> Result<(), Self::Error> {
        if (weekday < 1) || (weekday > 7) {
            return Err(Error::InvalidInputData);
        }
        self.modify(|regs| regs.dr.modify(|_, w| unsafe { w.wdu().bits(weekday) }));

        Ok(())
    }

    fn set_day(&mut self, day: u8) -> Result<(), Self::Error> {
        if (day < 1) || (day > 31) {
            return Err(Error::InvalidInputData);
        }
        let (dt, du) = bcd2_encode(day as u32)?;
        self.modify(|regs| regs.dr.modify(|_, w| w.dt().bits(dt).du().bits(du)));

        Ok(())
    }

    fn set_month(&mut self, month: u8) -> Result<(), Self::Error> {
        if (month < 1) || (month > 12) {
            return Err(Error::InvalidInputData);
        }
        let (mt, mu) = bcd2_encode(month as u32)?;
        self.modify(|regs| regs.dr.modify(|_, w| w.mt().bit(mt > 0).mu().bits(mu)));

        Ok(())
    }

    fn set_year(&mut self, year: u16) -> Result<(), Self::Error> {
        if (year < 1970) || (year > 2038) {
            return Err(Error::InvalidInputData);
        }
        let (yt, yu) = bcd2_encode(year as u32)?;
        self.modify(|regs| regs.dr.modify(|_, w| w.yt().bits(yt).yu().bits(yu)));

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

        self.modify(|regs| {
            regs.dr.write(|w| {
                w.dt().bits(dt);
                w.du().bits(du);
                w.mt().bit(mt > 0);
                w.mu().bits(mu);
                w.yt().bits(yt);
                w.yu().bits(yu)
            })
        });

        Ok(())
    }

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

        self.modify(|regs| {
            regs.dr.write(|w| {
                w.dt().bits(dt);
                w.du().bits(du);
                w.mt().bit(mt > 0);
                w.mu().bits(mu);
                w.yt().bits(yt);
                w.yu().bits(yu)
            })
        });

        self.modify(|regs| {
            regs.tr.write(|w| {
                w.ht().bits(ht);
                w.hu().bits(hu);
                w.mnt().bits(mnt);
                w.mnu().bits(mnu);
                w.st().bits(st);
                w.su().bits(su);
                w.pm().clear_bit()
            })
        });

        Ok(())
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

    fn get_hours(&mut self) -> Result<Hours, Self::Error> {
        let tr = self.regs.tr.read();
        let hours = bcd2_decode(tr.ht().bits(), tr.hu().bits());
        if self.is_24h_fmt() {
            return Ok(Hours::H24(hours as u8));
        }
        if !tr.pm().bit() {
            return Ok(Hours::AM(hours as u8));
        }
        Ok(Hours::PM(hours as u8))
    }

    fn get_time(&mut self) -> Result<NaiveTime, Self::Error> {
        self.set_24h_fmt();
        let seconds = self.get_seconds().unwrap();
        let minutes = self.get_minutes().unwrap();
        let hours = hours_to_u8(self.get_hours()?)?;

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
        let hours = hours_to_u8(self.get_hours()?)?;

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
    bdcr.bdcr().modify(|_, w| w.lseon().set_bit());
    bdcr.bdcr().modify(|_, w| w.lsebyp().bit(bypass));

    while bdcr.bdcr().read().lserdy().bit_is_clear() {}
}

fn unlock(apb1: &mut APB1, pwr: &mut PWR) {
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

/// Enables the RTC, and sets LSE as the timing source.
pub fn enable(bdcr: &mut BDCR) {
    bdcr.bdcr().modify(|_, w| w.rtcsel().lse());
    bdcr.bdcr().modify(|_, w| w.rtcen().enabled());
}

/// Disables the RTC.
pub fn disable(bdcr: &mut BDCR) {
    bdcr.bdcr().modify(|_, w| w.rtcen().disabled());
}

/// Resets the entire RTC domain
fn reset(bdcr: &mut BDCR) {
    bdcr.bdcr().modify(|_, w| w.bdrst().enabled());
    bdcr.bdcr().modify(|_, w| w.bdrst().disabled());
}
