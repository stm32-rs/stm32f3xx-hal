use crate::rcc::Rcc;
use crate::stm32::RTC;
use crate::time::{Date, Time, U32Ext};

/**
 Interface to the real time clock
**/

pub struct Rtc {
    regs: RTC,
}

impl Rtc {
    /**
    Initializes the RTC
     **/
    // Sets default clock source to LSI, since LSE is not included on STM32f3 discovery boards
    pub fn rtc(regs: RTC, src: RTCSrc, rcc: &mut Rcc) -> Self {
        let mut result = Rtc { regs };

        rcc.enable_rtc(&src);

        result.regs.cr.modify(|_, w| {
            w
                // sets hour format to 24 hours
                .fmt()
                .clear_bit()
        });

        // Prescalers set to produce a 1 hz signal
        let (prediv_s, prediv_a) = match src {
            RTCSrc::LSI => (311_u32, 127_u32),
            RTCSrc::HSE => (62992_u32, 127_u32),
            RTCSrc::LSE => (255_u32, 127_u32),
        };

        let raw_bits: u32 = prediv_s | (prediv_a << 16);
        result.modify(|regs| {
            regs.prer.write(|w| unsafe { 
                w.prediv_s().bits(prediv_s).prediv_a().bits(prediv_a)
            });
        });
        result
    }

    pub fn set_time(&mut self, time: &Time) {
        let (ht, hu) = bcd2_encode(time.hours);
        let (mnt, mnu) = bcd2_encode(time.minutes);
        let (st, su) = bcd2_encode(time.seconds);
        self.modify(|regs| {
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
            regs.cr.modify(|_, w| w.fmt().bit(time.daylight_savings));
        });
    }

    pub fn set_date(&mut self, date: &Date) {
        let (yt, yu) = bcd2_encode(date.year - 1970);
        let (mt, mu) = bcd2_encode(date.month);
        let (dt, du) = bcd2_encode(date.day);

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
                    .wdu()
                    .bits(date.day as u8)
            });
        });
    }

    pub fn get_time(&self) -> Time {
        let timer = self.regs.tr.read();
        Time::new(
            bcd2_decode(timer.ht().bits(), timer.hu().bits()).hours(),
            bcd2_decode(timer.mnt().bits(), timer.mnu().bits()).minutes(),
            bcd2_decode(timer.st().bits(), timer.su().bits()).seconds(),
            self.regs.cr.read().fmt().bit(),
        )
    }

    pub fn get_date(&self) -> Date {
        let date = self.regs.dr.read();
        Date::new(
            (bcd2_decode(date.yt().bits(), date.yu().bits()) + 1970).year(),
            bcd2_decode(date.mt().bit() as u8, date.mu().bits()).month(),
            bcd2_decode(date.dt().bits(), date.du().bits()).day(),
        )
    }

    pub fn get_week_day(&self) -> u8 {
        self.regs.dr.read().wdu().bits()
    }

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

/// RTC clock input source
#[derive(Clone, Copy)]
pub enum RTCSrc {
    LSE = 0b01,
    LSI = 0b10,
    HSE = 0b11,
}

pub trait RtcExt {
    fn constrain(self, rcc: &mut Rcc) -> Rtc;
}

impl RtcExt for RTC {
    fn constrain(self, rcc: &mut Rcc) -> Rtc {
        Rtc::rtc(self, RTCSrc::LSI, rcc)
    }
}

fn bcd2_encode(word: u32) -> (u8, u8) {
    let mut value = word as u8;
    let mut bcd_high: u8 = 0;
    while value >= 10 {
        bcd_high += 1;
        value -= 10;
    }
    let bcd_low = ((bcd_high << 4) | value) as u8;
    (bcd_high, bcd_low)
}

fn bcd2_decode(fst: u8, snd: u8) -> u32 {
    let value = snd | fst << 4;
    let value = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;
    value as u32
}
