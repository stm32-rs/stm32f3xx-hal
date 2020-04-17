//! Time units

use cortex_m::peripheral::DWT;

use crate::rcc::Clocks;

use core::fmt;

/// Bits per second
#[derive(Clone, Copy)]
pub struct Bps(pub u32);

/// Hertz
#[derive(Clone, Copy)]
pub struct Hertz(pub u32);

/// KiloHertz
#[derive(Clone, Copy)]
pub struct KiloHertz(pub u32);

/// MegaHertz
#[derive(Clone, Copy)]
pub struct MegaHertz(pub u32);

/// Time unit
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliSeconds(pub u32);

/// Seconds
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Second(pub u32);

/// Minutes
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Minute(pub u32);

/// Hours
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hour(pub u32);

/// WeekDay (1-7)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeekDay(pub u32);

/// Date (1-31)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MonthDay(pub u32);

/// Week (1-52)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Week(pub u32);

/// Month (1-12)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Month(pub u32);

/// Year
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Year(pub u32);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Time {
    pub hours: u32,
    pub minutes: u32,
    pub seconds: u32,
    pub daylight_savings: bool,
}

impl Time {
    pub fn new(hours: Hour, minutes: Minute, seconds: Second, daylight_savings: bool) -> Self {
        Self {
            hours: hours.0,
            minutes: minutes.0,
            seconds: seconds.0,
            daylight_savings,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Date {
    pub day: u32,
    pub month: u32,
    pub year: u32,
}

impl Date {
    pub fn new(year: Year, month: Month, day: MonthDay) -> Self {
        Self {
            day: day.0,
            month: month.0,
            year: year.0,
        }
    }
}

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hertz`
    fn hz(self) -> Hertz;

    /// Wrap in `KiloHertz`
    fn khz(self) -> KiloHertz;

    /// Wrap in `MegaHertz`
    fn mhz(self) -> MegaHertz;

    /// Wrap in `MilliSeconds`
    fn ms(self) -> MilliSeconds;

    /// Seconds
    fn seconds(self) -> Second;

    /// Minutes
    fn minutes(self) -> Minute;

    /// Hours
    fn hours(self) -> Hour;

    /// Day in month
    fn day(self) -> MonthDay;

    /// Month
    fn month(self) -> Month;

    /// Year
    fn year(self) -> Year;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn khz(self) -> KiloHertz {
        KiloHertz(self)
    }

    fn mhz(self) -> MegaHertz {
        MegaHertz(self)
    }

    fn ms(self) -> MilliSeconds {
        MilliSeconds(self)
    }

    fn seconds(self) -> Second {
        Second(self)
    }

    fn minutes(self) -> Minute {
        Minute(self)
    }

    fn hours(self) -> Hour {
        Hour(self)
    }

    fn day(self) -> MonthDay {
        MonthDay(self)
    }

    fn month(self) -> Month {
        Month(self)
    }

    fn year(self) -> Year {
        Year(self)
    }
}

impl From<KiloHertz> for Hertz {
    fn from(val: KiloHertz) -> Self {
        Self(val.0 * 1_000)
    }
}

impl From<MegaHertz> for Hertz {
    fn from(val: MegaHertz) -> Self {
        Self(val.0 * 1_000_000)
    }
}

impl From<MegaHertz> for KiloHertz {
    fn from(val: MegaHertz) -> Self {
        Self(val.0 * 1_000)
    }
}

/// A monotonic nondecreasing timer
#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, clocks: Clocks) -> Self {
        dwt.enable_cycle_counter();

        // now the CYCCNT counter can't be stopped or resetted
        drop(dwt);

        MonoTimer {
            frequency: clocks.hclk(),
        }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(&self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(&self) -> Instant {
        Instant {
            now: DWT::get_cycle_count(),
        }
    }
}

/// A measurement of a monotonically nondecreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(&self) -> u32 {
        DWT::get_cycle_count().wrapping_sub(self.now)
    }
}
