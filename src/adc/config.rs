//! Provides a [`Config`] for ADC configuration and all configuration options options.
//!
//! The `Config` can be applied like that:
//!
//! ```
//! use stm32f3xx_hal::adc::{
//!     Adc,
//!     config::{self, Config},
//! }
//!
//! # let dp = pac::Peripherals::take::unwrap()
//!
//! let config = Config::default()
//!     .align(config::Align::Left)
//!     .resolution(config::Resolution::Eight);
//!
//! let adc = Adc::new(dp.ADC1, config, /* ... */);
//! ```
//!
//! which will configure the peripheral accordingly.
//!
//! Configuration can be changed later on:
//!
//! ```
//! let adc = // ...
//!
//! // ...
//!
//! adc::set_align(config::Align::Right);
//! ```

use crate::pac::adc1::{cfgr, smpr1::SMP1_A, smpr2::SMP10_A};
use core::convert::TryFrom;

/// The place in the sequence a given channel should be captured.
/// See [`Channel`] / [`channel::Id`]
///
/// # Note
///
/// * In some STM documentations this might also be called "rank".
///
/// # Related functions
///
/// * [`Adc::set_pin_sequence_position`]
/// * [`Adc::set_channel_sequence_position`]
/// * [`Adc::channel_sequence`]
///
/// [`Channel`]: `embedded_hal::adc::Channel`
/// [`channel::Id`]: `super::channel::Id`
/// [`Adc::set_pin_sequence_position`]: `super::Adc::set_pin_sequence_position`
/// [`Adc::set_channel_sequence_position`]: `super::Adc::set_channel_sequence_position`
/// [`Adc::channel_sequence`]: `super::Adc::channel_sequence`
#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum Sequence {
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
    Nine,
    Ten,
    Eleven,
    Twelve,
    Thirteen,
    Fourteen,
    Fifteen,
    Sixteen,
}

impl From<Sequence> for u8 {
    fn from(s: Sequence) -> u8 {
        match s {
            Sequence::One => 0,
            Sequence::Two => 1,
            Sequence::Three => 2,
            Sequence::Four => 3,
            Sequence::Five => 4,
            Sequence::Six => 5,
            Sequence::Seven => 6,
            Sequence::Eight => 7,
            Sequence::Nine => 8,
            Sequence::Ten => 9,
            Sequence::Eleven => 10,
            Sequence::Twelve => 11,
            Sequence::Thirteen => 12,
            Sequence::Fourteen => 13,
            Sequence::Fifteen => 14,
            Sequence::Sixteen => 15,
        }
    }
}

impl TryFrom<u8> for Sequence {
    type Error = crate::TryFromIntError;

    fn try_from(bits: u8) -> Result<Self, Self::Error> {
        Ok(match bits {
            0 => Sequence::One,
            1 => Sequence::Two,
            2 => Sequence::Three,
            3 => Sequence::Four,
            4 => Sequence::Five,
            5 => Sequence::Six,
            6 => Sequence::Seven,
            7 => Sequence::Eight,
            8 => Sequence::Nine,
            9 => Sequence::Ten,
            10 => Sequence::Eleven,
            11 => Sequence::Twelve,
            12 => Sequence::Thirteen,
            13 => Sequence::Fourteen,
            14 => Sequence::Fifteen,
            15 => Sequence::Sixteen,
            _ => return Err(crate::TryFromIntError),
        })
    }
}

impl From<Sequence> for usize {
    fn from(s: Sequence) -> usize {
        match s {
            Sequence::One => 0,
            Sequence::Two => 1,
            Sequence::Three => 2,
            Sequence::Four => 3,
            Sequence::Five => 4,
            Sequence::Six => 5,
            Sequence::Seven => 6,
            Sequence::Eight => 7,
            Sequence::Nine => 8,
            Sequence::Ten => 9,
            Sequence::Eleven => 10,
            Sequence::Twelve => 11,
            Sequence::Thirteen => 12,
            Sequence::Fourteen => 13,
            Sequence::Fifteen => 14,
            Sequence::Sixteen => 15,
        }
    }
}

/// The bit width / resolution to sample with the ADC.
///
/// # Related functions
///
/// * [`Adc::set_resolution`]
/// * [`Adc::resolution`]
///
/// [`Adc::set_resolution`]: [`super::Adc::set_resolution`]
/// [`Adc::resolution`]: [`super::Adc::resolution`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Resolution {
    /// 12-bit
    Twelve,
    /// 10-bit
    Ten,
    /// 8-bit
    Eight,
    /// 6-bit
    Six,
}

impl From<Resolution> for cfgr::RES_A {
    fn from(r: Resolution) -> Self {
        match r {
            Resolution::Twelve => cfgr::RES_A::BITS12,
            Resolution::Ten => cfgr::RES_A::BITS10,
            Resolution::Eight => cfgr::RES_A::BITS8,
            Resolution::Six => cfgr::RES_A::BITS6,
        }
    }
}

impl From<cfgr::RES_A> for Resolution {
    fn from(r: cfgr::RES_A) -> Self {
        match r {
            cfgr::RES_A::BITS12 => Resolution::Twelve,
            cfgr::RES_A::BITS10 => Resolution::Ten,
            cfgr::RES_A::BITS8 => Resolution::Eight,
            cfgr::RES_A::BITS6 => Resolution::Six,
        }
    }
}

impl Default for Resolution {
    fn default() -> Self {
        Self::Twelve
    }
}

/// Possible external triggers which can trigger a start of conversion instead of
/// doing it manually via [`Adc::start_conversion`].
///
/// # Related functions
///
/// * [`Adc::set_external_trigger`]
/// * [`Adc::external_trigger`]
///
/// [`Adc::set_external_trigger`]: `super::Adc::set_external_trigger`
/// [`Adc::external_trigger`]: `super::Adc::external_trigger`
/// [`Adc::start_conversion`]: `super::Adc::start_conversion`
// TODO(Sh3rm4n): Implement from / into to timer::Event when supported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExternalTrigger {
    /// Timer 1 CC1 event
    Tim1Cc1(TriggerMode),
    /// Timer 1 CC2 event
    Tim1Cc2(TriggerMode),
    /// Timer 1 CC3 event
    Tim1Cc3(TriggerMode),
    /// Timer 2 CC2 event
    Tim2Cc2(TriggerMode),
    /// Timer 3 TRGO event
    Tim3Trgo(TriggerMode),
    /// EXTI line 11
    Exti11(TriggerMode),
    /// HRTIM_ADCTRG1 event
    HrtimAdcTrg1(TriggerMode),
    /// HRTIM_ADCTRG3 event
    HrtimAdcTrg3(TriggerMode),
    /// Timer 1 TRGO event
    Tim1Trgo(TriggerMode),
    /// Timer 1 TRGO2 event
    Tim1Trgo2(TriggerMode),
    /// Timer 2 TRGO event
    Tim2Trgo(TriggerMode),
    /// Timer 6 TRGO event
    Tim6Trgo(TriggerMode),
    /// Timer 15 TRGO event
    Tim15Trgo(TriggerMode),
    /// Timer 3 CC4 event
    Tim3Cc4(TriggerMode),
}

impl From<ExternalTrigger> for cfgr::EXTSEL_A {
    fn from(et: ExternalTrigger) -> Self {
        match et {
            ExternalTrigger::Tim1Cc1(_) => cfgr::EXTSEL_A::TIM1_CC1,
            ExternalTrigger::Tim1Cc2(_) => cfgr::EXTSEL_A::TIM1_CC2,
            ExternalTrigger::Tim1Cc3(_) => cfgr::EXTSEL_A::TIM1_CC3,
            ExternalTrigger::Tim2Cc2(_) => cfgr::EXTSEL_A::TIM2_CC2,
            ExternalTrigger::Tim3Trgo(_) => cfgr::EXTSEL_A::TIM3_TRGO,
            ExternalTrigger::Exti11(_) => cfgr::EXTSEL_A::EXTI11,
            ExternalTrigger::HrtimAdcTrg1(_) => cfgr::EXTSEL_A::HRTIM_ADCTRG1,
            ExternalTrigger::HrtimAdcTrg3(_) => cfgr::EXTSEL_A::HRTIM_ADCTRG3,
            ExternalTrigger::Tim1Trgo(_) => cfgr::EXTSEL_A::TIM1_TRGO,
            ExternalTrigger::Tim1Trgo2(_) => cfgr::EXTSEL_A::TIM1_TRGO2,
            ExternalTrigger::Tim2Trgo(_) => cfgr::EXTSEL_A::TIM2_TRGO,
            ExternalTrigger::Tim6Trgo(_) => cfgr::EXTSEL_A::TIM6_TRGO,
            ExternalTrigger::Tim15Trgo(_) => cfgr::EXTSEL_A::TIM15_TRGO,
            ExternalTrigger::Tim3Cc4(_) => cfgr::EXTSEL_A::TIM1_CC3,
        }
    }
}

impl From<ExternalTrigger> for cfgr::EXTEN_A {
    fn from(et: ExternalTrigger) -> Self {
        match et {
            ExternalTrigger::Tim1Cc1(n)
            | ExternalTrigger::Tim1Cc2(n)
            | ExternalTrigger::Tim1Cc3(n)
            | ExternalTrigger::Tim2Cc2(n)
            | ExternalTrigger::Tim3Trgo(n)
            | ExternalTrigger::Exti11(n)
            | ExternalTrigger::HrtimAdcTrg1(n)
            | ExternalTrigger::HrtimAdcTrg3(n)
            | ExternalTrigger::Tim1Trgo(n)
            | ExternalTrigger::Tim1Trgo2(n)
            | ExternalTrigger::Tim2Trgo(n)
            | ExternalTrigger::Tim6Trgo(n)
            | ExternalTrigger::Tim15Trgo(n)
            | ExternalTrigger::Tim3Cc4(n) => n.into(),
        }
    }
}

/// Possible trigger modes of the [`ExternalTrigger`].
///
/// # Related functions
///
/// * [`Adc::set_external_trigger`]
/// * [`Adc::external_trigger`]
///
/// [`Adc::set_external_trigger`]: `super::Adc::set_external_trigger`
/// [`Adc::external_trigger`]: `super::Adc::external_trigger`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TriggerMode {
    /// Listen for rising edges of external trigger
    RisingEdge,
    /// Listen for falling edges of external trigger
    FallingEdge,
    /// Listen for both rising and falling edges of external trigger
    BothEdges,
}

impl From<TriggerMode> for cfgr::EXTEN_A {
    fn from(tm: TriggerMode) -> Self {
        match tm {
            TriggerMode::RisingEdge => cfgr::EXTEN_A::RISINGEDGE,
            TriggerMode::FallingEdge => cfgr::EXTEN_A::FALLINGEDGE,
            TriggerMode::BothEdges => cfgr::EXTEN_A::BOTHEDGES,
        }
    }
}

/// Configure the data register alignment.
///
/// At the end of each regular conversion channel (when EOC event occurs), the result of the
/// converted data is stored into the [data register](`super::Adc::data_register`) which is 16 bits wide.
///
/// For more details see [RM0316] _Figure 82-85_..
///
/// # Related functions
///
/// * [`Adc::set_data_alignment`]
/// * [`Adc::data_alignment`]
///
/// [`Adc::set_data_alignment`]: `super::Adc::set_data_alignment`
/// [`Adc::data_alignment`]: `super::Adc::data_alignment`
/// [RM0316]: https://www.st.com/resource/en/reference_manual/dm00094349.pdf
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataAlignment {
    /// Right align output data
    Right,
    /// Left align output data
    Left,
}

impl From<DataAlignment> for cfgr::ALIGN_A {
    fn from(a: DataAlignment) -> Self {
        match a {
            DataAlignment::Right => cfgr::ALIGN_A::RIGHT,
            DataAlignment::Left => cfgr::ALIGN_A::LEFT,
        }
    }
}

impl From<cfgr::ALIGN_A> for DataAlignment {
    fn from(a: cfgr::ALIGN_A) -> Self {
        match a {
            cfgr::ALIGN_A::RIGHT => DataAlignment::Right,
            cfgr::ALIGN_A::LEFT => DataAlignment::Left,
        }
    }
}

impl Default for DataAlignment {
    fn default() -> Self {
        Self::Right
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f373")] {
        /// This mode is used to scan a group of analog channels
        ///
        /// After each end of conversion the next channel of the group is converted automatically.
        /// If [`ConersionMode::Continuous`] is set, conversion does not stop at the last selected
        /// group channel but continues again from the first selected group channel
        ///
        /// # Related functions
        ///
        /// * [`Adc::set_scan`]
        /// * [`Adc::scan`]
        ///
        /// [`Adc::set_scan`]: `super::Adc::set_scan`
        /// [`Adc::scan`]: `super::Adc::scan`
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum Scan {
            /// Right align output data
            Disabled,
            /// Left align output data
            Enabled,
        }

        impl From<Scan> for cr::SCAN_R {
            fn from(a: Scan) -> Self {
                match a {
                    Scan::Disabled => cr1::SCAN_R::DISABLED,
                    Scan::Enabled => cr1::SCAN_R::ENABLED,
                }
            }
        }

        impl From<Scan> for cr::SCAN_R {
            fn from(a: Scan) -> Self {
                match a {
                    Scan::Disabled => cr1::SCAN_R::DISABLED,
                    Scan::Enabled => cr1::SCAN_R::ENABLED,
                }
            }
        }

        impl Default for Scan {
            fn default() -> Self {
                Self::Disabled
            }
        }
    }
}
/// Overrun Mode
///
/// This configures the way data overrun is managed, if the data in the
/// [data register](`super::Adc::data_register`) was not already read from and is about to be overwritten.
///
/// # Note
///
/// This can only be changed, when no conversion is ongoing
/// (ADSTART=0).
///
/// # Related function
///
/// * [`Adc::set_overrun_mode`]
/// * [`Adc::overrun_mode`]
///
/// [`Adc::set_overrun_mode`]: `super::Adc::set_overrun_mode`
/// [`Adc::overrun_mode`]: `super::Adc::overrun_mode`
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OverrunMode {
    /// Preserve old data register content when an overrun is detected
    Preserve,
    /// Overwrite data register with the last conversion result
    /// when an overrun is detected.
    Overwrite,
}

impl From<cfgr::OVRMOD_A> for OverrunMode {
    fn from(a: cfgr::OVRMOD_A) -> Self {
        match a {
            cfgr::OVRMOD_A::PRESERVE => OverrunMode::Preserve,
            cfgr::OVRMOD_A::OVERWRITE => OverrunMode::Overwrite,
        }
    }
}

impl From<OverrunMode> for cfgr::OVRMOD_A {
    fn from(a: OverrunMode) -> Self {
        match a {
            OverrunMode::Preserve => cfgr::OVRMOD_A::PRESERVE,
            OverrunMode::Overwrite => cfgr::OVRMOD_A::OVERWRITE,
        }
    }
}

impl Default for OverrunMode {
    fn default() -> Self {
        OverrunMode::Preserve
    }
}

/// ADC sampling time
///
/// Each channel can be sampled with a different sample time.
/// There is always an overhead of 13 ADC clock cycles.
///
/// E.g. For SampleTime::T19C5 the total conversion time (in ADC clock cycles) is
/// 13 + 19 = 32 ADC Clock Cycles
///
/// # Related functions
///
/// * [`Adc::set_sample_time`]
/// * [`Adc::sample_time`]
///
/// [`Adc::set_sample_time`]: `super::Adc::set_sample_time`
/// [`Adc::sample_time`]: `super::Adc::sample_time`
///
/// # Note
///
/// Currently, no way to set the specific clock of the [`Adc`] is
/// implemented. By default the [`Adc`] peripheral is clocked by the [`Clock::pllclk`],
/// and will fall back to using the [`Clock::pllclk`], if that is not set.
///
/// [`Clock::sysclk`]: `crate::rcc::Clocks::sysclk`
/// [`Clock::pllclk`]: `crate::rcc::Clocks::pllclk`
/// [`Adc`]: `super::Adc`
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleTime {
    /// 1.5 ADC clock cycles
    Cycles1C5,
    /// 2.5 ADC clock cycles
    Cycles2C5,
    /// 4.5 ADC clock cycles
    Cycles4C5,
    /// 7.5 ADC clock cycles
    Cycles7C5,
    /// 19.5 ADC clock cycles
    Cycles19C5,
    /// 61.5 ADC clock cycles
    Cycles61C5,
    /// 181.5 ADC clock cycles
    Cycles181C5,
    /// 601.5 ADC clock cycles
    Cycles601C5,
}

impl Default for SampleTime {
    /// [`SampleTime::Cycles1C5`] is also the reset value.
    fn default() -> Self {
        SampleTime::Cycles1C5
    }
}

impl From<SampleTime> for u16 {
    fn from(t: SampleTime) -> Self {
        match t {
            SampleTime::Cycles1C5 => 2,
            SampleTime::Cycles2C5 => 3,
            SampleTime::Cycles4C5 => 5,
            SampleTime::Cycles7C5 => 8,
            SampleTime::Cycles19C5 => 20,
            SampleTime::Cycles61C5 => 62,
            SampleTime::Cycles181C5 => 182,
            SampleTime::Cycles601C5 => 602,
        }
    }
}

impl From<SampleTime> for f32 {
    fn from(t: SampleTime) -> Self {
        match t {
            SampleTime::Cycles1C5 => 1.5,
            SampleTime::Cycles2C5 => 2.5,
            SampleTime::Cycles4C5 => 4.5,
            SampleTime::Cycles7C5 => 7.5,
            SampleTime::Cycles19C5 => 19.5,
            SampleTime::Cycles61C5 => 61.5,
            SampleTime::Cycles181C5 => 181.5,
            SampleTime::Cycles601C5 => 601.5,
        }
    }
}

impl From<SampleTime> for SMP1_A {
    fn from(t: SampleTime) -> Self {
        match t {
            SampleTime::Cycles1C5 => Self::CYCLES1_5,
            SampleTime::Cycles2C5 => Self::CYCLES2_5,
            SampleTime::Cycles4C5 => Self::CYCLES4_5,
            SampleTime::Cycles7C5 => Self::CYCLES7_5,
            SampleTime::Cycles19C5 => Self::CYCLES19_5,
            SampleTime::Cycles61C5 => Self::CYCLES61_5,
            SampleTime::Cycles181C5 => Self::CYCLES181_5,
            SampleTime::Cycles601C5 => Self::CYCLES601_5,
        }
    }
}

impl From<SampleTime> for SMP10_A {
    fn from(t: SampleTime) -> Self {
        match t {
            SampleTime::Cycles1C5 => Self::CYCLES1_5,
            SampleTime::Cycles2C5 => Self::CYCLES2_5,
            SampleTime::Cycles4C5 => Self::CYCLES4_5,
            SampleTime::Cycles7C5 => Self::CYCLES7_5,
            SampleTime::Cycles19C5 => Self::CYCLES19_5,
            SampleTime::Cycles61C5 => Self::CYCLES61_5,
            SampleTime::Cycles181C5 => Self::CYCLES181_5,
            SampleTime::Cycles601C5 => Self::CYCLES601_5,
        }
    }
}

impl From<SMP1_A> for SampleTime {
    fn from(t: SMP1_A) -> Self {
        match t {
            SMP1_A::CYCLES1_5 => Self::Cycles1C5,
            SMP1_A::CYCLES2_5 => Self::Cycles2C5,
            SMP1_A::CYCLES4_5 => Self::Cycles4C5,
            SMP1_A::CYCLES7_5 => Self::Cycles7C5,
            SMP1_A::CYCLES19_5 => Self::Cycles19C5,
            SMP1_A::CYCLES61_5 => Self::Cycles61C5,
            SMP1_A::CYCLES181_5 => Self::Cycles181C5,
            SMP1_A::CYCLES601_5 => Self::Cycles601C5,
        }
    }
}

impl From<SMP10_A> for SampleTime {
    fn from(t: SMP10_A) -> Self {
        match t {
            SMP10_A::CYCLES1_5 => Self::Cycles1C5,
            SMP10_A::CYCLES2_5 => Self::Cycles2C5,
            SMP10_A::CYCLES4_5 => Self::Cycles4C5,
            SMP10_A::CYCLES7_5 => Self::Cycles7C5,
            SMP10_A::CYCLES19_5 => Self::Cycles19C5,
            SMP10_A::CYCLES61_5 => Self::Cycles61C5,
            SMP10_A::CYCLES181_5 => Self::Cycles181C5,
            SMP10_A::CYCLES601_5 => Self::Cycles601C5,
        }
    }
}

/// DMA mode
///
/// # Note
///
/// The DMA transfer requests are blocked until the software clears the [`Event::Overrun`] event.
///
/// [`Event::Overrun`]: `super::Event::Overrun`
///
/// More details [RM0316] 15.3.26
///
/// [RM0316]: https://www.st.com/resource/en/reference_manual/dm00094349.pdf
///
/// # Related functions
///
/// * [`Adc::set_dma_mode`]
/// * [`Adc::dma_mode`]
///
/// [`Adc::set_dma_mode`]: `super::Adc::set_dma_mode`
/// [`Adc::dma_mode`]: `super::Adc::dma_mode`
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaMode {
    /// DMA is disabled.
    Disabled,
    /// In this mode, the ADC generates a DMA transfer request each time a new conversion data
    /// is available and stops generating DMA requests once the DMA has reached the last DMA
    /// transfer (when DMA end of transmission interrupt occurs) even if a conversion
    /// has been started again.
    ///
    /// See [`Event::EndOfSequence`]
    ///
    /// [`Event::EndOfSequence`]: `super::Event::EndOfSequence`
    OneShot,
    /// In this mode, the ADC generates a DMA transfer request each time a new conversion data is
    /// available in the data register, even if the DMA has reached the last DMA transfer. This
    /// allows configuring the DMA in circular mode to handle a continuous analog input data
    /// stream.
    Circular,
}

impl Default for DmaMode {
    fn default() -> Self {
        Self::Disabled
    }
}

/// The conversion mode the ADC peripheral is operating in.
///
/// The [`Adc`] can be configured through [`Adc::set_conversion_mode`] and obtained via
/// [`Adc::conversion_mode`].
///
/// A conversion is either started by [`Adc::start_conversion`],
/// or by an external trigger ([`ExternalTrigger`]).
///
/// # TLDR
///
/// * [`ConversionMode::Single`] once triggered goes through the whole sequence then stops.
/// * [`ConversionMode::Continuous`] once triggered goes through the whole sequence and repeats
///   that process until activly stopped (e.g. by calling [`Adc::start_conversion.`]
///   [`ConversionMode::Single`])
/// * [`ConversionMode::Discontinuous`] once triggered goes through part of the sequence in the specified step width
///   `n + 1` and has to be activly triggered to continue.
///
/// [`Adc::start_conversion`]: `super::Adc::start_conversion`
/// [`Adc::stop_conversion`]: `super::Adc::stop_conversion`
/// [`Adc::set_conversion_mode`]: `super::Adc::set_conversion_mode`
/// [`Adc::conversion_mode`]: `super::Adc::conversion_mode`
/// [`Adc`]: `super::Adc`
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConversionMode {
    /// In Single conversion mode, the ADC performs once all the conversions
    /// of the channels.
    ///
    /// # Start Conditions
    ///
    /// Inside the regular sequence, after each conversion is complete:
    ///
    /// * The converted data are stored into the 16-bit [data register](`super::Adc::data_register`)
    /// * The [`Event::EndOfConversion`] event is set.
    /// * An interrupt is generated if an interrupt is enabled for [`Event::EndOfConversion`].
    ///   ([`Adc::enable_interrupt`])
    ///
    /// After the regular sequence is complete:
    ///
    /// * The [`Event::EndOfSequence`] event is set.
    /// * An interrupt is generated if an interrupt is enabled for [`Event::EndOfSequence`].
    ///   ([`Adc::enable_interrupt`])
    ///
    /// [`Adc::enable_interrupt`]: `super::Adc::enable_interrupt`
    /// [`Event::EndOfConversion`]: `super::Event::EndOfConversion`
    /// [`Event::EndOfSequence`]: `super::Event::EndOfSequence`
    Single,
    /// In continuous conversion mode, when a software or hardware regular trigger event occurs, the
    /// ADC performs once all the regular conversions of the channels and then automatically
    /// re-starts and continuously converts each conversions of the sequence.
    ///
    /// # Note
    ///
    /// This mode applies to regular channls only.
    ///
    /// # Start conditions
    ///
    /// Inside the regular sequence, after each conversion is complete:
    ///
    /// * The converted data are stored into the 16-bit [data register](`super::Adc::data_register`)
    /// * The [`Event::EndOfConversion`] event is set.
    /// * An interrupt is generated if an interrupt is enabled for [`Event::EndOfConversion`].
    ///   ([`Adc::enable_interrupt`])
    ///
    /// After the sequence of conversions is complete:
    ///
    /// * The [`Event::EndOfSequence`] event is set.
    /// * An interrupt is generated if an interrupt is enabled for [`Event::EndOfSequence`].
    ///   ([`Adc::enable_interrupt`])
    ///
    /// [`Adc::enable_interrupt`]: `super::Adc::enable_interrupt`
    /// [`Event::EndOfConversion`]: `super::Event::EndOfConversion`
    /// [`Event::EndOfSequence`]: `super::Event::EndOfSequence`
    Continuous,
    /// It is used to convert a short sequence (sub-group) of `n + 1` conversions (`n` ≤ 8) that is
    /// part of the sequence of conversions selected via [`Adc::set_channel_sequence`]. The value
    /// is specified by the associated enum value `Discontinuous(n)`.
    ///
    /// When an [`ExternalTrigger`] occurs, it starts the next `n + 1` conversions selected with
    /// [`Adc::set_channel_sequence`] registers until all the conversions in the sequence
    /// are done. The total sequence length is defined by [`Adc::set_sequence_length`]
    ///
    /// [external trigger]: `ExternalTrigger`
    /// [`Adc::set_sequence_length`]: `super::Adc::set_sequence_length`
    /// [`Adc::set_channel_sequence`]: `super::Adc::set_channel_sequence_position`
    ///
    /// # Example
    ///
    /// ## `Discontinuous(2)`
    ///
    /// Channels to be converted = 1, 2, 3, 6, 7, 8, 9, 10, 11
    ///
    /// * 1st trigger:
    ///   * channels converted are 1, 2, 3 (an [`Event::EndOfConversion`] is generated at each conversion).
    /// * 2nd trigger:
    ///   * channels converted are 6, 7, 8 (an [`Event::EndOfConversion`] is generated at each conversion).
    /// * 3rd trigger:
    ///   * channels converted are 9, 10, 11 (an [`Event::EndOfConversion`] is generated at each conversion)
    ///     and an [`Event::EndOfSequence`] event is generated after the conversion of channel 11.
    /// * 4th trigger:
    ///   * channels converted are 1, 2, 3 (an [`Event::EndOfConversion`] is generated at each conversion).
    /// * ...
    ///
    /// ## `Continuous`
    ///
    /// Channels to be converted = 1, 2, 3, 6, 7, 8, 9, 10,11
    ///
    /// * 1st trigger:
    ///   * The complete sequence is converted: channel 1, then 2, 3, 6, 7, 9, 10
    ///     and 11. Each conversion generates an [`Event::EndOfConversion`] event and the last one also generates
    ///     an [`Event::EndOfSequence`] event.
    /// – All the next trigger events will relaunch the complete sequence
    ///
    /// # Note
    ///
    /// When a regular group is converted in [`ConversionMode::Discontinuous`], no rollover occurs
    /// (the last subgroup of the sequence can have less than `n` conversions).
    /// When all subgroups are converted, the next trigger starts the conversion of the first subgroup.
    /// In the example above, the 4th trigger reconverts the channels 1, 2 and 3 in the 1st subgroup.
    ///
    /// [`Event::EndOfConversion`]: `super::Event::EndOfConversion`
    /// [`Event::EndOfSequence`]: `super::Event::EndOfSequence`
    Discontinuous(u8),
    // TODO(Sh3rm4n): Support Scan Mode (which might only be supported by stm32f37x devices?)
    // Scan,
}

impl From<ConversionMode> for cfgr::CONT_A {
    fn from(mode: ConversionMode) -> Self {
        match mode {
            ConversionMode::Single | ConversionMode::Discontinuous(_) => Self::SINGLE,
            ConversionMode::Continuous => Self::CONTINUOUS,
        }
    }
}

impl From<ConversionMode> for cfgr::DISCEN_A {
    fn from(mode: ConversionMode) -> Self {
        match mode {
            ConversionMode::Single | ConversionMode::Continuous => Self::DISABLED,
            ConversionMode::Discontinuous(_) => Self::ENABLED,
        }
    }
}

impl From<cfgr::CONT_A> for ConversionMode {
    fn from(mode: cfgr::CONT_A) -> Self {
        match mode {
            cfgr::CONT_A::SINGLE => ConversionMode::Single,
            cfgr::CONT_A::CONTINUOUS => ConversionMode::Continuous,
        }
    }
}

impl Default for ConversionMode {
    fn default() -> Self {
        ConversionMode::Single
    }
}

/// Configuration for the ADC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The bit-width resolution of the peripheral.
    pub resolution: Resolution,
    /// The data alignment in the data register.
    pub data_alignment: DataAlignment,
    /// The scan mode on how a conversion sequence is being progressed.
    #[cfg(feature = "svd-f373")]
    pub scan: Scan,
    /// The mode, how to handle overrun scenarios of the data register.
    pub overrun: OverrunMode,
    /// The external trigger, which can start a conversion automatically.
    pub external_trigger: Option<ExternalTrigger>,
    /// The conversion mode, which will determined how a confersion sequence is progressed.
    pub conversion: ConversionMode,
    /// Disables or enables the DMA for the peripheral, and configured how the DMA
    /// is reading from the ADC data register.
    pub dma: DmaMode,
}

impl Config {
    /// Change the resolution
    pub fn resolution(mut self, resolution: Resolution) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set the align mode
    pub fn align(mut self, align: DataAlignment) -> Self {
        self.data_alignment = align;
        self
    }

    /// Set the scan mode
    #[cfg(feature = "svd-f373")]
    pub fn scan(mut self, scan: Scan) -> Self {
        self.scan = scan;
        self
    }

    /// Set the overrun mode
    pub fn overrun_mode(mut self, mode: OverrunMode) -> Self {
        self.overrun = mode;
        self
    }

    /// Set the conversion mode
    pub fn conversion_mode(mut self, mode: ConversionMode) -> Self {
        self.conversion = mode;
        self
    }

    /// Enable external trigger and the trigger source
    pub fn external_trigger(mut self, trigger: Option<ExternalTrigger>) -> Self {
        self.external_trigger = trigger;
        self
    }

    /// Enable DMA and the operation mode, in which DMA will transfer data from the ADC peripheral.
    pub fn dma_mode(mut self, dma: DmaMode) -> Self {
        self.dma = dma;
        self
    }
}
