//! # Analog to Digital Converter.
//!
//! ## Examples
//!
//! Check out [examples/adc.rs].
//!
//! It can be built for the STM32F3Discovery running
//! `cargo build --example adc --features=stm32f303xc`
//!
//! [examples/adc.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.0/examples/adc.rs

use core::ops::Deref;
use core::{convert::TryInto, marker::PhantomData};

use cortex_m::asm;
use embedded_hal::adc::Channel;

#[track_caller]
unsafe fn unreachable_unchecked() -> ! {
    #[cfg(debug_assertions)]
    crate::unreachable!();
    #[cfg(not(debug_assertions))]
    #[allow(unused_unsafe)]
    unsafe {
        core::hint::unreachable_unchecked();
    }
}

use crate::{
    pac::{self, adc1, Interrupt},
    rcc::{Clocks, Enable, AHB},
    time::{duration::Microseconds, fixed_point::FixedPoint, rate::Hertz},
    Toggle,
};

cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f3x4")] {
        use crate::pac::adc_common as adc1_2;

        use adc1_2::ccr::CKMODE_A;
    } else {
        use crate::pac::{adc1_2, adc1_2::ccr::CKMODE_A};
    }
}

#[cfg(feature = "enumset")]
use enumset::{EnumSet, EnumSetType};

const MAX_ADVREGEN_STARTUP: Microseconds = Microseconds(10);

/// Internal voltage reference, which is also used for calibration.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(alias = "V_REFINT")]
pub struct VoltageInternalReference<ADC> {
    _adc: PhantomData<ADC>,
}

/// Battery voltage signal, used for monitoring the battery (if used)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(alias = "V_BAT")]
pub struct VoltageBattery<ADC> {
    _adc: PhantomData<ADC>,
}

/// Internal core temperature signal
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(alias = "V_TS")]
pub struct TemperatureSensor<ADC> {
    _adc: PhantomData<ADC>,
}

// TODO(Sh3Rm4n): As soon as OPAMPs are implemented
// /// Reference Voltage for the Operational Amplifier 2
// #[derive(Debug)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// #[doc(alias = "V_REFOPAMP2")]
// pub struct VoltageOpAmp2Reference(pub(crate) ());
//
// /// Reference Voltage for the Operational Amplifier 3
// #[derive(Debug)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// #[doc(alias = "V_REFOPAMP3")]
// #[cfg(all(
//     feature = "svd-f303",
//     any(feature = "gpio-f303", feature = "gpio-f303e")
// ))]
// pub struct VoltageOpAmp3Reference(pub(crate) ());
//
// /// Reference Voltage for the Operational Amplifier 4
// #[derive(Debug)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// #[doc(alias = "V_REFOPAMP4")]
// #[cfg(all(
//     feature = "svd-f303",
//     any(feature = "gpio-f303", feature = "gpio-f303e")
// ))]
// pub struct VoltageOpAmp4Reference(pub(crate) ());

/// The common ADC peripheral, which is shared between different concrete ADC peripheral.
///
/// For example [`CommonAdc<ADC1_2>`] shares control over [`pac::ADC1`] and [`pac::ADC2`].
///
/// This peripheral can control different parts, like enabling internal sensors (like temperature sensor)
/// or enable dual channel mode (which is not supported yet.)
pub struct CommonAdc<ADC> {
    reg: ADC,
}

impl<ADC> CommonAdc<ADC>
where
    ADC: CommonInstance,
{
    /// Create a [`CommonAdc`] instance, consuming the [`pac`]'s ADC (e.g. [`pac::ADC1_2`].
    pub fn new(mut common_adc: ADC, clocks: &Clocks, ahb: &mut AHB) -> Self {
        common_adc.enable_clock(clocks, ahb);

        Self { reg: common_adc }
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
    pub unsafe fn peripheral(&mut self) -> &mut ADC {
        &mut self.reg
    }
}

impl<ADC> CommonAdc<ADC>
where
    ADC: CommonInstance + crate::rcc::Enable,
{
    /// Releases the common ADC peripheral
    pub fn free(self, _adcs: &<ADC as CommonInstance>::Childs) -> ADC {
        cortex_m::interrupt::free(|_| {
            // SAFETY: Guaranteed to be the only instance left, which has control over the
            // `ADC`perpherals, and criticala section ensure that no race condition happens
            // on the `Bus` peripheral.
            unsafe { ADC::disable_unchecked() };
        });
        self.reg
    }
}

/// Interrupt and status events.
///
/// All events can be cleared by [`Adc::clear_event`] or [`Adc::clear_events`].
/// Some events are also cleared on other conditions.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
#[non_exhaustive]
pub enum Event {
    /// This event is set by hardware after the ADC has been enabled.
    /// and when the ADC reaches a state where it is ready to accept conversion requests.
    #[doc(alias = "ADRDY")]
    AdcReady,
    /// This event is set by hardware during the conversion of any channel (only for regular channels),
    /// at the end of the sampling phase.
    #[doc(alias = "EOSMP")]
    EndOfSamplingPhase,
    /// This event is set by hardware at the end of each regular conversion of a channel
    /// when a new data is available in the data register ([`Adc::data_register`]).
    #[doc(alias = "EOC")]
    EndOfConversion,
    /// This event is set by hardware at the end of the conversions of a regular sequence of channels.
    ///
    /// When this event will occur, will depend on the [`Adc::sequence_length`] or
    /// [`config::ConversionMode`].
    #[doc(alias = "EOS")]
    EndOfSequence,
    /// This event is set by hardware when an overrun occurs on a regular channel, meaning that a new
    /// conversion has completed while the [`Event::EndOfConversion`] flag was already set.
    ///
    /// # Note
    ///
    /// If [`OverrunMode::Preserve`] is configured and DMA is enabled ([`config::DmaMode`]),
    /// the DMA transfer requests are blocked until the event is cleared. ([`Adc::clear_event`])
    ///
    /// [`OverrunMode::Preserve`]: `config::OverrunMode::Preserve`
    #[doc(alias = "OVR")]
    Overrun,
    /// This event is set by hardware at the end of each injected conversion of a channel
    /// when a new data is available in the corresponding ADCx_JDRy register.
    #[doc(alias = "JEOC")]
    InjectedChannelEndOfConversion,
    /// This event is set by hardware at the end of the conversions of all injected channels in the group.
    #[doc(alias = "JEOS")]
    InjectedChannelEndOfSequence,
    /// This event is set by hardware when the converted voltage crosses the values programmed
    /// in the fields `LT1[11:0]` and `HT1[11:0]` of ADCx_TR1 register.
    // TODO(Sh3Rm4n): Improve doc
    #[doc(alias = "AWD1")]
    AnalogWatchdog1,
    /// This event is set by hardware when the converted voltage crosses the values programmed
    /// in the fields `LT2[7:0]` and `HT2[7:0]` of ADCx_TR2 register.
    // TODO(Sh3Rm4n): Improve doc
    #[doc(alias = "AWD2")]
    AnalogWatchdog2,
    /// This event is set by hardware when the converted voltage crosses the values programmed
    /// in the fields `LT3[7:0]` and `HT3[7:0]` of ADCx_TR3 register. I
    // TODO(Sh3Rm4n): Improve doc
    #[doc(alias = "AWD3")]
    AnalogWatchdog3,
    /// This event is set by hardware when an Overflow of the Injected Queue of Context occurs.
    #[doc(alias = "JQOVF")]
    InjectedContextQueueOverfow,
}

/// Analog Digital Converter Peripheral
#[derive(Debug)]
pub struct Adc<ADC, State = Enabled> {
    /// ADC Register
    reg: ADC,
    /// [`Enabled`] / [`Disabled`] type state
    state: PhantomData<State>,
}

/// Implements [`Channel`] for all pins
pub mod channel;

pub mod config;

/// Implements [`Channel`] trait for special pins
macro_rules! sp_channel {
    ([$(($Pin:ident, $ADC:ident, $chan:expr)),+ $(,)*]) => {
        $(
            impl Channel<pac::$ADC> for $Pin<<pac::$ADC as Instance>::SharedInstance> {
                type ID = channel::Id;
                fn channel() -> Self::ID { $chan }
            }
        )+
    };
}

macro_rules! sp_pins {
    ([$(($Pin:ident, $en:ident)),+ $(,)*]) => {
        $(
            impl<Common> $Pin<Common>
            where
                Common: CommonInstance,
            {
                /// Creating a type associated with the special ADC sensor, which
                /// implements the [`embedded_hal::adc::Channel`] trait.
                ///
                /// # Example
                ///
                /// Imagine you'd like to measure the core temperature via `ADC1`.
                ///
                /// In that case this function takes both the common ADC, e.g. [`pac::ADC1_2`], as
                /// well as both associated ADCs, e.g. [`pac::ADC1`] & [`pac::ADC2`] to ensure,
                /// that these ADCs are still disabled, because the internal ADC sensor can only be
                /// enabled, if these ADCs are disabled, see [RM0316] 15.6.2.
                ///
                /// Code example can be seen in [examples/adc.rs].
                ///
                /// [examples/adc.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.0/examples/adc.rs
                /// [RM0316]: https://www.st.com/resource/en/reference_manual/dm00094349.pdf
                // FIXME(Sh3Rm4n): Soundness hole: Still mutliple sensor objects can be created.
                // An idea might be to split out the sensors from CommonAdc similar to the
                // DevicePeripheral.
                #[inline]
                pub fn new(
                    // Does this now expect CommonAdc or pac::ADC1_2?
                    common_adc: &mut CommonAdc<Common>,
                    // &mut adc ensures, that adc is disabled and with the shard instance the reference can be
                    // configured / enabled.
                    // As of RM0316 15.6.2: Software is allowed to write this bit only when the ADCs are
                    // disabled (ADCAL=0, JADSTART=0, ADSTART=0, ADSTP=0, ADDIS=0 and ADEN=0)
                    // TODO(Sh3Rm4n): Childs is currently a tuple of the raw ADCs but could also be
                    // (Adc<ADC1, Disabled>, Adc<ADC2, Disabled>)
                    _adcs: &mut <Common as CommonInstance>::Childs,
                ) -> Self {
                    common_adc.reg.ccr.modify(|_, w| w.$en().enabled());
                    Self { _adc: PhantomData }
                }
            }
        )+
    };
}

// See RM0316 table 86.
sp_channel!([
    (TemperatureSensor, ADC1, channel::Id::Sixteen),
    (VoltageBattery, ADC1, channel::Id::Seventeen),
    (VoltageInternalReference, ADC1, channel::Id::Eighteen),
]);

#[cfg(any(feature = "svd-f302", feature = "svd-f303", feature = "svd-f3x4"))]
sp_channel!([(VoltageInternalReference, ADC2, channel::Id::Eighteen),]);

#[cfg(feature = "svd-f303")]
sp_channel!([
    (VoltageInternalReference, ADC3, channel::Id::Eighteen),
    (VoltageInternalReference, ADC4, channel::Id::Eighteen),
]);

sp_pins!([
    (TemperatureSensor, tsen),
    (VoltageBattery, vbaten),
    (VoltageInternalReference, vrefen),
]);

impl<ADC> Adc<ADC, Enabled>
where
    ADC: Instance,
{
    /// Initialize a new ADC peripheral.
    ///
    /// Enables the clock, performs a calibration and enables the ADC.
    #[inline]
    pub fn new(
        adc: ADC,
        config: impl Into<config::Config>,
        clocks: &Clocks,
        common_adc: &CommonAdc<<ADC as Instance>::SharedInstance>,
    ) -> Adc<ADC, Enabled> {
        let mut adc = Adc::new_disabled(adc);
        let config: config::Config = config.into();
        adc.set_config(config);
        adc.calibrate(clocks, common_adc);
        adc.into_enabled()
    }

    /// Returns the current sample stored in the ADC data register
    ///
    /// At the end of each regular conversion channel (when [`Event::EndOfConversion`] event occurs),
    /// the result of the converted data is stored into the data register.
    ///
    /// # Note
    ///
    /// This function does no error handling or configuration of the
    /// peripheral. The value read might be invalid, because the device
    /// is not configured accordingly..
    ///
    /// ## Embedded HAL
    ///
    /// There is a more managed way to read from the adc via the [`embedded_hal::adc::OneShot`] trait.
    #[doc(alias = "ADC_DR")]
    #[must_use]
    #[inline]
    pub fn data_register(&self) -> u16 {
        self.reg.dr.read().rdata().bits()
    }

    /// Returns the address of the ADC data register. Primarily useful for configuring DMA.
    // TODO(Sh3Rm4n): Check against and integrate to DMA
    #[inline]
    pub fn data_register_address(&self) -> u32 {
        &self.reg.dr as *const _ as u32
    }

    /// Manually start a conversion sequence.
    ///
    /// * To automatically start a conversion consider setting [`Adc::set_external_trigger`].
    /// * The conversion mode is configured via [`Adc::set_conversion_mode`].
    #[inline]
    pub fn start_conversion(&mut self) {
        self.reg.cr.modify(|_, w| w.adstart().start());
    }

    /// Configure and convert the [`Adc`] instance into [`OneShot`] mode.
    #[inline]
    #[must_use]
    pub fn into_oneshot(mut self) -> Adc<ADC, OneShot> {
        self.stop_conversion();
        while self.is_conversion_ongoing() {}
        self.set_conversion_mode(config::ConversionMode::Single);
        self.set_dma_mode(config::DmaMode::Disabled);
        self.set_external_trigger(None);

        self.disable_interrupt(Event::EndOfConversion);
        self.disable_interrupt(Event::EndOfSequence);

        self.set_sequence_length(config::Sequence::One);

        Adc {
            reg: self.reg,
            state: PhantomData,
        }
    }

    /// Disables the peripheral and convert it into [`Disabled`] mode.
    #[inline]
    #[must_use]
    pub fn into_disabled(self) -> Adc<ADC, Disabled> {
        // Software procedure to disable the ADC according to [RM0316]: 15.3.9

        // 1. Check that both ADSTART=0 and JADSTART=0 to ensure that no conversion is ongoing. If
        //    required, stop any regular and injected conversion ongoing by setting ADSTP=1 and
        //    JADSTP=1 and then wait until ADSTP=0 and JADSTP=0
        if self.reg.cr.read().aden().bit()
            && (self.reg.cr.read().adstart().bit() || self.reg.cr.read().jadstart().bit())
        {
            self.reg.cr.modify(|_, w| w.adstp().stop());
            // In auto-injection mode (JAUTO=1), setting ADSTP bit aborts both
            // regular and injected conversions (do not use JADSTP)
            if !self.reg.cfgr.read().jauto().is_enabled() {
                #[cfg(not(feature = "svd-f301"))]
                self.reg.cr.modify(|_, w| w.jadstp().stop());
                #[cfg(feature = "svd-f301")]
                self.reg.cr.modify(|_, w| w.jadst().set_bit());
            }
            #[cfg(not(feature = "svd-f301"))]
            while self.reg.cr.read().adstp().bit() || self.reg.cr.read().jadstp().bit() {}
            // TODO(Sh3Rm4n): https://github.com/stm32-rs/stm32-rs/pull/696
            #[cfg(feature = "svd-f301")]
            while self.reg.cr.read().adstp().bit() || self.reg.cr.read().jadst().bit() {}
        }

        // Software is allowed to set ADDIS only when ADEN=1 and both ADSTART=0
        // and JADSTART=0 (which ensures that no conversion is ongoing)
        if self.reg.cr.read().aden().is_enable() {
            // 2. Disable ADC
            // TODO(Sh3Rm4n): Use w.aaddis().disable() once https://github.com/stm32-rs/stm32-rs/pull/699 is merged
            self.reg.cr.modify(|_, w| w.addis().set_bit());
            // 3. Wait for ADC being disabled
            while self.reg.cr.read().aden().is_enable() {}
        }

        Adc {
            reg: self.reg,
            state: PhantomData,
        }
    }

    /// Releases the ADC peripheral
    #[inline]
    pub fn free(self) -> ADC {
        self.into_disabled().reg
    }
}

impl<ADC> Adc<ADC, Disabled>
where
    ADC: Instance,
{
    /// Create an [`Adc`] instance in [`Disabled`] mode.
    ///
    /// In this mode, the peripheral is not yet enabled, and neither guaranteed to be calibrated.
    /// To convert the [`Adc`] peripheral into [`Enabled`] mode, you can do the following:
    ///
    /// ```
    /// let clocks = rcc.cfgr.freeze(/*... */);
    /// let adc_common = CommonAdc::new(/*...*/);
    ///
    /// let mut adc = adc::Adc::new_disabled(dp.ADC1);
    /// adc.calibrate(&clocks, &adc_common); // Optional
    /// adc.set_config(adc::config::Config::default()); // Optional
    /// let adc: Adc<ADC1, Enabled> = adc2.into_enabled();
    /// ```
    #[inline]
    pub fn new_disabled(adc: ADC) -> Self {
        Self {
            reg: adc,
            state: PhantomData,
        }
    }

    /// Convert the [`Adc`] into [`Enabled`] mode.
    #[inline]
    #[must_use]
    pub fn into_enabled(self) -> Adc<ADC, Enabled> {
        // Enabled According to [RM0316] 15.3.9.
        // This check assumes, that the ADC was enabled before and it was waited until
        // ADRDY=1 was set.
        // This assumption is true, if the peripheral was initially enabled through
        // this method.
        if !self.reg.cr.read().aden().is_enable() {
            // Set ADEN=1
            self.reg.cr.modify(|_, w| w.aden().enable());
            // Wait until ADRDY=1 (ADRDY is set after the ADC startup time). This can be
            // done using the associated interrupt (setting ADRDYIE=1).
            while self.reg.isr.read().adrdy().is_not_ready() {}
        }

        Adc {
            reg: self.reg,
            state: PhantomData,
        }
    }

    /// Calibrate according to [RM0316] 15.3.8
    ///
    /// The internal analog calibration is lost each time the power of the ADC is removed.
    ///
    /// # Note
    ///
    /// The ADC is calibrated, when created via [`Adc::new`], so immidiate calibration after that
    /// should not be needed.
    ///
    /// [RM0316]: https://www.st.com/resource/en/reference_manual/dm00094349.pdf
    // TODO(Sh3Rm4n): CALFACT to save the calibration state
    #[inline]
    pub fn calibrate(
        &mut self,
        clocks: &Clocks,
        common_adc: &CommonAdc<<ADC as Instance>::SharedInstance>,
        // TODO(Sh3Rm4n): Would about concurrent calibrations between related ADCs?
        // int_ref: &mut VoltageInternalReference<ADC>,
    ) {
        self.configure_voltage_regulator(Toggle::On, clocks);

        self.reg
            .cr
            .modify(|_, w| w.adcaldif().single_ended().adcal().calibration());

        while self.reg.cr.read().adcal().is_calibration() {}

        // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is
        // cleared by hardware
        let adc_cycles = 4;
        let frequency = common_adc
            .reg
            .clock(clocks)
            .unwrap_or_else(|| clocks.sysclk());
        let cpu_cycles = adc_cycles * clocks.sysclk().0 / frequency.0;
        asm::delay(cpu_cycles);

        // When the internal voltage regulator is disabled, the internal analog calibration is kept
        self.configure_voltage_regulator(Toggle::Off, clocks);
    }

    /// Enable the interal voltage generator Blocks until regulator is enabled.
    fn configure_voltage_regulator(&mut self, toggle: impl Into<Toggle>, clocks: &Clocks) {
        let already_on = self.reg.cr.read().advregen().is_enabled();
        let toggle = toggle.into();
        self.reg.cr.modify(|_, w| w.advregen().intermediate());
        self.reg.cr.modify(|_, w| {
            w.advregen().variant(match toggle {
                Toggle::On => adc1::cr::ADVREGEN_A::ENABLED,
                Toggle::Off => adc1::cr::ADVREGEN_A::DISABLED,
            })
        });
        if toggle == Toggle::On && !already_on {
            let wait = MAX_ADVREGEN_STARTUP.integer()
                * clocks.sysclk().integer()
                * <Microseconds as FixedPoint>::SCALING_FACTOR;
            asm::delay(wait);
        }
    }

    /// Releases the ADC peripheral
    #[inline]
    pub fn free(self) -> ADC {
        self.reg
    }
}

impl<ADC, State> Adc<ADC, State>
where
    ADC: Instance,
{
    /// Get the current overrun mode.
    #[inline]
    pub fn overrun_mode(&self) -> config::OverrunMode {
        self.reg.cfgr.read().ovrmod().variant().into()
    }

    /// Get the sampling resolution.
    #[inline]
    pub fn resolution(&self) -> config::Resolution {
        self.reg.cfgr.read().res().variant().into()
    }

    /// Get the current configured data alignment.
    #[inline]
    pub fn data_alignment(&self) -> config::DataAlignment {
        self.reg.cfgr.read().align().variant().into()
    }

    /// Get the current configured external trigger.
    ///
    /// If no (valid) trigger is set, return `None`
    #[inline]
    pub fn external_trigger(&self) -> Option<config::ExternalTrigger> {
        use adc1::cfgr::{EXTEN_A, EXTSEL_A};
        use config::ExternalTrigger;
        let cfgr = self.reg.cfgr.read();

        match (cfgr.exten().variant(), cfgr.extsel().variant()) {
            (EXTEN_A::DISABLED, _) | (_, None) => None,
            (edge, Some(ext)) => {
                let edge = match edge {
                    EXTEN_A::DISABLED => {
                        // SAFETY: This arm can not be reached, because of the
                        // (EXTEN_A::DISABLED, _) arm above.
                        unsafe { unreachable_unchecked() }
                    }
                    EXTEN_A::RISINGEDGE => config::TriggerMode::RisingEdge,
                    EXTEN_A::FALLINGEDGE => config::TriggerMode::FallingEdge,
                    EXTEN_A::BOTHEDGES => config::TriggerMode::BothEdges,
                };

                Some(match ext {
                    EXTSEL_A::HRTIM_ADCTRG1 => ExternalTrigger::HrtimAdcTrg1(edge),
                    EXTSEL_A::HRTIM_ADCTRG3 => ExternalTrigger::HrtimAdcTrg3(edge),
                    EXTSEL_A::TIM1_CC1 => ExternalTrigger::Tim1Cc1(edge),
                    EXTSEL_A::TIM1_CC2 => ExternalTrigger::Tim1Cc2(edge),
                    EXTSEL_A::TIM1_CC3 => ExternalTrigger::Tim1Cc3(edge),
                    EXTSEL_A::TIM2_CC2 => ExternalTrigger::Tim2Cc2(edge),
                    EXTSEL_A::TIM3_TRGO => ExternalTrigger::Tim3Trgo(edge),
                    EXTSEL_A::EXTI11 => ExternalTrigger::Exti11(edge),
                    EXTSEL_A::TIM1_TRGO => ExternalTrigger::Tim1Trgo(edge),
                    EXTSEL_A::TIM1_TRGO2 => ExternalTrigger::Tim1Trgo2(edge),
                    EXTSEL_A::TIM2_TRGO => ExternalTrigger::Tim2Trgo(edge),
                    EXTSEL_A::TIM6_TRGO => ExternalTrigger::Tim6Trgo(edge),
                    EXTSEL_A::TIM15_TRGO => ExternalTrigger::Tim15Trgo(edge),
                    EXTSEL_A::TIM3_CC4 => ExternalTrigger::Tim3Cc4(edge),
                })
            }
        }
    }

    /// Return the conversion mode the peripheral is currently operating in.
    #[inline]
    pub fn conversion_mode(&self) -> config::ConversionMode {
        use adc1::cfgr::{CONT_A, DISCEN_A};
        let cfgr = self.reg.cfgr.read();

        match (
            cfgr.cont().variant(),
            cfgr.discen().variant(),
            cfgr.discnum().bits(),
        ) {
            (CONT_A::SINGLE, DISCEN_A::DISABLED, _) => config::ConversionMode::Single,
            (CONT_A::CONTINUOUS, DISCEN_A::DISABLED, _) => config::ConversionMode::Continuous,
            // It is not possible to have both discontinuous mode and continuous mode enabled. In this
            // case (if DISCEN=1, CONT=1), the ADC behaves as if continuous mode was disabled.
            // [RM0316 15.3.20]
            (_, DISCEN_A::ENABLED, n) => config::ConversionMode::Discontinuous(n),
        }
    }

    /// Get the current configured DMA mode.
    #[inline]
    pub fn dma_mode(&self) -> config::DmaMode {
        let cfgr = self.reg.cfgr.read();
        use adc1::cfgr::{DMACFG_A, DMAEN_A};
        match (cfgr.dmaen().variant(), cfgr.dmacfg().variant()) {
            (DMAEN_A::DISABLED, _) => config::DmaMode::Disabled,
            (DMAEN_A::ENABLED, DMACFG_A::ONESHOT) => config::DmaMode::OneShot,
            (DMAEN_A::ENABLED, DMACFG_A::CIRCULAR) => config::DmaMode::Circular,
        }
    }

    #[inline]
    #[cfg(feature = "svd-f373")]
    // TODO(Sh3Rm4n): This is dead code, becuase svd-f373 is not supported as of now.
    pub fn scan(&self) -> config::Scan {
        self.reg.cr1.modify(|_, w| w.scan().variant(scan.into()));
    }

    /// Get the current [`config::Config`] the peripheral is configured to.
    pub fn config(&self) -> config::Config {
        config::Config {
            resolution: self.resolution(),
            data_alignment: self.data_alignment(),
            #[cfg(feature = "svd-f373")]
            scan: self.scan(),
            overrun: self.overrun_mode(),
            external_trigger: self.external_trigger(),
            conversion: self.conversion_mode(),
            dma: self.dma_mode(),
            // TODO(Sh3Rm4n): Voltage reference?
        }
    }

    /// Check if a conversion is ongoing.
    ///
    /// * A conversion can not only be started manually via [`Adc::start_conversion`], but
    ///   also by an [`config::ExternalTrigger`].
    /// * Stopping a conversion might be needed to be able to configure the peripheral,
    ///   which is not allowed during a conversion.
    /// * Alternativly a conversino stop is signaled via the [`Event::EndOfConversion`] event.
    #[inline]
    pub fn is_conversion_ongoing(&self) -> bool {
        self.reg.cr.read().adstart().is_start()
    }

    /// Get the [`channel::Id`] at a specific [`config::Sequence`] position.
    ///
    /// Return [`None`] when no channel was set at the sequence position.
    ///
    /// # Example
    ///
    /// Getting the pin in a specific sequence position
    ///
    /// ```
    /// let pin_id: u8 = adc.channel_sequence(adc::config::Sequence::Seven).unwrap().into();
    /// ```
    #[inline]
    #[rustfmt::skip]
    pub fn channel_sequence(&self, sequence: config::Sequence) ->  Option<channel::Id> {
        // Set the channel in the right sequence field
        match sequence {
            config::Sequence::One      => self.reg.sqr1.read().sq1().bits().try_into().ok(),
            config::Sequence::Two      => self.reg.sqr1.read().sq2().bits().try_into().ok(),
            config::Sequence::Three    => self.reg.sqr1.read().sq3().bits().try_into().ok(),
            config::Sequence::Four     => self.reg.sqr1.read().sq4().bits().try_into().ok(),
            config::Sequence::Five     => self.reg.sqr2.read().sq5().bits().try_into().ok(),
            config::Sequence::Six      => self.reg.sqr2.read().sq6().bits().try_into().ok(),
            config::Sequence::Seven    => self.reg.sqr2.read().sq7().bits().try_into().ok(),
            config::Sequence::Eight    => self.reg.sqr2.read().sq8().bits().try_into().ok(),
            config::Sequence::Nine     => self.reg.sqr2.read().sq9().bits().try_into().ok(),
            config::Sequence::Ten      => self.reg.sqr3.read().sq10().bits().try_into().ok(),
            config::Sequence::Eleven   => self.reg.sqr3.read().sq11().bits().try_into().ok(),
            config::Sequence::Twelve   => self.reg.sqr3.read().sq12().bits().try_into().ok(),
            config::Sequence::Thirteen => self.reg.sqr3.read().sq13().bits().try_into().ok(),
            config::Sequence::Fourteen => self.reg.sqr3.read().sq14().bits().try_into().ok(),
            config::Sequence::Fifteen  => self.reg.sqr4.read().sq15().bits().try_into().ok(),
            config::Sequence::Sixteen  => self.reg.sqr4.read().sq16().bits().try_into().ok(),
        }
    }

    /// Return the current configured sample time for the given pin.
    // TODO: Sample_time or sampling_time (same for the type)
    pub fn sample_time<Pin>(&self, _pin: &Pin) -> config::SampleTime
    where
        Pin: Channel<ADC, ID = channel::Id>,
    {
        let channel = Pin::channel();
        // Set the sample time for the channel
        match channel {
            #[cfg(feature = "gpio-f373")]
            channel::Id::Zero => self.adc.smpr1.read().smp0().variant().into(),
            channel::Id::One => self.reg.smpr1.read().smp1().variant().into(),
            channel::Id::Two => self.reg.smpr1.read().smp2().variant().into(),
            channel::Id::Three => self.reg.smpr1.read().smp3().variant().into(),
            channel::Id::Four => self.reg.smpr1.read().smp4().variant().into(),
            channel::Id::Five => self.reg.smpr1.read().smp5().variant().into(),
            channel::Id::Six => self.reg.smpr1.read().smp6().variant().into(),
            channel::Id::Seven => self.reg.smpr1.read().smp7().variant().into(),
            channel::Id::Eight => self.reg.smpr1.read().smp8().variant().into(),
            channel::Id::Nine => self.reg.smpr1.read().smp9().variant().into(),
            channel::Id::Ten => self.reg.smpr2.read().smp10().variant().into(),
            channel::Id::Eleven => self.reg.smpr2.read().smp11().variant().into(),
            channel::Id::Twelve => self.reg.smpr2.read().smp12().variant().into(),
            channel::Id::Thirteen => self.reg.smpr2.read().smp13().variant().into(),
            channel::Id::Fourteen => self.reg.smpr2.read().smp14().variant().into(),
            channel::Id::Fifteen => self.reg.smpr2.read().smp15().variant().into(),
            channel::Id::Sixteen => self.reg.smpr2.read().smp16().variant().into(),
            channel::Id::Seventeen => self.reg.smpr2.read().smp17().variant().into(),
            channel::Id::Eighteen => self.reg.smpr2.read().smp18().variant().into(),
            // #[cfg(not(feature = "gpio-f373"))]
            // // SAFETY: We know, that channel IDs will not exceed 18, see [`crate::adc::channel`]
            // _ => unsafe { unreachable_unchecked() },
        }
    }

    /// Returns the total number of conversions in a regular channel sequence.
    #[inline]
    pub fn sequence_length(&self) -> config::Sequence {
        match self.reg.sqr1.read().l().bits().try_into() {
            Ok(seq) => seq,
            // SAFETY: We are directly reading from the register, which can't give
            // use back invalid values, so this should never be called.
            Err(_) => unsafe { unreachable_unchecked() },
        }
    }

    #[inline]
    #[cfg(feature = "__disabled")]
    pub fn is_injected_conversion_stopped(&self) -> bool {
        self.adc.cr.read().jadstp().is_stop()
    }

    /// Check if an interrupt is configured for the [`Event`]
    #[inline]
    pub fn is_interrupt_configured(&self, event: Event) -> bool {
        match event {
            Event::AdcReady => self.reg.ier.read().adrdyie().is_enabled(),
            Event::EndOfSamplingPhase => self.reg.ier.read().eosmpie().is_enabled(),
            Event::EndOfConversion => self.reg.ier.read().eocie().is_enabled(),
            Event::EndOfSequence => self.reg.ier.read().eosie().is_enabled(),
            Event::Overrun => self.reg.ier.read().ovrie().is_enabled(),
            Event::InjectedChannelEndOfConversion => self.reg.ier.read().jeocie().is_enabled(),
            Event::InjectedChannelEndOfSequence => self.reg.ier.read().jeosie().is_enabled(),
            Event::AnalogWatchdog1 => self.reg.ier.read().awd1ie().is_enabled(),
            Event::AnalogWatchdog2 => self.reg.ier.read().awd2ie().is_enabled(),
            Event::AnalogWatchdog3 => self.reg.ier.read().awd3ie().is_enabled(),
            Event::InjectedContextQueueOverfow => self.reg.ier.read().jqovfie().is_enabled(),
        }
    }

    /// Check which interrupts are enabled for all [`Event`]s
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    #[inline]
    pub fn configured_interrupts(&mut self) -> EnumSet<Event> {
        let mut events = EnumSet::new();

        for event in EnumSet::<Event>::all().iter() {
            if self.is_interrupt_configured(event) {
                events |= event;
            }
        }

        events
    }

    /// Check if an interrupt event happend.
    #[inline]
    pub fn is_event_triggered(&self, event: Event) -> bool {
        let isr = self.reg.isr.read();
        match event {
            Event::AdcReady => isr.adrdy().is_ready(),
            Event::EndOfSamplingPhase => isr.eosmp().is_ended(),
            Event::EndOfConversion => isr.eoc().is_complete(),
            Event::EndOfSequence => isr.eos().is_complete(),
            Event::Overrun => isr.ovr().is_overrun(),
            Event::InjectedChannelEndOfConversion => isr.jeoc().is_complete(),
            Event::InjectedChannelEndOfSequence => isr.jeos().is_complete(),
            Event::AnalogWatchdog1 => isr.awd1().is_event(),
            Event::AnalogWatchdog2 => isr.awd2().is_event(),
            Event::AnalogWatchdog3 => isr.awd3().is_event(),
            Event::InjectedContextQueueOverfow => isr.jqovf().is_overflow(),
        }
    }

    /// Get an [`EnumSet`] of all fired interrupt events.
    ///
    /// # Examples
    ///
    /// This allows disabling all fired event at once, via the enum set abstraction, like so
    ///
    /// ```rust
    /// for event in adc.events() {
    ///     adc.configure_interrupt(event, false);
    /// }
    /// ```
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn triggered_events(&self) -> EnumSet<Event> {
        let mut events = EnumSet::new();

        for event in EnumSet::<Event>::all().iter() {
            if self.is_event_triggered(event) {
                events |= event;
            }
        }

        events
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
    #[inline]
    pub unsafe fn peripheral(&mut self) -> &mut ADC {
        &mut self.reg
    }
}

impl<ADC, State> Adc<ADC, State>
where
    ADC: Instance,
    State: Configurable,
{
    /// Apply a whole new [`config::Config`] to the peripheral.
    pub fn set_config(&mut self, config: config::Config) {
        self.set_resolution(config.resolution);
        self.set_data_alignment(config.data_alignment);
        #[cfg(feature = "svd-f373")]
        self.set_scan(config.scan);
        self.set_overrun_mode(config.overrun);
        self.set_external_trigger(config.external_trigger);
        self.set_conversion_mode(config.conversion);
        self.set_dma_mode(config.dma);
    }

    /// Set the overrun mode
    #[inline]
    pub fn set_overrun_mode(&mut self, mode: config::OverrunMode) {
        self.reg.cfgr.modify(|_, w| w.ovrmod().variant(mode.into()))
    }

    /// Sets the sampling resolution.
    #[inline]
    pub fn set_resolution(&mut self, resolution: config::Resolution) {
        self.reg
            .cfgr
            .modify(|_, w| w.res().variant(resolution.into()));
    }

    /// Sets the data register alignment
    #[inline]
    pub fn set_data_alignment(&mut self, align: config::DataAlignment) {
        self.reg.cfgr.modify(|_, w| w.align().variant(align.into()));
    }

    // TODO(Sh3Rm4n): setting offset might be useful, see RM0316 15.3.26
    // #[inline]
    // pub fn set_channel_offset(&self) {}

    /// Set an external trigger for a conversion.
    ///
    /// This is an alternative to starting a conversion by software ([`Adc::start_conversion`])
    #[inline]
    // Allow external trigger beeing None if trigger mode is disabled?
    pub fn set_external_trigger(&mut self, trigger: Option<config::ExternalTrigger>) {
        // Only allowed to write when no conversion is ongoing
        self.stop_conversion();

        self.reg.cfgr.modify(|_, w| {
            if let Some(ext) = trigger {
                w.extsel().variant(ext.into()).exten().variant(ext.into())
            } else {
                w.exten().disabled()
            }
        });
    }

    /// Set the conversion mode the ADC peripheral is operating in.
    ///
    /// This mode is the heart of how the ADC is operating and what it is used for.
    /// For a better explenatnion of the modes, read the documentation of [`config::ConversionMode`].
    ///
    /// E.g. for the [`embedded_hal::adc::OneShot`] trait this peripheral is operating in
    /// [`config::ConversionMode::Single`], while, when using the ADC for DMA or similar,
    /// [`config::ConversionMode::Continuous`] makes more sense. (DMA itself is configured via
    /// [`Adc::set_dma_mode`]).
    #[inline]
    pub fn set_conversion_mode(&mut self, conversion_mode: config::ConversionMode) {
        // TODO: If we don't support 0, to what mode should the periperhal be set?
        self.reg.cfgr.modify(|_, w| {
            w.discen()
                .variant(conversion_mode.into())
                .cont()
                .variant(conversion_mode.into())
        });

        if let config::ConversionMode::Discontinuous(n) = conversion_mode {
            self.reg
                .cfgr
                .modify(|_, w| w.discnum().bits(if n < 0b111 { n } else { 0b111 }));
        }
    }

    /// Sets DMA to disabled, single or continuous
    // TODO: Check against DMA, so that it is working
    #[inline]
    pub fn set_dma_mode(&mut self, dma: config::DmaMode) {
        use adc1::cfgr::{DMACFG_A, DMAEN_A};
        let (en, mode) = match dma {
            config::DmaMode::Disabled => (DMAEN_A::DISABLED, DMACFG_A::ONESHOT),
            config::DmaMode::OneShot => (DMAEN_A::ENABLED, DMACFG_A::ONESHOT),
            config::DmaMode::Circular => (DMAEN_A::ENABLED, DMACFG_A::CIRCULAR),
        };

        self.reg
            .cfgr
            .modify(|_, w| w.dmaen().variant(en).dmacfg().variant(mode));
    }

    /// Enables and disables scan mode
    #[inline]
    #[cfg(feature = "svd-f373")]
    // TODO(Sh3Rm4n): This is dead code, becuase svd-f373 is not supported as of now.
    // * Rename to set_scan_mode or merge with conversion Mode?
    // * Is this a [`Event`]?
    pub fn set_scan(&mut self, scan: config::Scan) {
        self.reg.cr1.modify(|_, w| w.scan().variant(scan.into()));
    }

    /// Signal the peripheral to stop any ongoing conversion
    ///
    /// This function will **block** until the conversion is stopped.
    ///
    /// For further details how stop is working, see [RM0316] 15.3.17
    ///
    /// [RM0316]: https://www.st.com/resource/en/reference_manual/dm00094349.pdf
    #[inline]
    pub fn stop_conversion(&mut self) {
        // If no conversion is ongoing, this may lead to the stop bit never beeing reset?
        if self.is_conversion_ongoing() {
            self.reg.cr.modify(|_, w| w.adstp().stop());
        }

        while self.reg.cr.read().adstp().bit_is_set() {}
    }

    /// Wraps [`Adc::set_channel_sequence_position`] to be able to take a pin.
    #[inline]
    pub fn set_pin_sequence_position<Pin>(&mut self, sequence: config::Sequence, _pin: &mut Pin)
    where
        Pin: Channel<ADC, ID = channel::Id>,
    {
        let channel = Pin::channel();

        // SAFETY: We have the Pin not the channel, so it should be safe to set the corresponding
        // channel to the pin.
        unsafe { self.set_channel_sequence_position(sequence, channel) };
    }

    /// Configure the sequence position of a channel.
    ///
    /// # Note
    ///
    /// * The [`Adc::sequence_length`] will be increased, if [`Sequence`] < [`Adc::sequence_length`].
    /// * To configure a channel, any ongoing conversion will be stopped ([`Adc::stop_conversion`])
    ///
    /// # Safety
    ///
    /// This function can not guarantee, that one channel is not used by multiple ADCs at similar
    /// times. For example [`channel::Id::Eighteen`], which is the [`VoltageInternalReference`].
    ///
    /// It can also not guarantee that the corresponding GPIO pin was set into analog mode.
    ///
    /// For the **safe** variant use [`Adc::set_pin_sequence_position`].
    ///
    /// [`ConversionMode::Continuous`]: `config::ConversionMode::Continuous`
    /// [`Sequence`]: `config::Sequence`
    ///
    #[inline]
    #[rustfmt::skip]
    pub unsafe fn set_channel_sequence_position(&mut self, sequence: config::Sequence, channel: channel::Id) {
        self.stop_conversion();

        // Increase the Sequence lenght, if it is not long enough
        if self.sequence_length() < sequence {
            self.set_sequence_length(sequence);
        }

        // Set the channel in the right sequence field
        match sequence {
            config::Sequence::One      => self.reg.sqr1.modify(|_, w| w.sq1().bits(channel.into())),
            config::Sequence::Two      => self.reg.sqr1.modify(|_, w| w.sq2().bits(channel.into())),
            config::Sequence::Three    => self.reg.sqr1.modify(|_, w| w.sq3().bits(channel.into())),
            config::Sequence::Four     => self.reg.sqr1.modify(|_, w| w.sq4().bits(channel.into())),
            config::Sequence::Five     => self.reg.sqr2.modify(|_, w| w.sq5().bits(channel.into())),
            config::Sequence::Six      => self.reg.sqr2.modify(|_, w| w.sq6().bits(channel.into())),
            config::Sequence::Seven    => self.reg.sqr2.modify(|_, w| w.sq7().bits(channel.into())),
            config::Sequence::Eight    => self.reg.sqr2.modify(|_, w| w.sq8().bits(channel.into())),
            config::Sequence::Nine     => self.reg.sqr2.modify(|_, w| w.sq9().bits(channel.into())),
            config::Sequence::Ten      => self.reg.sqr3.modify(|_, w| w.sq10().bits(channel.into())),
            config::Sequence::Eleven   => self.reg.sqr3.modify(|_, w| w.sq11().bits(channel.into())),
            config::Sequence::Twelve   => self.reg.sqr3.modify(|_, w| w.sq12().bits(channel.into())),
            config::Sequence::Thirteen => self.reg.sqr3.modify(|_, w| w.sq13().bits(channel.into())),
            config::Sequence::Fourteen => self.reg.sqr3.modify(|_, w| w.sq14().bits(channel.into())),
            config::Sequence::Fifteen  => self.reg.sqr4.modify(|_, w| w.sq15().bits(channel.into())),
            config::Sequence::Sixteen  => self.reg.sqr4.modify(|_, w| w.sq16().bits(channel.into())),
        }
    }

    /// Each channel can be sampled with a different sampling time.
    ///
    /// # Note
    ///
    /// Before starting a conversion, the ADC must establish a direct connection between the
    /// voltage source under measurement and the embedded sampling capacitor of the ADC.
    /// This sampling time must be enough for the input voltage source to charge the embedded
    /// capacitor to the input voltage level.
    #[rustfmt::skip]
    pub fn set_sample_time<Pin>(&mut self, _pin: &Pin, sample_time: config::SampleTime)
    where
        Pin: Channel<ADC, ID = channel::Id>,
    {
        let channel = Pin::channel();
        // Set the sample time for the channel
        match channel {
            #[cfg(feature = "gpio-f373")]
            channel::Id::Zero     => self.adc.smpr1.modify(|_, w| w.smp0().variant(sample_time.into())),
            channel::Id::One      => self.reg.smpr1.modify(|_, w| w.smp1().variant(sample_time.into())),
            channel::Id::Two      => self.reg.smpr1.modify(|_, w| w.smp2().variant(sample_time.into())),
            channel::Id::Three    => self.reg.smpr1.modify(|_, w| w.smp3().variant(sample_time.into())),
            channel::Id::Four     => self.reg.smpr1.modify(|_, w| w.smp4().variant(sample_time.into())),
            channel::Id::Five     => self.reg.smpr1.modify(|_, w| w.smp5().variant(sample_time.into())),
            channel::Id::Six      => self.reg.smpr1.modify(|_, w| w.smp6().variant(sample_time.into())),
            channel::Id::Seven    => self.reg.smpr1.modify(|_, w| w.smp7().variant(sample_time.into())),
            channel::Id::Eight    => self.reg.smpr1.modify(|_, w| w.smp8().variant(sample_time.into())),
            channel::Id::Nine     => self.reg.smpr1.modify(|_, w| w.smp9().variant(sample_time.into())),
            channel::Id::Ten      => self.reg.smpr2.modify(|_, w| w.smp10().variant(sample_time.into())),
            channel::Id::Eleven   => self.reg.smpr2.modify(|_, w| w.smp11().variant(sample_time.into())),
            channel::Id::Twelve   => self.reg.smpr2.modify(|_, w| w.smp12().variant(sample_time.into())),
            channel::Id::Thirteen => self.reg.smpr2.modify(|_, w| w.smp13().variant(sample_time.into())),
            channel::Id::Fourteen => self.reg.smpr2.modify(|_, w| w.smp14().variant(sample_time.into())),
            channel::Id::Fifteen  => self.reg.smpr2.modify(|_, w| w.smp15().variant(sample_time.into())),
            channel::Id::Sixteen  => self.reg.smpr2.modify(|_, w| w.smp16().variant(sample_time.into())),
            channel::Id::Seventeen => self.reg.smpr2.modify(|_, w| w.smp17().variant(sample_time.into())),
            channel::Id::Eighteen => self.reg.smpr2.modify(|_, w| w.smp18().variant(sample_time.into())),
            // #[cfg(not(feature = "gpio-f373"))]
            // _ => () // Make it a no-op for channels which are not available.
        }
    }

    /// Define the total number of conversions in the regular channel conversion sequence.
    ///
    /// The end of the whole sequence is notfied via the [`Event::EndOfSequence`], while
    /// the end of a single conversion of a "slot" is notfied via [`Event::EndOfConversion`].
    #[inline]
    pub fn set_sequence_length(&mut self, sequence: config::Sequence) {
        self.reg.sqr1.modify(|_, w| w.l().bits(sequence.into()))
    }

    // TODO(Sh3Rm4n): Implement, when injection mode is implemented.
    #[inline]
    #[cfg(feature = "__disabled")]
    pub fn stop_injected_conversion(&mut self) {
        self.adc.cr.modify(|_, w| w.jadstp().stop());
    }

    /// Enable the interrupt for the specified [`Event`].
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, Toggle::On);
    }

    /// Disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn disable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, Toggle::Off);
    }

    /// Enable or disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn configure_interrupt(&mut self, event: Event, enable: impl Into<Toggle>) {
        // Do a round way trip to be convert Into<Toggle> -> bool
        let enable: Toggle = enable.into();
        let enable: bool = enable.into();
        match event {
            Event::AdcReady => self.reg.ier.modify(|_, w| w.adrdyie().bit(enable)),
            Event::EndOfSamplingPhase => self.reg.ier.modify(|_, w| w.eosmpie().bit(enable)),
            Event::EndOfConversion => self.reg.ier.modify(|_, w| w.eocie().bit(enable)),
            Event::EndOfSequence => self.reg.ier.modify(|_, w| w.eosie().bit(enable)),
            Event::Overrun => self.reg.ier.modify(|_, w| w.ovrie().bit(enable)),
            Event::InjectedChannelEndOfConversion => {
                self.reg.ier.modify(|_, w| w.jeocie().bit(enable))
            }
            Event::InjectedChannelEndOfSequence => {
                self.reg.ier.modify(|_, w| w.jeosie().bit(enable))
            }
            Event::AnalogWatchdog1 => self.reg.ier.modify(|_, w| w.awd1ie().bit(enable)),
            Event::AnalogWatchdog2 => self.reg.ier.modify(|_, w| w.awd2ie().bit(enable)),
            Event::AnalogWatchdog3 => self.reg.ier.modify(|_, w| w.awd3ie().bit(enable)),
            Event::InjectedContextQueueOverfow => {
                self.reg.ier.modify(|_, w| w.jqovfie().bit(enable))
            }
        };
    }

    /// Enable or disable interrupt for the specified [`Event`]s.
    ///
    /// Like [`Adc::configure_interrupt`], but instead using an enumset. The corresponding
    /// interrupt for every [`Event`] in the set will be enabled, every other interrupt will be
    /// **disabled**.
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    #[inline]
    pub fn configure_interrupts(&mut self, events: EnumSet<Event>) {
        for event in events.complement().iter() {
            self.configure_interrupt(event, false);
        }
        for event in events.iter() {
            self.configure_interrupt(event, true);
        }
    }

    /// Clear the given interrupt event flag.
    #[inline]
    pub fn clear_event(&mut self, event: Event) {
        self.reg.isr.write(|w| match event {
            Event::AdcReady => w.adrdy().clear(),
            Event::EndOfSamplingPhase => w.eosmp().clear(),
            Event::EndOfConversion => w.eoc().clear(),
            Event::EndOfSequence => w.eos().clear(),
            Event::Overrun => w.ovr().clear(),
            Event::InjectedChannelEndOfConversion => w.jeoc().clear(),
            Event::InjectedChannelEndOfSequence => w.jeos().clear(),
            Event::AnalogWatchdog1 => w.awd1().clear(),
            Event::AnalogWatchdog2 => w.awd2().clear(),
            Event::AnalogWatchdog3 => w.awd3().clear(),
            Event::InjectedContextQueueOverfow => w.jqovf().clear(),
        });
    }

    /// Clear **all** interrupt events.
    #[inline]
    pub fn clear_events(&mut self) {
        // SAFETY: TO clear all events, write 1 to it.
        self.reg.isr.write(|w| unsafe { w.bits(u32::MAX) });
    }
}

impl<ADC> Adc<ADC, OneShot>
where
    ADC: Instance,
{
    /// Converts the [`Adc`] in [`OneShot`] mode back to [`Enabled`] mode, while still staying in
    /// the confiugration for [`OneShot`] mode.
    pub fn into_enabled(self) -> Adc<ADC, Enabled> {
        Adc {
            reg: self.reg,
            state: PhantomData,
        }
    }

    /// Releases the ADC peripheral
    #[inline]
    pub fn free(self) -> ADC {
        self.into_enabled().into_disabled().reg
    }
}

impl<ADC> Adc<ADC>
where
    ADC: crate::interrupts::InterruptNumber,
{
    /// Obtain the associated interrupt number for the serial peripheral.
    ///
    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
    /// This is useful for all `cortex_m::peripheral::INTERRUPT` functions.
    ///
    /// # Note
    ///
    /// This is the easier alternative to obatain the interrupt for:
    ///
    /// ```
    /// use cortex_m::peripheral::INTERRUPT;
    /// use stm32f3xx_hal::pac::ADC1;
    /// use stm32f3xx_hal::interrupt::InterruptNumber;
    ///
    /// const INTERRUPT: Interrupt = <ADC1 as InterruptNumber>::INTERRUPT;
    /// ```
    ///
    /// though this function can not be used in a const context.
    // #[doc(alias = "unmask")]
    pub fn interrupt(&self) -> <ADC as crate::interrupts::InterruptNumber>::Interrupt {
        <ADC as crate::interrupts::InterruptNumber>::INTERRUPT
    }
}

/// Marker trait if a type state allows to configure the [`Adc`] peripheral.
pub trait Configurable: crate::private::Sealed {}

impl<ADC, Word, Pin> embedded_hal::adc::OneShot<ADC, Word, Pin> for Adc<ADC>
where
    ADC: Instance,
    Word: From<u16>,
    Pin: Channel<ADC, ID = channel::Id>,
{
    type Error = ();

    /// Synchronously record a single sample of `pin`.
    ///
    /// # Note
    ///
    /// This function needs to reconfigure the peripheral to read a single sample, but no such
    /// reconfiguration should be visible to the user. If this is the case, consider it a bug!
    ///
    /// This function should not trigger any interrupt.
    ///
    /// To have a more efficient implementation of this trait, configure the device
    /// to [`OneShot`] mode via [`Adc::into_oneshot`].
    fn read(&mut self, pin: &mut Pin) -> nb::Result<Word, Self::Error> {
        let conv = self.conversion_mode();
        let dma = self.dma_mode();
        let ext = self.external_trigger();
        self.set_conversion_mode(config::ConversionMode::Single);
        self.set_dma_mode(config::DmaMode::Disabled);
        self.set_external_trigger(None);

        // TODO(Sh3Rm4n): Is this an event?
        #[cfg(feature = "svd-f373")]
        let scan = self.scan();
        #[cfg(feature = "svd-f373")]
        self.set_scan(config::Scan::Disabled);

        let is_eoc_enabled = self.is_interrupt_configured(Event::EndOfConversion);
        let is_eos_enabled = self.is_interrupt_configured(Event::EndOfSequence);
        self.disable_interrupt(Event::EndOfConversion);
        self.disable_interrupt(Event::EndOfSequence);

        let seq_len = self.sequence_length();
        let old_id = self.channel_sequence(config::Sequence::One);
        self.set_sequence_length(config::Sequence::One);
        self.set_pin_sequence_position(config::Sequence::One, pin);

        // Wait for the sequence to complete
        self.clear_event(Event::EndOfConversion);
        self.clear_event(Event::EndOfSequence);
        self.start_conversion();
        while !self.is_event_triggered(Event::EndOfConversion)
            && !self.is_event_triggered(Event::EndOfSequence)
        {}
        self.clear_event(Event::EndOfConversion);
        self.clear_event(Event::EndOfSequence);

        // Resolve this method overwrite.
        let result = self.data_register();

        // Reset the config
        if let Some(id) = old_id {
            // SAFETY: As long as the user has set the channel through the safe interface,
            // this should'nt be a problem.
            unsafe { self.set_channel_sequence_position(config::Sequence::One, id) };
        }
        self.set_sequence_length(seq_len);

        self.configure_interrupt(Event::EndOfSequence, is_eos_enabled);
        self.configure_interrupt(Event::EndOfConversion, is_eoc_enabled);

        #[cfg(feature = "svd-f373")]
        self.set_scan(config::Scan::Disabled);

        self.set_external_trigger(ext);
        self.set_dma_mode(dma);
        self.set_conversion_mode(conv);

        Ok(result.into())
    }
}

impl<ADC, Word, Pin> embedded_hal::adc::OneShot<ADC, Word, Pin> for Adc<ADC, OneShot>
where
    ADC: Instance,
    Word: From<u16>,
    Pin: Channel<ADC, ID = channel::Id>,
{
    type Error = ();

    /// Synchronously record a single sample of `pin`.
    fn read(&mut self, _pin: &mut Pin) -> nb::Result<Word, Self::Error> {
        // self.set_pin_sequence_position(config::Sequence::One, pin);
        self.reg
            .sqr1
            .modify(|_, w| unsafe { w.sq1().bits(Pin::channel().into()) });

        // Wait for the sequence to complete

        // self.clear_events();
        self.reg.isr.reset();
        // self.start_conversion();
        self.reg.cr.modify(|_, w| w.adstart().start());
        // while !self.is_event_triggered(Event::EndOfConversion)
        //     && !self.is_event_triggered(Event::EndOfSequence)
        // {}
        while !self.reg.isr.read().eoc().is_complete() && !self.reg.isr.read().eos().is_complete() {
        }
        // self.clear_events();
        self.reg.isr.reset();

        // Resolve this method overwrite.
        // self.data_register().into()
        Ok(self.reg.dr.read().rdata().bits().into())
    }
}

/// ADC Instance
pub trait Instance: Deref<Target = adc1::RegisterBlock> + crate::private::Sealed {
    /// Shared Instance / Registerblock between multiple ADCs
    type SharedInstance: CommonInstance;
}

/// The common ADC instance, which is shared by two ADC peripherals
pub trait CommonInstance: Deref<Target = adc1_2::RegisterBlock> + crate::private::Sealed {
    /// The one ADC peripheral associated with the common ADC
    type Childs;

    #[doc(hidden)]
    fn enable_clock(&mut self, clocks: &Clocks, ahb: &mut AHB);
    #[doc(hidden)]
    // TODO: Make public?
    fn clock(&self, clocks: &Clocks) -> Option<Hertz>;
}

/// Type State for an enabled [`Adc`]
///
/// An [`Adc`] in that state can be obtained via [`Adc::new`].
pub struct Enabled;
impl crate::private::Sealed for Enabled {}
impl Configurable for Enabled {}

/// Type State for an disabled [`Adc`], which does also not guarantee to be calibrated.
///
/// An [`Adc`] in that state can be obtained via [`Adc::new_disabled`].
pub struct Disabled;
impl crate::private::Sealed for Disabled {}
impl Configurable for Disabled {}

/// Type State for an [`Adc`] ready to be used for the [`embedded_hal::adc::OneShot`] trait.
///
/// To convert an [`Adc`] into that state, use [`Adc::into_oneshot`].
///
/// # Note
///
/// This ensure the most efficient implementation for this trait and is inteded to be used,
/// when passing the peripheral to drivers erxpecting that mode to be configured.
///
/// Nothing else can be done with the peripheral in that mode though. Converting
/// the peripheral back can be done via [`Adc::into_enabled`].
pub struct OneShot;

// Macro to implement ADC functionality for ADC1 and ADC2
macro_rules! adc {
    ($(
        $ADC:ident: (
            $ADCX_Y:ident,
            $INTERRUPT:path
        ),
    )+) => {
        $(
            impl crate::private::Sealed for pac::$ADC {}

            impl Instance for pac::$ADC {
                type SharedInstance = pac::$ADCX_Y;
            }

            impl crate::interrupts::InterruptNumber for pac::$ADC {
                type Interrupt = Interrupt;
                const INTERRUPT: Interrupt = $INTERRUPT;
            }
        )+
    };

    ([ $(($A:literal, $X:literal, $Y:literal, $INTERRUPT:path)),+ ]) => {
        paste::paste! {
            adc!(
                $(
                    [<ADC $A>]: (
                        [<ADC $X _ $Y>],
                        $INTERRUPT
                    ),
                )+
            );
        }
    };

    // TODO(Sh3Rm4n): https://github.com/stm32-rs/stm32-rs/pull/696
    ([ $(($A:literal, $ADC:ident, $INTERRUPT:path)),+ ]) => {
        paste::paste! {
            adc!(
                $(
                    [<ADC $A>]: (
                        $ADC,
                        $INTERRUPT
                    ),
                )+
            );
        }
    };
}

macro_rules! adc_common {
    ($(
        $ADCX_Y:ident: (
            $ADC_CHILDS:ty,
            $ADCXYPRES_A:ident,
            $adcXYen:ident,
            $adcXYpres:ident
        ),
    )+) => {
        $(

            impl CommonInstance for pac::$ADCX_Y {
                type Childs = $ADC_CHILDS;

                fn enable_clock(&mut self, clocks: &Clocks, ahb: &mut AHB) {
                    pac::$ADCX_Y::enable(ahb);
                    // self.enable(ahb);


                    // No clock can be set, so we have to fallback to a default.
                    if self.clock(clocks).is_none() {
                        self.ccr.modify(|_, w| w
                            .ckmode().variant(CKMODE_A::SYNCDIV1)
                        );
                    };
                }

                fn clock(&self, clocks: &Clocks) -> Option<Hertz> {
                    use crate::pac::rcc::cfgr2::$ADCXYPRES_A;
                    use crate::pac::RCC;
                    // SAFETY: atomic read with no side effects
                    let adc_pres = unsafe { &(*RCC::ptr()).cfgr2.read().$adcXYpres() };
                    // let common_adc = unsafe { &(*Self::SharedInstance::ptr()) };

                    // FIXME(Sh3Rm4n): ADC_PRES is not clocking the peripheral?
                    // (Note HSE has to be on) Also setting SYNCDIV does not work.
                    // TODO: For no ADCPRES to work. ASYNC mode has to be on!
                    Some(match clocks.pllclk() {
                        Some(pllclk) if !adc_pres.is_no_clock()  => {
                            pllclk
                                / match adc_pres.variant() {
                                    Some($ADCXYPRES_A::DIV1) => 1,
                                    Some($ADCXYPRES_A::DIV2) => 2,
                                    Some($ADCXYPRES_A::DIV4) => 4,
                                    Some($ADCXYPRES_A::DIV6) => 6,
                                    Some($ADCXYPRES_A::DIV8) => 8,
                                    Some($ADCXYPRES_A::DIV10) => 10,
                                    Some($ADCXYPRES_A::DIV12) => 12,
                                    Some($ADCXYPRES_A::DIV16) => 16,
                                    Some($ADCXYPRES_A::DIV32) => 32,
                                    Some($ADCXYPRES_A::DIV64) => 64,
                                    Some($ADCXYPRES_A::DIV128) => 128,
                                    Some($ADCXYPRES_A::DIV256) => 256,
                                    Some($ADCXYPRES_A::NOCLOCK) | None => 1,
                                }
                        }
                        _ => {
                            clocks.sysclk()
                                / match self.ccr.read().ckmode().variant() {
                                    CKMODE_A::SYNCDIV1 => 1,
                                    CKMODE_A::SYNCDIV2 => 2,
                                    CKMODE_A::SYNCDIV4 => 4,
                                    // Asynchronous should be enabled if PLL is on. If this line of
                                    // code is reached PLL is off. Indicate that, so fallbacks can
                                    // be set.
                                    CKMODE_A::ASYNCHRONOUS => return None,
                                }
                        }
                    })
                }
            }
        )+
    };

    ([ $(($X:literal, $Y:literal)),+ ]) => {
        paste::paste! {
            adc_common!(
                $(
                    [<ADC $X _ $Y>]: (
                        (pac::[<ADC $X>], pac::[<ADC $Y>]),
                        [<ADC $X $Y PRES_A>],
                        [<adc $X $Y en>],
                        [<adc $X $Y pres>]
                    ),
                )+
            );
        }
    };

    // TODO(Sh3Rm4n): https://github.com/stm32-rs/stm32-rs/pull/696
    ([ $(($X:literal, $Y:literal, $ADC:ident)),+ ]) => {
        paste::paste! {
            adc_common!(
                $(
                    $ADC: (
                        (pac::[<ADC $X>], pac::[<ADC $Y>]),
                        [<ADC $X $Y PRES_A>],
                        [<adc $X $Y en>],
                        [<adc $X $Y pres>]
                    ),
                )+
            );
        }
    };

    ([ $(($A:literal, $X:literal, $Y:literal)),+ ]) => {
        paste::paste! {
            adc_common!(
                $(
                    [<ADC $X _ $Y>]: (
                        pac::[<ADC $A>],
                        [<ADC $X PRES_A>],
                        [<adc $X en>],
                        [<adc $X pres>]
                    ),
                )+
            );
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f301")] {
        adc_common!([(1, 1, 2)]);
        adc!([(1, 1, 2, Interrupt::ADC1_IRQ)]);
    } else if #[cfg(not(feature = "svd-f3x4"))]{
        adc_common!([(1, 2)]);
        adc!([(1, 1, 2, Interrupt::ADC1_2)]);
    }
}

// TODO(Sh3Rm4n):
// * stm32f373 will become complicated, because no ADC1_2

// See https://stm32-rs.github.io/stm32-rs/stm32f/stm32f3/index.html for an overview
#[cfg(any(feature = "svd-f302", feature = "svd-f303"))]
adc!([(2, 1, 2, Interrupt::ADC1_2)]);

// FIXME(Sh3Rm4n): https://github.com/stm32-rs/stm32-rs/pull/696
// #[cfg(feature = "svd-f3x4")]
cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f3x4")] {
        adc!([(1, ADC_COMMON, Interrupt::ADC1_2)]);
        adc!([(2, ADC_COMMON, Interrupt::ADC1_2)]);
        adc_common!([(1, 2, ADC_COMMON)]);
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f303")] {
        adc_common!([(3, 4)]);
        adc!([(3, 3, 4, Interrupt::ADC3), (4, 3, 4, Interrupt::ADC4)]);
    }
}
