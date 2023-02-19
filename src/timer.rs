//! # Timers
//!
//! Abstractions of the internal timer peripherals
//! The timer modules implements the [`CountDown`] and [`Periodic`] traits.
//!
//! ## Examples
//!
//! Check out [examples/adc.rs], where a [`Periodic`] timer is used to wake
//! up the main-loop regularly.
//!
//! [examples/adc.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.1/examples/adc.rs

use core::convert::TryFrom;

use crate::pac::{DCB, DWT};
#[cfg(feature = "enumset")]
use enumset::{EnumSet, EnumSetType};
use void::Void;

use crate::hal::timer::{Cancel, CountDown, Periodic};
#[allow(unused)]
use crate::pac::{self, RCC};
use crate::rcc::{self, Clocks};
use crate::time::{duration, fixed_point::FixedPoint, rate::Hertz};

mod interrupts;

/// A monotonic nondecreasing timer.
#[derive(Debug, Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

#[cfg(feature = "defmt")]
impl defmt::Format for MonoTimer {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "MonoTimer {{ frequency: {} Hz }}",
            self.frequency.integer(),
        );
    }
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(mut dwt: DWT, clocks: Clocks, dcb: &mut DCB) -> Self {
        // Takes ownership of `DWT` so that the CYCCNT counter can't be stopped or resetted.

        // This is needed, so that the DWT timer starts to count.
        dcb.enable_trace();
        dwt.enable_cycle_counter();

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
            now: DWT::cycle_count(),
        }
    }
}

/// A measurement of a monotonically nondecreasing clock
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.now)
    }
}

/// Hardware timers
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<TIM> {
    tim: TIM,
    clocks: Clocks,
}

/// Interrupt events
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
#[non_exhaustive]
// TODO(Sh3Rm4n): What to do about events, which not all timers support?
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

impl<TIM> Timer<TIM>
where
    TIM: Instance,
{
    /// Configures a TIM peripheral as a periodic count down timer
    pub fn new(tim: TIM, clocks: Clocks, apb: &mut <TIM as rcc::RccBus>::Bus) -> Self {
        TIM::enable(apb);
        TIM::reset(apb);

        Timer { clocks, tim }
    }

    /// Stops the timer
    #[inline]
    pub fn stop(&mut self) {
        self.tim.set_cr1_cen(false);
    }

    /// Enable or disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn enable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, true);
    }

    /// Enable or disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn disable_interrupt(&mut self, event: Event) {
        self.configure_interrupt(event, false);
    }

    /// Obtain the associated interupt number for the serial peripheral.
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
    /// use stm32f3xx_hal::pac::TIM1;
    /// use stm32f3xx_hal::interrupts::InterruptNumber;
    ///
    /// const INTERRUPT: Interrupt = <TIM2 as InterruptNumber>::INTERRUPT;
    /// ```
    ///
    /// though this function can not be used in a const context.
    #[doc(alias = "unmask")]
    pub fn interrupt(&self) -> <TIM as crate::interrupts::InterruptNumber>::Interrupt {
        <TIM as crate::interrupts::InterruptNumber>::INTERRUPT
    }

    /// Enable or disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn configure_interrupt(&mut self, event: Event, enable: bool) {
        match event {
            Event::Update => self.tim.set_dier_uie(enable),
        }
    }

    /// Enable or disable interrupt for the specified [`Event`]s.
    ///
    /// Like [`Timer::configure_interrupt`], but instead using an enumset. The corresponding
    /// interrupt for every [`Event`] in the set will be enabled, every other interrupt will be
    /// **disabled**.
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn configure_interrupts(&mut self, events: EnumSet<Event>) {
        for event in events.complement().iter() {
            self.configure_interrupt(event, false);
        }
        for event in events.iter() {
            self.configure_interrupt(event, true);
        }
    }

    /// Check if an interrupt is configured for the [`Event`]
    #[inline]
    pub fn is_interrupt_configured(&self, event: Event) -> bool {
        match event {
            Event::Update => self.tim.is_dier_uie_set(),
        }
    }

    // TODO: Add to other implementations as well (Serial, ...)
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

    /// Check if an interrupt event happened.
    pub fn is_event_triggered(&self, event: Event) -> bool {
        match event {
            Event::Update => self.tim.is_sr_uief_set(),
        }
    }

    /// Get an [`EnumSet`] of all fired interrupt events.
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

    /// Clear the given interrupt event flag.
    #[inline]
    pub fn clear_event(&mut self, event: Event) {
        match event {
            Event::Update => self.tim.clear_sr_uief(),
        }
    }

    /// Clear **all** interrupt events.
    #[inline]
    pub fn clear_events(&mut self) {
        self.tim.clear_sr();
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
    pub unsafe fn peripheral(&mut self) -> &mut TIM {
        &mut self.tim
    }

    /// Releases the TIM peripheral
    #[inline]
    pub fn free(mut self) -> TIM {
        self.stop();
        self.tim
    }
}

impl<TIM> Periodic for Timer<TIM> where TIM: Instance {}

impl<TIM> CountDown for Timer<TIM>
where
    TIM: Instance,
{
    type Time = duration::Generic<u32>;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Self::Time>,
    {
        self.stop();

        let timeout: Self::Time = timeout.into();
        let clock = TIM::clock(&self.clocks);

        let ticks = clock.integer().saturating_mul(timeout.integer()) * *timeout.scaling_factor();

        let psc: u32 = (ticks.saturating_sub(1)) / (1 << 16);
        self.tim.set_psc(crate::unwrap!(u16::try_from(psc).ok()));

        let mut arr = ticks / psc.saturating_add(1);
        // If the set frequency is to high to get a meaningful timer resolution,
        // set arr to one, so the timer can at least do something and code waiting
        // on it is not stuck.
        if psc == 0 && arr == 0 {
            arr = 1;
        }
        self.tim.set_arr(crate::unwrap!(u16::try_from(arr).ok()));

        // Ensure that the below procedure does not create an unexpected interrupt.
        let is_update_interrupt_active = self.is_interrupt_configured(Event::Update);
        if is_update_interrupt_active {
            self.configure_interrupt(Event::Update, false);
        }

        // Trigger an update event to load the prescaler value to the clock The above line raises
        // an update event which will indicate that the timer is already finished.
        self.tim.set_egr_ug();
        // Since this is not the case, it should be cleared.
        self.clear_event(Event::Update);

        if is_update_interrupt_active {
            self.configure_interrupt(Event::Update, true)
        }

        // start counter
        self.tim.set_cr1_cen(true);
    }

    /// Wait until [`Event::Update`] / the timer has elapsed
    /// and than clear the event.
    fn wait(&mut self) -> nb::Result<(), Void> {
        if !self.tim.is_sr_uief_set() {
            Err(nb::Error::WouldBlock)
        } else {
            self.clear_event(Event::Update);
            Ok(())
        }
    }
}

/// Error if a [`Cancel`]-ble [`Timer`] was cancled already or never been started.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AlreadyCancled;

impl<TIM> Cancel for Timer<TIM>
where
    TIM: Instance,
{
    type Error = AlreadyCancled;
    fn cancel(&mut self) -> Result<(), Self::Error> {
        // If timer is already stopped.
        if !self.tim.is_cr1_cen_set() {
            return Err(AlreadyCancled);
        }
        self.stop();
        Ok(())
    }
}

/// Common functionalities of all timer `RegisterBlock` types
/// based on [`crate::pac::tim6::RegisterBlock`].
///
/// This is not meant to be used outside of this crate.
pub trait CommonRegisterBlock: crate::private::Sealed {
    #[doc(hidden)]
    fn set_cr1_cen(&mut self, enable: bool);
    #[doc(hidden)]
    fn is_cr1_cen_set(&mut self) -> bool;
    #[doc(hidden)]
    fn set_dier_uie(&mut self, enable: bool);
    #[doc(hidden)]
    fn is_dier_uie_set(&self) -> bool;
    #[doc(hidden)]
    fn clear_sr_uief(&mut self);
    #[doc(hidden)]
    fn clear_sr(&mut self);
    #[doc(hidden)]
    fn is_sr_uief_set(&self) -> bool;
    #[doc(hidden)]
    fn set_egr_ug(&mut self);
    #[doc(hidden)]
    fn set_psc(&mut self, psc: u16);
    #[doc(hidden)]
    fn set_arr(&mut self, arr: u16);
}

/// Associated clocks with timers
pub trait Instance:
    CommonRegisterBlock
    + crate::interrupts::InterruptNumber
    + crate::private::Sealed
    + rcc::Enable
    + rcc::Reset
{
    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Hertz;
}

macro_rules! timer {
    ($TIMX:ident) => {
        impl CommonRegisterBlock for crate::pac::$TIMX {
            #[inline]
            fn set_cr1_cen(&mut self, enable: bool) {
                self.cr1.modify(|_, w| w.cen().bit(enable));
            }

            #[inline]
            fn is_cr1_cen_set(&mut self) -> bool {
                self.cr1.read().cen().bit()
            }

            #[inline]
            fn set_dier_uie(&mut self, enable: bool) {
                self.dier.modify(|_, w| w.uie().bit(enable));
            }

            #[inline]
            fn is_dier_uie_set(&self) -> bool {
                self.dier.read().uie().bit()
            }

            #[inline]
            fn clear_sr_uief(&mut self) {
                self.sr.modify(|_, w| w.uif().clear())
            }

            #[inline]
            fn clear_sr(&mut self) {
                // SAFETY: This atomic write clears all flags and ignores the reserverd bit fields.
                self.sr.write(|w| unsafe { w.bits(0) });
            }

            #[inline]
            fn is_sr_uief_set(&self) -> bool {
                self.sr.read().uif().is_update_pending()
            }

            #[inline]
            fn set_egr_ug(&mut self) {
                // NOTE(write): uses all bits in this register.
                self.egr.write(|w| w.ug().update());
            }

            #[inline]
            fn set_psc(&mut self, psc: u16) {
                // NOTE(write): uses all bits in this register.
                self.psc.write(|w| w.psc().bits(psc));
            }

            #[inline]
            fn set_arr(&mut self, arr: u16) {
                // TODO (sh3rm4n)
                // self.tim.arr.write(|w| { w.arr().bits(arr) });
                self.arr.write(|w| unsafe { w.bits(u32::from(arr)) });
            }
        }
    };
}

#[allow(unused)]
macro_rules! timer_var_clock {
    ($($TIMX:ident, $timXsw:ident),+) => {
        $(
            impl Instance for crate::pac::$TIMX {
                #[inline]
                fn clock(clocks: &Clocks) -> Hertz {
                    // SAFETY: Atomic read with no side-effects.
                    match unsafe {(*RCC::ptr()).cfgr3.read().$timXsw().variant()} {
                        // PCLK2 is really the wrong name, as depending on the type of chip, it is
                        // pclk1 or pclk2. This distinction is however not made in stm32f3.
                        crate::pac::rcc::cfgr3::TIM1SW_A::PCLK2 =>  {
                            // Conditional mutliplier after APB prescaler is used.
                            // See RM0316 Fig 13.
                            <pac::$TIMX as rcc::BusTimerClock>::timer_clock(clocks)
                        }
                        crate::pac::rcc::cfgr3::TIM1SW_A::PLL => {
                            if let Some(pllclk) = clocks.pllclk() {
                                pllclk * 2
                            } else {
                                // This state should currently not be possible,
                                // because the software source can not be configured right now.
                                crate::panic!("Invalid timer clock source.");
                            }
                        }
                    }
                }
            }

            timer!($TIMX);
        )+
    };
    ($(($X:literal: $Y:literal)),+) => {
        paste::paste! {
            timer_var_clock!(
                $([<TIM $X>], [<tim $Y sw>]),+
            );

        }
    };
    ($($X:literal),+) => {
        timer_var_clock!($(($X: $X)),+);
    };
}

macro_rules! timer_static_clock {
    ($($TIMX:ident),+) => {
        $(
            impl Instance for crate::pac::$TIMX {
                #[inline]
                fn clock(clocks: &Clocks) -> Hertz {
                    <pac::$TIMX as rcc::BusTimerClock>::timer_clock(clocks)
                }
            }

            timer!($TIMX);
        )+
    };
    ($($X:literal),+) => {
        paste::paste! {
            timer_static_clock!(
                $([<TIM $X>]),+
            );
        }
    };
}

// NOTE: Most informations are read out of the clock tree (example RM0316 Fig. 13)
// This gives us the following information:
// - To which APB is the timer connected
// - Which timers are really supported by the chip family
// - Which timers have a static clock and which can also be connected to the PLL clock line
cfg_if::cfg_if! {
    // RM0366 Fig. 11
    //
    // NOTE: Even though timer 7 appears in the clock tree and an interrupt exists for 7
    // (Interrupt::TIM7_IRQ), TIM7 has no chapter is nowwhere to be mentioned. Also tim7sw() does
    // not exsist.
    if #[cfg(feature = "svd-f301")] {
        timer_static_clock!(2, 6);
        timer_var_clock!(1, 15, 16, 17);
    }
}

cfg_if::cfg_if! {
    // RM0365 Fig. 12
    if #[cfg(all(feature = "svd-f302", feature = "gpio-f303"))] {
        timer_static_clock!(2, 3, 4, 6, 15, 16, 17);
        timer_var_clock!(1);
    }

    // RM0365 Fig. 13
    else if #[cfg(all(feature = "svd-f302", feature = "gpio-f303e"))] {
        timer_static_clock!(6);
        timer_var_clock!(1, 2, 15, 16, 17);
        timer_var_clock!((3: 34), (4: 34));
    }

    // RM0365 Fig. 14
    //
    // NOTE: Even though timer 7 appears in the clock tree and an interrupt exists for 7
    // (Interrupt::TIM7_IRQ), TIM7 has no chapter is nowhere to be mentioned. Also tim7sw() does
    // not exsist.
    else if #[cfg(all(feature = "svd-f302", feature = "gpio-f302"))] {
        timer_static_clock!(2, 6);
        timer_var_clock!(1, 15, 16, 17);
    }

    // RM0316 Fig. 13
    else if #[cfg(all(feature = "svd-f303", feature = "gpio-f303"))] {
        timer_static_clock!(2, 3, 4, 6, 7, 15, 16, 17);
        timer_var_clock!(1, 8);
    }

    // RM0316 Fig. 14
    else if #[cfg(all(feature = "svd-f303", feature = "gpio-f303e"))] {
        timer_static_clock!(6, 7);
        timer_var_clock!(1, 2, 8, 15, 16, 17, 20);
        timer_var_clock!((3: 34), (4: 34));
    }

    // RM0316 Fig. 15
    else if #[cfg(all(feature = "svd-f303", feature = "gpio-f333"))] {
        timer_static_clock!(2, 3, 6, 7, 15, 16, 17);
        timer_var_clock!(1);
    }

    // RM0313 Fig. 12 - this clock does not deliver the information about the timers.
    //
    // This information is from chapter 16, 17 and 18 of RM0313.
    // The information, which APB is connected to the timer, is only avaliable while looking
    // at the apb[1,2]rst registers.
    else if #[cfg(feature = "gpio-f373")] {
        timer_static_clock!(2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 17, 18, 19);
    }

    // RM0364 Fig. 10
    else if #[cfg(all(feature = "svd-f3x4", feature = "gpio-f333"))] {
        timer_static_clock!(2, 3, 6, 7, 15, 16, 17);
        timer_var_clock!(1);
    }
}
