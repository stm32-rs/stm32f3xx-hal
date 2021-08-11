//! # Timers
//!
//! Abstractions of the internal timer peripherals
//! The timer modules implements the [`CountDown`] and [`Periodic`] traits.
//!
//! [Read]: embedded_hal::timer::CountDown
//! [Write]: embedded_hal::timer::Periodic

use core::convert::{From, TryFrom};

use crate::pac::{DCB, DWT};
#[cfg(feature = "enumset")]
use enumset::{EnumSet, EnumSetType};
use void::Void;

use crate::hal::timer::{Cancel, CountDown, Periodic};
#[allow(unused)]
use crate::pac::RCC;
use crate::rcc::{Clocks, APB1, APB2};
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
        // This is needed, so that the DWT timer starts to count.
        dcb.enable_trace();
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
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(self) -> u32 {
        DWT::get_cycle_count().wrapping_sub(self.now)
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
    pub fn new(tim: TIM, clocks: Clocks, apb: &mut <TIM as Instance>::APB) -> Self {
        TIM::enable_clock(apb);

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

    /// Check if an interrupt event happend.
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

        let ticks = clock.integer() * *timeout.scaling_factor() * timeout.integer();

        let psc = crate::unwrap!(u16::try_from((ticks - 1) / (1 << 16)).ok());
        self.tim.set_psc(psc);

        let arr = crate::unwrap!(u16::try_from(ticks / u32::from(psc + 1)).ok());
        self.tim.set_arr(arr);

        // Ensure that the below procedure does not create an unexpected interrupt.
        let is_update_interrupt_active = self.tim.is_dier_uie_set();
        if is_update_interrupt_active {
            self.tim.set_dier_uie(false);
        }

        // Trigger an update event to load the prescaler value to the clock The above line raises
        // an update event which will indicate that the timer is already finished.
        self.tim.set_egr_ug();
        // Since this is not the case, it should be cleared.
        self.clear_event(Event::Update);

        if is_update_interrupt_active {
            self.tim.set_dier_uie(true);
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
    CommonRegisterBlock + crate::interrupts::InterruptNumber + crate::private::Sealed
{
    /// Peripheral bus instance which is responsible for the peripheral
    type APB;

    #[doc(hidden)]
    fn enable_clock(apb: &mut Self::APB);
    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Hertz;
}

macro_rules! timer {
    ($({
        $TIMX:ident: (
            $timXen:ident,
            $timXrst:ident,
            $timXsw:ident,
            $timerXclock:ident,
            $APB:ident,
            $pclkX:ident,
            $PCLKX:ident,
            $ppreX:ident,
            $INTERRUPT:ident
        ),
    },)+) => {
        $(
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

            impl crate::private::Sealed for crate::pac::$TIMX {}
            impl Instance for crate::pac::$TIMX {
                type APB = $APB;

                #[inline]
                fn enable_clock(apb: &mut Self::APB) {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().enabled());
                    apb.rstr().modify(|_, w| w.$timXrst().reset());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());
                }

                #[inline]
                fn clock(clocks: &Clocks) -> Hertz {
                    $timerXclock(clocks)
                }


            }
        )+
    };

    ([ $(($X:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            timer! {
                $(
                    {
                        [<TIM $X>]: (
                            [<tim $X en>],
                            [<tim $X rst>],
                            [<tim $X sw>],
                            [<timer $X clock>],
                            [<APB $APB>],
                            [<pclk $APB>],
                            [<PCLK $APB>],
                            [<ppre $APB>],
                            [<TIM $X _INTERRUPT_TYPES>]
                        ),
                    },
                )+
            }
        }
    };
}

#[allow(unused)]
macro_rules! timer_var_clock {
    ($($timerXclock:ident, $timXsw:ident, $pclkX:ident, $ppreX:ident),+) => {
        $(
            /// Return the currently set source frequency for the UART peripheral
            /// depending on the clock source.
            fn $timerXclock(clocks: &Clocks) -> Hertz {
                // SAFETY: Atomic read with no side-effects.
                match unsafe {(*RCC::ptr()).cfgr3.read().$timXsw().variant()} {
                    // PCLK2 is really the wrong name, as depending on the type of chip, it is
                    // pclk1 or pclk2. This distinction is however not made in stm32f3.
                    crate::pac::rcc::cfgr3::TIM1SW_A::PCLK2 =>  {
                        // Conditional mutliplier after APB prescaler is used.
                        // See RM0316 Fig 13.
                        if clocks.$ppreX() > 1 {
                            clocks.$pclkX() * 2
                        } else {
                            clocks.$pclkX()
                        }
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
        )+
    };
    ([ $(($X:literal, $Y:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            timer_var_clock!(
                $([<timer $X clock>], [<tim $Y sw>], [<pclk $APB>], [<ppre $APB>]),+
            );

        }

        timer!([$(($X, $APB)),+]);
    };
    ([ $(($X:literal, $APB:literal)),+ ]) => {
        timer_var_clock!([$(($X, $X, $APB)),+]);
    };
}

macro_rules! timer_static_clock {
    ($($timerXclock:ident, $pclkX:ident, $ppreX:ident),+) => {
        $(
            /// Return the currently set source frequency the UART peripheral
            /// depending on the clock source.
            fn $timerXclock(clocks: &Clocks) -> Hertz {
                if clocks.$ppreX() > 1 {
                    clocks.$pclkX() * 2
                } else {
                    clocks.$pclkX()
                }
            }
        )+
    };
    ([ $(($X:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            timer_static_clock!(
                $([<timer $X clock>], [<pclk $APB>], [<ppre $APB>]),+
            );
        }

        timer!([$(($X, $APB)),+]);
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
        timer_static_clock!([(2, 1), (6, 1)]);
        timer_var_clock!([(1, 2), (15, 2), (16, 2), (17, 2)]);
    }
}

cfg_if::cfg_if! {
    // RM0365 Fig. 12
    if #[cfg(all(feature = "svd-f302", feature = "gpio-f303"))] {
        timer_static_clock!([
            (2, 1),
            (3, 1),
            (4, 1),
            (6, 1),
            (15, 2),
            (16, 2),
            (17, 2)
        ]);
        timer_var_clock!([(1, 2)]);
    }

    // RM0365 Fig. 13
    else if #[cfg(all(feature = "svd-f302", feature = "gpio-f303e"))] {
        timer_static_clock!([
            (6, 1)
        ]);
        timer_var_clock!([
            (1, 2),
            (2, 1),
            (15, 2),
            (16, 2),
            (17, 2)
        ]);
        timer_var_clock!([(3, 34, 1), (4, 34, 1)]);
    }

    // RM0365 Fig. 14
    //
    // NOTE: Even though timer 7 appears in the clock tree and an interrupt exists for 7
    // (Interrupt::TIM7_IRQ), TIM7 has no chapter is nowhere to be mentioned. Also tim7sw() does
    // not exsist.
    else if #[cfg(all(feature = "svd-f302", feature = "gpio-f302"))] {
        timer_static_clock!([
            (2, 1),
            (6, 1)
        ]);
        timer_var_clock!([
            (1, 2),
            (15, 2),
            (16, 2),
            (17, 2)
        ]);
    }

    // RM0316 Fig. 13
    else if #[cfg(all(feature = "svd-f303", feature = "gpio-f303"))] {
        timer_static_clock!([
            (2, 1),
            (3, 1),
            (4, 1),
            (6, 1),
            (7, 1),
            (15, 2),
            (16, 2),
            (17, 2)
        ]);
        timer_var_clock!([(1, 2), (8, 2)]);
    }

    // RM0316 Fig. 14
    else if #[cfg(all(feature = "svd-f303", feature = "gpio-f303e"))] {
        timer_static_clock!([(6, 1), (7, 1)]);
        timer_var_clock!([
            (1, 2),
            (2, 1),
            (8, 2),
            (15, 2),
            (16, 2),
            (17, 2),
            (20, 2)
        ]);
        timer_var_clock!([(3, 34, 1), (4, 34, 1)]);
    }

    // RM0316 Fig. 15
    else if #[cfg(all(feature = "svd-f303", feature = "gpio-f333"))] {
        timer_static_clock!([
            (2, 1),
            (3, 1),
            (6, 1),
            (7, 1),
            (15, 2),
            (16, 2),
            (17, 2)
        ]);
        timer_var_clock!([(1, 2)]);
    }

    // RM0313 Fig. 12 - this clock does not deliver the information about the timers.
    //
    // This information is from chapter 16, 17 and 18 of RM0313.
    // The information, which APB is connected to the timer, is only avaliable while looking
    // at the apb[1,2]rst registers.
    else if #[cfg(feature = "gpio-f373")] {
        timer_static_clock!([
            (2, 1),
            (3, 1),
            (4, 1),
            (5, 1),
            (6, 1),
            (7, 1),
            (12, 1),
            (13, 1),
            (14, 1),
            (15, 2),
            (16, 2),
            (17, 2),
            (18, 1),
            (19, 2)
        ]);
    }

    // RM0364 Fig. 10
    else if #[cfg(all(feature = "svd-f3x4", feature = "gpio-f333"))] {
        timer_static_clock!([
            (2, 1),
            (3, 1),
            (6, 1),
            (7, 1),
            (15, 2),
            (16, 2),
            (17, 2)
        ]);
        timer_var_clock!([(1, 2)]);
    }
}
