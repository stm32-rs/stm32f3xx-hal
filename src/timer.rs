//! # Timers
//!
//! Abstractions of the internal timer peripherals
//! The timer modules implements the [`CountDown`] and [`Periodic`] traits.
//!
//! [Read]: embedded_hal::timer::CountDown
//! [Write]: embedded_hal::timer::Periodic

use core::convert::{From, TryFrom};

use cortex_m::peripheral::DWT;
#[cfg(feature = "enumset")]
use enumset::{EnumSet, EnumSetType};
use void::Void;

use crate::hal::timer::{CountDown, Periodic};
#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::pac::TIM1;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::pac::TIM20;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::pac::TIM4;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::pac::TIM8;
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
use crate::pac::{TIM12, TIM13, TIM14, TIM18, TIM19, TIM5};
use crate::pac::{TIM15, TIM16, TIM17, TIM2, TIM6};
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::pac::{TIM3, TIM7};
use crate::rcc::{Clocks, APB1, APB2};
use crate::time::rate::*;

/// A monotonic nondecreasing timer.
#[derive(Debug, Clone, Copy)]
pub struct MonoTimer {
    frequency: Hertz,
}

#[cfg(feature = "defmt")]
impl defmt::Format for MonoTimer {
    fn format(&self, f: defmt::Formatter) {
        // Format as hexadecimal.
        defmt::write!(
            f,
            "MonoTimer {{ frequency: {} Hz }}",
            self.frequency.integer(),
        );
    }
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
    pub fn frequency(self) -> Hertz {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(self) -> Instant {
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

/// Associated clocks with timers
pub trait PclkSrc {
    /// Get the current frequency of the associated clock
    fn get_clk(clocks: &Clocks) -> Hertz;
}

/// Hardware timers
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
}

/// Interrupt events
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
#[non_exhaustive]
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

macro_rules! hal {
    ($({
        $TIMX:ident: ($tim:ident, $timXen:ident, $timXrst:ident),
        $APB:ident: ($apb:ident, $pclkX:ident),
    },)+) => {
        $(
            impl PclkSrc for $TIMX {
                fn get_clk(clocks: &Clocks) -> Hertz {
                    clocks.$pclkX()
                }
            }

            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Self::Time>,
                {
                    self.stop();

                    let frequency = timeout.into().integer();
                    let timer_clock = $TIMX::get_clk(&self.clocks);
                    let ticks = timer_clock.0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;
                    let psc = crate::unwrap!(u16::try_from((ticks - 1) / (1 << 16)).ok());

                    // NOTE(write): uses all bits in this register.
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = crate::unwrap!(u16::try_from(ticks / u32::from(psc + 1)).ok());

                    // TODO (sh3rm4n)
                    // self.tim.arr.write(|w| { w.arr().bits(arr) });
                    self.tim.arr.write(|w| unsafe { w.bits(u32::from(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register.
                    self.tim.egr.write(|w| w.ug().update());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_event(Event::Update);

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().enabled());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_event(Event::Update);
                        Ok(())
                    }
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIMX, timeout: T, clocks: Clocks, $apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    $apb.enr().modify(|_, w| w.$timXen().enabled());
                    $apb.rstr().modify(|_, w| w.$timXrst().reset());
                    $apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer { clocks, tim };
                    timer.start(timeout);

                    timer
                }

                /// Stops the timer
                #[inline]
                pub fn stop(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().disabled());
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


                // TODO(Sh3Rm4n): After macro -> Instance refactor introduce associated const INTERRUPT.
                // /// Get the correspoing Interrupt number
                // // pub fn nvic() -> Interrupt { }

                /// Enable or disable the interrupt for the specified [`Event`].
                #[inline]
                pub fn configure_interrupt(&mut self, event: Event, enable: bool) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().bit(enable)),
                    }
                }

                /// Enable or disable interrupt for the specified [`Event`]s.
                ///
                /// Like [`Timer::configure_interrupt`], but instead using an enumset. The corresponding
                /// interrupt for every [`Event`] in the set will be enabled, every other interrupt will be
                /// **disabled**.
                #[cfg(feature = "enumset")]
                #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
                pub fn configure_interrupts(&mut self, events: EnumSet<Event>) -> &mut Self {
                    for event in events.complement().iter() {
                        self.configure_interrupt(event, false);
                    }
                    for event in events.iter() {
                        self.configure_interrupt(event, true);
                    }

                    self
                }

                /// Check if an interrupt event happend.
                pub fn is_event_triggered(&self, event: Event) -> bool {
                    match event {
                        Event::Update => self.tim.sr.read().uif().bit(),
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
                        Event::Update => self.tim.sr.modify(|_, w| w.uif().clear()),
                    }
                }

                /// Clear **all** interrupt events.
                #[inline]
                pub fn clear_events(&mut self) {
                    // SAFETY: This atomic write clears all flags and ignores the reserverd bit fields.
                    self.tim.sr.write(|w| unsafe { w.bits(0) });
                }

                /// Releases the TIM peripheral
                #[inline]
                pub fn free(mut self) -> $TIMX {
                    self.stop();
                    self.tim
                }
            }
        )+
    }
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
hal! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM15: (tim15, tim15en, tim15rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
    },
}

#[cfg(feature = "stm32f302")]
hal! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1,pclk1),
    },
    {
        TIM15: (tim15, tim15en, tim15rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
    },
}

#[cfg(feature = "stm32f303")]
hal! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM4: (tim4, tim4en, tim4rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM8: (tim8, tim8en, tim8rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM15: (tim15, tim15en, tim15rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM20: (tim20, tim20en, tim20rst),
        APB2: (apb2, pclk2),
    },
}

#[cfg(feature = "stm32f334")]
hal! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM15: (tim15, tim15en, tim15rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
    },
}

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
hal! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM4: (tim4, tim4en, tim4rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM5: (tim5, tim5en, tim5rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM12: (tim12, tim12en, tim12rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM13: (tim13, tim13en, tim13rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM14: (tim14, tim14en, tim14rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM15: (tim15, tim15en, tim15rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM18: (tim18, tim18en, tim18rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM19: (tim19, tim19en, tim19rst),
        APB2: (apb2, pclk2),
    },
}

#[cfg(any(feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
hal! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM4: (tim4, tim4en, tim4rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
    },
    {
        TIM8: (tim8, tim8en, tim8rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM15: (tim15, tim15en, tim15rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
    },
    {
        TIM20: (tim20, tim20en, tim20rst),
        APB2: (apb2, pclk2),
    },
}
