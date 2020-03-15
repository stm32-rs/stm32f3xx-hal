//! Timers

use crate::hal::timer::{CountDown, Periodic};
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f378",
    feature = "stm32f398",
))]
use crate::stm32::TIM1;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::stm32::TIM20;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::stm32::TIM4;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f378",
    feature = "stm32f398",
))]
use crate::stm32::TIM8;
#[cfg(feature = "stm32f373")]
use crate::stm32::{TIM12, TIM13, TIM14, TIM18, TIM19, TIM5};
use crate::stm32::{TIM15, TIM16, TIM17, TIM2, TIM6};
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::stm32::{TIM3, TIM7};

use cast::{u16, u32};
use nb;
use void::Void;

use crate::rcc::{Clocks, APB1, APB2};
use crate::time::Hertz;

/// Associated clocks with timers
pub trait PclkSrc {
    fn get_clk(clocks: &Clocks) -> Hertz;
}

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
}

/// Interrupt events
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
                    T: Into<Hertz>,
                {
                    self.stop();

                    let frequency = timeout.into().0;
                    let timer_clock = $TIMX::get_clk(&self.clocks);
                    let ticks = timer_clock.0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    // NOTE(write): uses all bits in this register.
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    // TODO (sh3rm4n)
                    // self.tim.arr.write(|w| { w.arr().bits(arr) });
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register.
                    self.tim.egr.write(|w| w.ug().update());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().enabled());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
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

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().enabled()),
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().disabled()),
                    }
                }

                /// Stops the timer
                pub fn stop(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().disabled());
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear());
                }

                /// Releases the TIM peripheral
                pub fn release(mut self) -> $TIMX {
                    self.stop();
                    self.tim
                }
            }
        )+
    }
}

#[cfg(feature = "stm32f301")]
hal! {
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

#[cfg(feature = "stm32f373")]
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

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f378",
    feature = "stm32f398"
))]
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
