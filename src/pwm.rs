use core::marker::PhantomData;
use crate::stm32::{TIM3, TIM8, TIM16};
use embedded_hal::PwmPin;
use super::gpio::{AF1, AF2, AF4, AF10};
use super::gpio::gpioa::{PA6, PA12};
use super::gpio::gpiob::{PB4, PB8, PB9};
use super::gpio::gpioc::{PC8};
use super::gpio::gpioe::{PE0, PE4};
use crate::rcc::{Clocks};
use crate::stm32::{RCC};

//pub struct Tim1Ch1 {}
//pub struct Tim1Ch2 {}
//pub struct Tim1Ch3 {}
//pub struct Tim1Ch4 {}

pub struct Tim3Ch1 {}
pub struct Tim3Ch2 {}
pub struct Tim3Ch3 {}
pub struct Tim3Ch4 {}

pub struct Tim8Ch1 {}
pub struct Tim8Ch2 {}
pub struct Tim8Ch3 {}
pub struct Tim8Ch4 {}

pub struct Tim16Ch1 {}

pub struct NoPins {}
pub struct WithPins {}

pub struct PwmChannel<X, T> {
    pub(crate) timx_chx: PhantomData<X>,
    pub(crate) pin_status: PhantomData<T>,
}

macro_rules! pwm_timer_private {
    // TODO: TimxChy needs to become a list
    ($timx:ident, $TIMx:ty, $apbxenr:ident, $timxen:ident, $trigger_update_event:expr, $enable_break_timer:expr, $reset_slave_master_config:expr, [$($TimxChy:ident,)+], [$($x:ident,)+]) => {
        pub fn $timx(tim: $TIMx, res: u16, freq: u16, clocks: &Clocks) -> ($(PwmChannel<$TimxChy, NoPins>,)+) {
            // Power the timer
            // We use unsafe here to abstract away this implementation detail
            // Justification: It is safe because only scopes with mutable references
            // to TIMx should ever modify this bit.
            unsafe {
                &(*RCC::ptr()).$apbxenr.modify(|_, w| w.$timxen().set_bit());
            }

            // enable auto reload preloader
            tim.cr1.write(|w| w.arpe().set_bit());

            // Set the "resolution" of the duty cycle (ticks before restarting at 0)
            // Oddly this is unsafe for some timers and not others
            #[allow(unused_unsafe)]
            tim.arr.write(|w| unsafe {
                w.arr().bits(res)
            });
            // TODO: Use Hertz?
            // Set the pre-scaler
            tim.psc.write(|w| w.psc().bits(clocks.pclk2().0 as u16 / (res * freq)));

            // Make the settings reload immediately for TIM1/8
            $trigger_update_event(&tim);

            // Reset the slave/master config
            $reset_slave_master_config(&tim);

            // reset
            tim.cr2.write(|w| w);

            // Enable outputs (STM32 Break Timer Specific)
            $enable_break_timer(&tim);

            // Enable the Timer
            tim.cr1.modify(|_, w| w.cen().set_bit());

            // TODO: Passing in the constructor is a bit silly,
            // is there an alternative approach to get this to repeat,
            // even though its not dynamic?
            ($($x { timx_chx: PhantomData, pin_status: PhantomData },)+)
        }
    }
}

macro_rules! pwm_timer_basic {
    ($timx:ident, $TIMx:ty, $apbxenr:ident, $timxen:ident, [$($TimxChy:ident,)+], [$($x:ident,)+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $apbxenr,
            $timxen,
            |_| (),
            |_| (),
            |_| (),
            [$($TimxChy,)+],
            [$($x,)+]
        );
    }
}

macro_rules! pwm_timer_with_break {
    ($timx:ident, $TIMx:ty, $apbxenr:ident, $timxen:ident, [$($TimxChy:ident,)+], [$($x:ident,)+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $apbxenr,
            $timxen,
            |tim: &$TIMx| tim.egr.write(|w| w.ug().set_bit()),
            |tim: &$TIMx| tim.bdtr.write(|w| w.moe().set_bit()),
            |_| (),
            [$($TimxChy,)+],
            [$($x,)+]
        );
    }
}

macro_rules! pwm_timer_advanced {
    ($timx:ident, $TIMx:ty, $apbxenr:ident, $timxen:ident, [$($TimxChy:ident,)+], [$($x:ident,)+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $apbxenr,
            $timxen,
            |tim: &$TIMx| tim.egr.write(|w| w.ug().set_bit()),
            |tim: &$TIMx| tim.bdtr.write(|w| w.moe().set_bit()),
            |tim: &$TIMx| tim.smcr.write(|w| w),
            [$($TimxChy,)+],
            [$($x,)+]
        );
    }
}


macro_rules! pwm_channel_pin {
    ($TIMx:ident, $TimiChi:ident, $output_to_pxi:ident, $PXi:ident, $AFi:ident, $ccmrz_output:ident, $ocym:ident, $ocype:ident) => {
        impl PwmChannel<$TimiChi, NoPins> {
            pub fn $output_to_pxi(self, _p: $PXi<$AFi>) -> PwmChannel<$TimiChi, WithPins> {
                unsafe {
                    (*$TIMx::ptr()).$ccmrz_output().write(|w| w
                        // Select PWM Mode 1 for CHy
                        .$ocym().bits(0b0110)
                        // set pre-load enable so that updates to the duty cycle
                        // propagate but _not_ in the middle of a cycle.
                        .$ocype().set_bit()
                    );
                }
                PwmChannel { timx_chx: PhantomData, pin_status: PhantomData }
            }
        }

        impl PwmChannel<$TimiChi, WithPins> {
            pub fn $output_to_pxi(self, _p: $PXi<$AFi>) -> PwmChannel<$TimiChi, WithPins> {
                self
            }
        }
    }
}


macro_rules! pwm_pin_for_pwm_channel {
    ($TIMx:ident, $TimxChy:ty, $ccxe:ident, $ccrx:ident, $ccrq:ident) => {
        impl PwmPin for PwmChannel<$TimxChy, WithPins> {
            type Duty = u16;

            fn disable(&mut self) {
                unsafe {
                    &(*$TIMx::ptr()).ccer.modify(|_, w| w.$ccxe().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    &(*$TIMx::ptr()).ccer.modify(|_, w| w.$ccxe().set_bit());
                }
            }

            fn get_max_duty(&self) -> Self::Duty {
                unsafe {
                    // TODO: should the resolution just be stored in the channel rather than read?
                    // This would work if it changed, but isn't it the point that it can't be?
                    (*$TIMx::ptr()).arr.read().arr().bits()
                }
            }

            fn get_duty(&self) -> Self::Duty {
                unsafe {
                    // TODO: This could theoretically be passed into the PwmChannel struct
                    (*$TIMx::ptr()).$ccrx.read().$ccrq().bits()
                }
            }

            fn set_duty(&mut self, duty: Self::Duty) -> () {
                unsafe {
                    // TODO: This could theoretically be passed into the PwmChannel struct
                    // and it would then be safe to modify
                    &(*$TIMx::ptr()).$ccrx.modify(|_, w| w.$ccrq().bits(duty));
                }
            }
        }
    }
}


// TIM3

#[cfg(feature = "stm32f303")]
pwm_timer_basic!(
    tim3,
    TIM3,
    apb1enr,
    tim3en,
    [Tim3Ch1,Tim3Ch2,Tim3Ch3,Tim3Ch4,],
    [PwmChannel,PwmChannel,PwmChannel,PwmChannel,]
);

#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM3, Tim3Ch3, output_to_pc8, PC8, AF4, ccmr2_output, oc3m, oc3pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM3, Tim3Ch3, output_to_pe4, PE4, AF2, ccmr2_output, oc3m, oc3pe);

#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM3, Tim3Ch1, cc1e, ccr1, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM3, Tim3Ch2, cc2e, ccr2, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM3, Tim3Ch3, cc3e, ccr3, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM3, Tim3Ch4, cc4e, ccr4, ccr);


// TIM8

#[cfg(feature = "stm32f303")]
pwm_timer_advanced!(
    tim8,
    TIM8,
    apb2enr,
    tim8en,
    [Tim8Ch1,Tim8Ch2,Tim8Ch3,Tim8Ch4,],
    [PwmChannel,PwmChannel,PwmChannel,PwmChannel,]
);

#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM8, Tim8Ch3, output_to_pb9, PB9, AF10, ccmr2_output, oc3m, oc3pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM8, Tim8Ch3, output_to_pc8, PC8, AF4, ccmr2_output, oc3m, oc3pe);

#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM8, Tim8Ch1, cc1e, ccr1, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM8, Tim8Ch2, cc2e, ccr2, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM8, Tim8Ch3, cc3e, ccr3, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM8, Tim8Ch4, cc4e, ccr4, ccr);

// TIM16

// TODO: It would be nice if single channel timers didn't return a 1-tuple
// and instead just returned the single channel directly
#[cfg(feature = "stm32f303")]
pwm_timer_with_break!(
    tim16,
    TIM16,
    apb2enr,
    tim16en,
    [Tim16Ch1,],
    [PwmChannel,]
);

#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM16, Tim16Ch1, output_to_pa9, PA6, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM16, Tim16Ch1, output_to_pa12, PA12, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM16, Tim16Ch1, output_to_pb4, PB4, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM16, Tim16Ch1, output_to_pb8, PB8, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(TIM16, Tim16Ch1, output_to_pe0, PE0, AF4, ccmr1_output, oc1m, oc1pe);

#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM16, Tim16Ch1, cc1e, ccr1, ccr1);
