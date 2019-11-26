use core::marker::PhantomData;
use crate::stm32::{TIM2, TIM3, TIM8, TIM15, TIM16, TIM17};
use embedded_hal::PwmPin;
use crate::gpio::{AF1, AF2, AF3, AF4, AF9, AF10};
use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA9, PA10, PA12, PA13, PA15};
use crate::gpio::gpiob::{PB0, PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB14, PB15};
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};
use crate::gpio::gpiod::{PD3, PD4, PD6, PD7};
use crate::gpio::gpioe::{PE0, PE1, PE4, PE6, PE7, PE8, PE9};
use crate::gpio::gpiof::{PF9, PF10};
use crate::rcc::{Clocks};
use crate::time::Hertz;
use crate::stm32::{RCC};

//pub struct TIM1_CH1 {}
//pub struct TIM1_CH2 {}
//pub struct TIM1_CH3 {}
//pub struct TIM1_CH4 {}

pub struct TIM2_CH1 {}
pub struct TIM2_CH2 {}
pub struct TIM2_CH3 {}
pub struct TIM2_CH4 {}

pub struct TIM8_CH1 {}
pub struct TIM8_CH2 {}
pub struct TIM8_CH3 {}
pub struct TIM8_CH4 {}

pub struct TIM15_CH1 {}
pub struct TIM15_CH2 {}

pub struct TIM16_CH1 {}

pub struct TIM17_CH1 {}

pub struct NoPins {}
pub struct WithPins {}
pub struct WithNPins {}

pub struct PwmChannel<X, T> {
    timx_chy: PhantomData<X>,
    pin_status: PhantomData<T>,
}

macro_rules! pwm_timer_private {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $pclkz:ident, $timxen:ident, $trigger_update_event:expr, $enable_break_timer:expr, $reset_slave_master_config:expr, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pub fn $timx(tim: $TIMx, res: $res, freq: Hertz, clocks: &Clocks) -> ($(PwmChannel<$TIMx_CHy, NoPins>),+) {
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

            // Set the pre-scaler
            tim.psc.write(|w| w.psc().bits(
                (clocks.$pclkz().0 / res as u32 / freq.0) as u16
            ));

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
            ($($x { timx_chy: PhantomData, pin_status: PhantomData }),+)
        }
    }
}

macro_rules! pwm_timer_basic {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $pclkz:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $pclkz,
            $timxen,
            |_| (),
            |_| (),
            |_| (),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}

macro_rules! pwm_timer_with_break {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $pclkz:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $pclkz,
            $timxen,
            |tim: &$TIMx| tim.egr.write(|w| w.ug().set_bit()),
            |tim: &$TIMx| tim.bdtr.write(|w| w.moe().set_bit()),
            |_| (),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}

macro_rules! pwm_timer_advanced {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $pclkz:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $pclkz,
            $timxen,
            |tim: &$TIMx| tim.egr.write(|w| w.ug().set_bit()),
            |tim: &$TIMx| tim.bdtr.write(|w| w.moe().set_bit()),
            |tim: &$TIMx| tim.smcr.write(|w| w),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}


macro_rules! pwm_channel_pin {
    ($resulting_state:ident, $TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pzi:ident, $AFj:ident, $ccmrz_output:ident, $ocym:ident, $ocype:ident) => {
        impl PwmChannel<$TIMx_CHy, NoPins> {
            pub fn $output_to_pzx(self, _p: $Pzi<$AFj>) -> PwmChannel<$TIMx_CHy, $resulting_state> {
                unsafe {
                    (*$TIMx::ptr()).$ccmrz_output().write(|w| w
                        // Select PWM Mode 1 for CHy
                        .$ocym().bits(0b0110)
                        // set pre-load enable so that updates to the duty cycle
                        // propagate but _not_ in the middle of a cycle.
                        .$ocype().set_bit()
                    );
                }
                PwmChannel { timx_chy: PhantomData, pin_status: PhantomData }
            }
        }

        impl PwmChannel<$TIMx_CHy, $resulting_state> {
            pub fn $output_to_pzx(self, _p: $Pzi<$AFj>) -> PwmChannel<$TIMx_CHy, $resulting_state> {
                self
            }
        }
    }
}

macro_rules! pwm_pin_for_pwm_channel_private {
    ($state:ident, $TIMx:ident, $TIMx_CHy:ty, $res:ty, $ccx_enable:ident, $ccrx:ident, $ccrq:ident) => {
        impl PwmPin for PwmChannel<$TIMx_CHy, $state> {
            type Duty = $res;

            fn disable(&mut self) {
                unsafe {
                    &(*$TIMx::ptr()).ccer.modify(|_, w| w.$ccx_enable().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    &(*$TIMx::ptr()).ccer.modify(|_, w| w.$ccx_enable().set_bit());
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

macro_rules! pwm_pin_for_pwm_channel {
    ($TIMx:ident, $TIMx_CHy:ty, $res:ty, $ccxe:ident, $ccrx:ident, $ccrq:ident) => {
        pwm_pin_for_pwm_channel_private!(WithPins, $TIMx, $TIMx_CHy, $res, $ccxe, $ccrx, $ccrq);
    }
}

macro_rules! pwm_pin_for_pwm_n_channel {
    ($TIMx:ident, $TIMx_CHy:ty, $res:ty, $ccxe:ident, $ccxne:ident, $ccrx:ident, $ccrq:ident) => {
        pwm_pin_for_pwm_channel_private!(WithPins, $TIMx, $TIMx_CHy, $res, $ccxe, $ccrx, $ccrq);

        pwm_pin_for_pwm_channel_private!(WithNPins, $TIMx, $TIMx_CHy, $res, $ccxne, $ccrx, $ccrq);
    }
}


// TIM2

pwm_timer_basic!(
    tim2,
    TIM2,
    u32,
    apb1enr,
    pclk1,
    tim2en,
    [TIM2_CH1,TIM2_CH2,TIM2_CH3,TIM2_CH4],
    [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
);

// Channels
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH1, u32, cc1e, ccr1, ccr);
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH2, u32, cc2e, ccr2, ccr);
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH3, u32, cc3e, ccr3, ccr);
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH4, u32, cc4e, ccr4, ccr);

// Pins
pwm_channel_pin!(WithPins, TIM2, TIM2_CH1, output_to_pa0, PA0, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH1, output_to_pa5, PA5, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH1, output_to_pa15, PA15, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(any(feature = "stm32f303", feature = "stm32f302", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM2, TIM2_CH1, output_to_pd3, PD3, AF2, ccmr1_output, oc1m, oc1pe);

pwm_channel_pin!(WithPins, TIM2, TIM2_CH2, output_to_pa1, PA1, AF1, ccmr1_output, oc2m, oc2pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH2, output_to_pb3, PB3, AF1, ccmr1_output, oc2m, oc2pe);
#[cfg(any(feature = "stm32f303", feature = "stm32f302", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM2, TIM2_CH2, output_to_pd4, PD4, AF2, ccmr1_output, oc2m, oc2pe);

pwm_channel_pin!(WithPins, TIM2, TIM2_CH3, output_to_pa2, PA2, AF1, ccmr2_output, oc3m, oc3pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH3, output_to_pa9, PA9, AF10, ccmr2_output, oc3m, oc3pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH3, output_to_pb10, PB10, AF1, ccmr2_output, oc3m, oc3pe);
#[cfg(any(feature = "stm32f303", feature = "stm32f302", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM2, TIM2_CH3, output_to_pd7, PD7, AF2, ccmr2_output, oc3m, oc3pe);

pwm_channel_pin!(WithPins, TIM2, TIM2_CH4, output_to_pa3, PA3, AF1, ccmr2_output, oc4m, oc4pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH4, output_to_pa10, PA10, AF1, ccmr2_output, oc4m, oc4pe);
pwm_channel_pin!(WithPins, TIM2, TIM2_CH4, output_to_pb11, PB11, AF1, ccmr2_output, oc4m, oc4pe);
#[cfg(any(feature = "stm32f303", feature = "stm32f302", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM2, TIM2_CH4, output_to_pd6, PD6, AF2, ccmr2_output, oc4m, oc4pe);


// TIM3

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim3_common {
    () => {
        pub struct TIM3_CH1 {}
        pub struct TIM3_CH2 {}
        pub struct TIM3_CH3 {}
        pub struct TIM3_CH4 {}

        pwm_timer_basic!(
            tim3,
            TIM3,
            u16,
            apb1enr,
            pclk1,
            tim3en,
            [TIM3_CH1,TIM3_CH2,TIM3_CH3,TIM3_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH2, u16, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH3, u16, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH4, u16, cc4e, ccr4, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH1, output_to_pa6, PA6, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH1, output_to_pb4, PB4, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH2, output_to_pa4, PA4, AF2, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH2, output_to_pa7, PA7, AF2, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH2, output_to_pb5, PB5, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH3, output_to_pb0, PB0, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH4, output_to_pb1, PB1, AF2, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH4, output_to_pb7, PB7, AF10, ccmr2_output, oc4m, oc4pe);
    }
}

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim3_ext1 {
    () => {
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH1, output_to_pc6, PC6, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH2, output_to_pc7, PC7, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH3, output_to_pc8, PC8, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH4, output_to_pc9, PC9, AF2, ccmr2_output, oc4m, oc4pe);
    }
}

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim3_ext2 {
    () => {
        pwm_channel_pin!(WithPins, TIM3, TIM3_CH1, output_to_pe2, PE6, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH2, output_to_pe3, PE7, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH3, output_to_pe4, PE8, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM3, TIM3_CH4, output_to_pe5, PE9, AF2, ccmr2_output, oc4m, oc4pe);
    }
}


#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
tim3_common!();

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
tim3_ext1!();

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
tim3_ext2!();

#[cfg(feature = "stm32f373")]
pwm_channel_pin!(WithPins, TIM3, TIM3_CH2, output_to_pb0, PB0, AF10, ccmr1_output, oc2m, oc2pe);

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
pwm_channel_pin!(WithPins, TIM3, TIM3_CH3, output_to_pb6, PB6, AF10, ccmr2_output, oc3m, oc3pe);


// TIM8

#[cfg(feature = "stm32f303")]
pwm_timer_advanced!(
    tim8,
    TIM8,
    u16,
    apb2enr,
    pclk2,
    tim8en,
    [TIM8_CH1,TIM8_CH2,TIM8_CH3,TIM8_CH4],
    [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
);

// Channels
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH1, u16, cc1e, cc1ne, ccr1, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH2, u16, cc2e, cc2ne, ccr2, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH3, u16, cc3e, cc3ne, ccr3, ccr);
#[cfg(feature = "stm32f303")]
pwm_pin_for_pwm_channel!(TIM8, TIM8_CH4, u16, cc4e, ccr4, ccr);

// Pins
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(WithPins, TIM8, TIM8_CH3, output_to_pb9, PB9, AF10, ccmr2_output, oc3m, oc3pe);
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(WithPins, TIM8, TIM8_CH3, output_to_pc8, PC8, AF4, ccmr2_output, oc3m, oc3pe);

// NPins
#[cfg(feature = "stm32f303")]
pwm_channel_pin!(WithNPins, TIM8, TIM8_CH3, output_to_pb1, PB1, AF4, ccmr2_output, oc3m, oc3pe);


// TIM15

pwm_timer_with_break!(
    tim15,
    TIM15,
    u16,
    apb2enr,
    pclk2,
    tim15en,
    [TIM15_CH1,TIM15_CH2],
    [PwmChannel,PwmChannel]
);

// Channels
pwm_pin_for_pwm_n_channel!(TIM15, TIM15_CH1, u16, cc1e, cc1ne, ccr1, ccr1);
pwm_pin_for_pwm_channel!(TIM15, TIM15_CH2, u16, cc2e, ccr2, ccr2);

// Pins
pwm_channel_pin!(WithPins, TIM15, TIM15_CH1, output_to_pa2, PA2, AF9, ccmr1_output, oc1m, oc1pe);
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
pwm_channel_pin!(WithPins, TIM15, TIM15_CH1, output_to_pb6, PB6, AF9, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM15, TIM15_CH1, output_to_pb14, PB14, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM15, TIM15_CH1, output_to_pf9, PF9, AF3, ccmr1_output, oc1m, oc1pe);

pwm_channel_pin!(WithNPins, TIM15, TIM15_CH1, output_to_pa1, PA1, AF9, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithNPins, TIM15, TIM15_CH1, output_to_pb15, PB15, AF2, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM15, TIM15_CH2, output_to_pa3, PA3, AF9, ccmr1_output, oc2m, oc2pe);
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
pwm_channel_pin!(WithPins, TIM15, TIM15_CH2, output_to_pb7, PB7, AF9, ccmr1_output, oc2m, oc2pe);
pwm_channel_pin!(WithPins, TIM15, TIM15_CH2, output_to_pb15, PB15, AF2, ccmr1_output, oc2m, oc2pe);
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM15, TIM15_CH2, output_to_pf10, PF10, AF3, ccmr1_output, oc2m, oc2pe);



// TIM16

pwm_timer_with_break!(
    tim16,
    TIM16,
    u16,
    apb2enr,
    pclk2,
    tim16en,
    [TIM16_CH1],
    [PwmChannel]
);

// Channels
pwm_pin_for_pwm_n_channel!(TIM16, TIM16_CH1, u16, cc1e, cc1ne, ccr1, ccr1);

// Pins
pwm_channel_pin!(WithPins, TIM16, TIM16_CH1, output_to_pa9, PA6, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM16, TIM16_CH1, output_to_pa12, PA12, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM16, TIM16_CH1, output_to_pb4, PB4, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM16, TIM16_CH1, output_to_pb8, PB8, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM16, TIM16_CH1, output_to_pe0, PE0, AF4, ccmr1_output, oc1m, oc1pe);

pwm_channel_pin!(WithNPins, TIM16, TIM16_CH1, output_to_pa13, PA13, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithNPins, TIM16, TIM16_CH1, output_to_pb6, PB6, AF1, ccmr1_output, oc1m, oc1pe);


// TIM17

pwm_timer_with_break!(
    tim17,
    TIM17,
    u16,
    apb2enr,
    pclk2,
    tim17en,
    [TIM17_CH1],
    [PwmChannel]
);

// Channels
pwm_pin_for_pwm_n_channel!(TIM17, TIM17_CH1, u16, cc1e, cc1ne, ccr1, ccr1);

// Pins
pwm_channel_pin!(WithPins, TIM17, TIM17_CH1, output_to_pa7, PA7, AF1, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM17, TIM17_CH1, output_to_pb5, PB5, AF10, ccmr1_output, oc1m, oc1pe);
pwm_channel_pin!(WithPins, TIM17, TIM17_CH1, output_to_pb9, PB9, AF1, ccmr1_output, oc1m, oc1pe);
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
pwm_channel_pin!(WithPins, TIM17, TIM17_CH1, output_to_pe1, PE1, AF4, ccmr1_output, oc1m, oc1pe);

pwm_channel_pin!(WithNPins, TIM17, TIM17_CH1, output_to_pa13, PA13, AF1, ccmr1_output, oc1m, oc1pe);
