use core::marker::PhantomData;
use crate::stm32::{TIM2, TIM15, TIM16, TIM17};
use embedded_hal::PwmPin;

use crate::gpio::{AF1, AF2, AF9, AF10};
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::AF3;
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::{AF4, AF6};
#[cfg(any(feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::AF5;
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f373", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::AF11;

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA5, PA6, PA7, PA9, PA10, PA12, PA13, PA15};
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioa::PA4;
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f373", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioa::PA8;
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioa::PA11;
#[cfg(any(feature = "stm32f303", feature = "stm32f373", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioa::PA14;

use crate::gpio::gpiob::{PB3, PB4, PB5, PB6, PB8, PB9, PB10, PB11, PB14, PB15};
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiob::{PB0, PB1};
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiob::PB7;
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiob::PB13;

#[cfg(any(feature = "stm32f334", feature = "stm32f373", feature = "stm32f398"))]
use crate::gpio::gpioc::{PC0, PC1, PC2, PC3};
#[cfg(feature = "stm32f373")]
use crate::gpio::gpioc::PC4;
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};
#[cfg(any(feature = "stm32f303", feature = "stm32f373", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioc::{PC10, PC11, PC12};
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioc::PC13;

#[cfg(any(feature = "stm32f303", feature = "stm32f302", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiod::{PD3, PD4, PD6, PD7};
#[cfg(feature = "stm32f373")]
use crate::gpio::gpiod::PD0;
#[cfg(any(feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiod::PD1;
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiod::{PD12, PD13, PD14, PD15};

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpioe::{PE0, PE1, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14};
#[cfg(feature = "stm32f398")]
use crate::gpio::gpioe::{PE2, PE4};

#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiof::PF0;
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiof::PF6;
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiof::PF9;
#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
use crate::gpio::gpiof::PF10;

use crate::rcc::{Clocks};
use crate::time::Hertz;
use crate::stm32::{RCC};

/// Output Compare Channel 1 of Timer 1 (type state)
pub struct TIM2_CH1 {}
/// Output Compare Channel 2 of Timer 1 (type state)
pub struct TIM2_CH2 {}
/// Output Compare Channel 3 of Timer 1 (type state)
pub struct TIM2_CH3 {}
/// Output Compare Channel 4 of Timer 1 (type state)
pub struct TIM2_CH4 {}

/// Output Compare Channel 1 of Timer 15 (type state)
pub struct TIM15_CH1 {}
/// Output Compare Channel 2 of Timer 15 (type state)
pub struct TIM15_CH2 {}

/// Output Compare Channel 1 of Timer 16 (type state)
pub struct TIM16_CH1 {}

/// Output Compare Channel 1 of Timer 17 (type state)
pub struct TIM17_CH1 {}

/// Type state used to represent a channel that has no pins yet
pub struct NoPins {}
/// Type state used to represent a channel is using regular pins
pub struct WithPins {}
/// Type state used to represent a channel is using (only) complementary pins
pub struct WithNPins {}

/// Representation of a Channel for an abritary timer channel,
/// that also holds a type state for whether or not this channel
/// is using any pins yet.
///
/// If there are no pins supplied, it cannot be enabled.
pub struct PwmChannel<X, T> {
    timx_chy: PhantomData<X>,
    pin_status: PhantomData<T>,
}

macro_rules! pwm_timer_private {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $apbxrstr:ident, $pclkz:ident, $timxrst:ident, $timxen:ident, $enable_break_timer:expr, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        /// Create one or more output channels from a TIM Peripheral
        /// This function requires the maximum resolution of the duty cycle,
        /// the period of the PWM signal and the frozen clock configuration.
        ///
        /// The resolution should be chosen to offer sufficient steps against
        /// your target peripheral.  For example, a servo that can turn from
        /// 0 degrees (2% duty cycle) to 180 degrees (4% duty cycle) might choose
        /// a resolution of 9000.  This allows the servo to be set in increments
        /// of exactly one degree.
        pub fn $timx(tim: $TIMx, res: $res, freq: Hertz, clocks: &Clocks) -> ($(PwmChannel<$TIMx_CHy, NoPins>),+) {
            // Power the timer and reset it to ensure a clean state
            // We use unsafe here to abstract away this implementation detail
            // Justification: It is safe because only scopes with mutable references
            // to TIMx should ever modify this bit.
            unsafe {
                (*RCC::ptr()).$apbxenr.modify(|_, w| w.$timxen().set_bit());
                (*RCC::ptr()).$apbxrstr.modify(|_, w| w.$timxrst().set_bit());
                (*RCC::ptr()).$apbxrstr.modify(|_, w| w.$timxrst().clear_bit());
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
            // TODO: This is repeated in the timer/pwm module.
            // It might make sense to move into the clocks as a crate-only property.
            // TODO: ppre1 is used in timer.rs (never ppre2), should this be dynamic?
            let clock_freq = clocks.$pclkz().0 * if clocks.ppre1() == 1 { 1 } else { 2 };
            tim.psc.write(|w| w.psc().bits(
                (clock_freq / res as u32 / freq.0) as u16
            ));

            // Make the settings reload immediately
            tim.egr.write(|w| w.ug().set_bit());

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
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $apb1rstr:ident, $pclkz:ident, $timxrst:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $apb1rstr,
            $pclkz,
            $timxrst,
            $timxen,
            |_| (),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}

macro_rules! pwm_timer_with_break {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $apbxrstr:ident, $pclkz:ident, $timxrst:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $apbxrstr,
            $pclkz,
            $timxrst,
            $timxen,
            |tim: &$TIMx| tim.bdtr.write(|w| w.moe().set_bit()),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}


macro_rules! pwm_channel_pin {
    ($resulting_state:ident, $TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pzi:ident, $AFj:ident, $ccmrz_output:ident, $ocym:ident, $ocype:ident) => {
        impl PwmChannel<$TIMx_CHy, NoPins> {
            /// Output to a specific pin from a channel that does not yet have
            /// any pins.  This channel cannot be enabled until this method
            /// is called.
            ///
            /// The pin is consumed and cannot be returned.
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
            /// Output to a specific pin from a channel is already configured
            /// with output pins.  There is no limit to the number of pins that
            /// can be used (as long as they are compatible).
            ///
            /// The pin is consumed and cannot be returned.
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
                    (*$TIMx::ptr()).ccer.modify(|_, w| w.$ccx_enable().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    (*$TIMx::ptr()).ccer.modify(|_, w| w.$ccx_enable().set_bit());
                }
            }

            fn get_max_duty(&self) -> Self::Duty {
                unsafe {
                    (*$TIMx::ptr()).arr.read().arr().bits()
                }
            }

            fn get_duty(&self) -> Self::Duty {
                unsafe {
                    (*$TIMx::ptr()).$ccrx.read().$ccrq().bits()
                }
            }

            fn set_duty(&mut self, duty: Self::Duty) -> () {
                unsafe {
                    (*$TIMx::ptr()).$ccrx.modify(|_, w| w.$ccrq().bits(duty));
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


// TIM1

#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim1_common {
    () => {
        use crate::stm32::TIM1;

        /// Output Compare Channel 1 of Timer 1 (type state)
        pub struct TIM1_CH1 {}
        /// Output Compare Channel 2 of Timer 1 (type state)
        pub struct TIM1_CH2 {}
        /// Output Compare Channel 3 of Timer 1 (type state)
        pub struct TIM1_CH3 {}
        /// Output Compare Channel 4 of Timer 1 (type state)
        pub struct TIM1_CH4 {}

        pwm_timer_with_break!(
            tim1,
            TIM1,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim1rst,
            tim1en,
            [TIM1_CH1,TIM1_CH2,TIM1_CH3,TIM1_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_n_channel!(TIM1, TIM1_CH1, u16, cc1e, cc1ne, ccr1, ccr);
        pwm_pin_for_pwm_n_channel!(TIM1, TIM1_CH2, u16, cc2e, cc2ne, ccr2, ccr);
        pwm_pin_for_pwm_n_channel!(TIM1, TIM1_CH3, u16, cc3e, cc3ne, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM1, TIM1_CH4, u16, cc4e, ccr4, ccr);

        //Pins
        pwm_channel_pin!(WithPins, TIM1, TIM1_CH1, output_to_pa8, PA8, AF6, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH1, output_to_pa7, PA7, AF6, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH1, output_to_pa11, PA11, AF6, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH1, output_to_pb13, PB13, AF6, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH1, output_to_pc13, PC13, AF4, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH2, output_to_pa9, PA9, AF6, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH2, output_to_pa12, PA12, AF6, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH2, output_to_pb0, PB0, AF6, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH2, output_to_pb14, PB14, AF6, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH3, output_to_pa10, PA10, AF6, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH3, output_to_pb1, PB1, AF6, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH3, output_to_pb15, PB15, AF4, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH3, output_to_pf0, PF0, AF6, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH3, output_to_pa11, PA11, AF11, ccmr2_output, oc4m, oc4pe);
    }
}

#[cfg(any(feature = "stm32f334", feature = "stm32f398"))]
macro_rules! tim1_ext1 {
    () => {
        pwm_channel_pin!(WithPins, TIM1, TIM1_CH1, output_to_pc0, PC0, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH2, output_to_pc1, PC1, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH3, output_to_pc2, PC2, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH4, output_to_pc3, PC3, AF2, ccmr2_output, oc4m, oc4pe);
    }
}

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim1_ext2 {
    () => {
        pwm_channel_pin!(WithPins, TIM1, TIM1_CH1, output_to_pe9, PE9, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH1, output_to_pe8, PE8, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH2, output_to_pe11, PE11, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH2, output_to_pe10, PE10, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH3, output_to_pe13, PE13, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithNPins, TIM1, TIM1_CH3, output_to_pe12, PE12, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM1, TIM1_CH3, output_to_pe14, PE14, AF2, ccmr2_output, oc4m, oc4pe);
    }
}

// TODO: stm32f301 has TIM1 with ext1
#[cfg(any(feature = "stm32f318", feature = "stm32f302", feature = "stm32f303", feature = "stm32f334", feature = "stm32f358", feature = "stm32f398"))]
tim1_common!();

#[cfg(any(feature = "stm32f334", feature = "stm32f398"))]
tim1_ext1!();

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
tim1_ext2!();


// TIM2

pwm_timer_basic!(
    tim2,
    TIM2,
    u32,
    apb1enr,
    apb1rstr,
    pclk1,
    tim2rst,
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
        use crate::stm32::TIM3;

        /// Output Compare Channel 1 of Timer 3 (type state)
        pub struct TIM3_CH1 {}
        /// Output Compare Channel 2 of Timer 3 (type state)
        pub struct TIM3_CH2 {}
        /// Output Compare Channel 3 of Timer 3 (type state)
        pub struct TIM3_CH3 {}
        /// Output Compare Channel 4 of Timer 3 (type state)
        pub struct TIM3_CH4 {}

        pwm_timer_basic!(
            tim3,
            TIM3,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim3rst,
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


// TIM4

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim4 {
    () => {
        use crate::stm32::TIM4;

        /// Output Compare Channel 1 of Timer 4 (type state)
        pub struct TIM4_CH1 {}
        /// Output Compare Channel 2 of Timer 4 (type state)
        pub struct TIM4_CH2 {}
        /// Output Compare Channel 3 of Timer 4 (type state)
        pub struct TIM4_CH3 {}
        /// Output Compare Channel 4 of Timer 4 (type state)
        pub struct TIM4_CH4 {}

        pwm_timer_basic!(
            tim4,
            TIM4,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim4rst,
            tim4en,
            [TIM4_CH1,TIM4_CH2,TIM4_CH3,TIM4_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH2, u16, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH3, u16, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH4, u16, cc4e, ccr4, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH1, output_to_pa11, PA11, AF10, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH1, output_to_pb6, PB6, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH1, output_to_pd12, PD12, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM4, TIM4_CH2, output_to_pa12, PA12, AF10, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH2, output_to_pb7, PB7, AF2, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH2, output_to_pd13, PD13, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM4, TIM4_CH3, output_to_pa13, PA13, AF10, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH3, output_to_pb8, PB8, AF2, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH3, output_to_pd14, PD14, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM4, TIM4_CH4, output_to_pb9, PB9, AF2, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH4, output_to_pd15, PD15, AF2, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM4, TIM4_CH4, output_to_pf6, PF6, AF2, ccmr2_output, oc4m, oc4pe);
    }
}

#[cfg(any(feature = "stm32f302", feature = "stm32f303", feature = "stm32f373", feature = "stm32f378", feature = "stm32f358", feature = "stm32f398"))]
tim4!();


// TIM5

#[cfg(feature = "stm32f373")]
macro_rules! tim5 {
    () => {
        use crate::stm32::TIM5;

        /// Output Compare Channel 1 of Timer 5 (type state)
        pub struct TIM5_CH1 {}
        /// Output Compare Channel 2 of Timer 5 (type state)
        pub struct TIM5_CH2 {}
        /// Output Compare Channel 3 of Timer 5 (type state)
        pub struct TIM5_CH3 {}
        /// Output Compare Channel 4 of Timer 5 (type state)
        pub struct TIM5_CH4 {}

        pwm_timer_basic!(
            tim5,
            TIM5,
            u32,
            apb1enr,
            apb1rstr,
            pclk1,
            tim5rst,
            tim5en,
            [TIM5_CH1,TIM5_CH2,TIM5_CH3,TIM5_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH1, u32, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH2, u32, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH3, u32, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH4, u32, cc4e, ccr4, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH1, output_to_pa0, PA0, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH1, output_to_pa8, PA8, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH1, output_to_pc0, PC0, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM5, TIM5_CH2, output_to_pa1, PA1, AF2, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH2, output_to_pa11, PA11, AF2, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH2, output_to_pc1, PC1, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM5, TIM5_CH3, output_to_pa2, PA2, AF2, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH3, output_to_pa12, PA12, AF2, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH3, output_to_pc2, PC2, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM5, TIM5_CH4, output_to_pa3, PA3, AF2, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH4, output_to_pa13, PA13, AF2, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM5, TIM5_CH4, output_to_pc3, PC3, AF2, ccmr2_output, oc4m, oc4pe);
    }
}

// TODO: This timer is also present in stm32f378
#[cfg(any(feature = "stm32f373"))]
tim5!();


// TIM8

#[cfg(any(feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim8 {
    () => {
        use crate::stm32::TIM8;

        /// Output Compare Channel 1 of Timer 8 (type state)
        pub struct TIM8_CH1 {}
        /// Output Compare Channel 2 of Timer 8 (type state)
        pub struct TIM8_CH2 {}
        /// Output Compare Channel 3 of Timer 8 (type state)
        pub struct TIM8_CH3 {}
        /// Output Compare Channel 4 of Timer 8 (type state)
        pub struct TIM8_CH4 {}

        pwm_timer_with_break!(
            tim8,
            TIM8,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim8rst,
            tim8en,
            [TIM8_CH1,TIM8_CH2,TIM8_CH3,TIM8_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH1, u16, cc1e, cc1ne, ccr1, ccr);
        pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH2, u16, cc2e, cc2ne, ccr2, ccr);
        pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH3, u16, cc3e, cc3ne, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM8, TIM8_CH4, u16, cc4e, ccr4, ccr);

        //Pins
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH1, output_to_pa15, PA15, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH1, output_to_pb6, PB6, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH1, output_to_pc6, PC6, AF4, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH1, output_to_pa7, PA7, AF4, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH1, output_to_pb3, PB3, AF4, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH1, output_to_pc10, PC10, AF4, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM8, TIM8_CH2, output_to_pa14, PA14, AF5, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH2, output_to_pb8, PB8, AF10, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH2, output_to_pc7, PC7, AF4, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH2, output_to_pb0, PB0, AF4, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH2, output_to_pb4, PB4, AF4, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH2, output_to_pc11, PC11, AF4, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM8, TIM8_CH3, output_to_pb9, PB9, AF10, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH3, output_to_pc8, PC8, AF4, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH3, output_to_pb1, PB1, AF4, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH3, output_to_pb5, PB5, AF3, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithNPins, TIM8, TIM8_CH3, output_to_pc12, PC12, AF4, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM8, TIM8_CH3, output_to_pc9, PC9, AF4, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM8, TIM8_CH3, output_to_pd1, PD1, AF4, ccmr2_output, oc4m, oc4pe);
    }
}

#[cfg(any(feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
tim8!();


// TIM12

#[cfg(feature = "stm32f373")]
macro_rules! tim12 {
    () => {
        use crate::stm32::TIM12;

        /// Output Compare Channel 1 of Timer 12 (type state)
        pub struct TIM12_CH1 {}
        /// Output Compare Channel 2 of Timer 12 (type state)
        pub struct TIM12_CH2 {}
        /// Output Compare Channel 3 of Timer 12 (type state)
        pub struct TIM12_CH3 {}
        /// Output Compare Channel 4 of Timer 12 (type state)
        pub struct TIM12_CH4 {}

        pwm_timer_basic!(
            tim12,
            TIM12,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim12rst,
            tim12en,
            [TIM12_CH1,TIM12_CH2],
            [PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM12, TIM12_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM12, TIM12_CH2, u16, cc2e, ccr2, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM12, TIM12_CH1, output_to_pa4, PA4, AF10, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM12, TIM12_CH1, output_to_pa14, PA14, AF10, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM12, TIM12_CH1, output_to_pb14, PB14, AF10, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM12, TIM12_CH2, output_to_pa5, PA5, AF10, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM12, TIM12_CH2, output_to_pa15, PA15, AF10, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM12, TIM12_CH2, output_to_pb15, PB15, AF10, ccmr1_output, oc2m, oc2pe);
    }
}

// TODO: This timer is also present in stm32f378
#[cfg(feature = "stm32f373")]
tim12!();


// TIM13

#[cfg(feature = "stm32f373")]
macro_rules! tim13 {
    () => {
        use crate::stm32::TIM13;

        /// Output Compare Channel 1 of Timer 13 (type state)
        pub struct TIM13_CH1 {}
        /// Output Compare Channel 2 of Timer 13 (type state)
        pub struct TIM13_CH2 {}
        /// Output Compare Channel 3 of Timer 13 (type state)
        pub struct TIM13_CH3 {}
        /// Output Compare Channel 4 of Timer 13 (type state)
        pub struct TIM13_CH4 {}

        pwm_timer_basic!(
            tim13,
            TIM13,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim13rst,
            tim13en,
            [TIM13_CH1],
            [PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM13, TIM13_CH1, u16, cc1e, ccr1, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM13, TIM13_CH1, output_to_pa6, PA6, AF9, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM13, TIM13_CH1, output_to_pa9, PA9, AF2, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM13, TIM13_CH1, output_to_pb3, PB3, AF9, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM13, TIM13_CH1, output_to_pc4, PC4, AF2, ccmr1_output, oc1m, oc1pe);
    }
}

#[cfg(feature = "stm32f373")]
tim13!();


// TIM14

#[cfg(feature = "stm32f373")]
macro_rules! tim14 {
    () => {
        use crate::stm32::TIM14;

        /// Output Compare Channel 1 of Timer 14 (type state)
        pub struct TIM14_CH1 {}
        /// Output Compare Channel 2 of Timer 14 (type state)
        pub struct TIM14_CH2 {}
        /// Output Compare Channel 3 of Timer 14 (type state)
        pub struct TIM14_CH3 {}
        /// Output Compare Channel 4 of Timer 14 (type state)
        pub struct TIM14_CH4 {}

        pwm_timer_basic!(
            tim14,
            TIM14,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim14rst,
            tim14en,
            [TIM14_CH1],
            [PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM14, TIM14_CH1, u16, cc1e, ccr1, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM14, TIM14_CH1, output_to_pa5, PA5, AF9, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM14, TIM14_CH1, output_to_pa7, PA7, AF9, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM14, TIM14_CH1, output_to_pa10, PA10, AF9, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM14, TIM14_CH1, output_to_pf9, PF9, AF2, ccmr1_output, oc1m, oc1pe);
    }
}

// TODO: This timer is also present in stm32f378
#[cfg(feature = "stm32f373")]
tim14!();


// TIM15

pwm_timer_with_break!(
    tim15,
    TIM15,
    u16,
    apb2enr,
    apb2rstr,
    pclk2,
    tim15rst,
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
    apb2rstr,
    pclk2,
    tim16rst,
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
    apb2rstr,
    pclk2,
    tim17rst,
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


// TIM19

#[cfg(feature = "stm32f373")]
macro_rules! tim19 {
    () => {
        use crate::stm32::TIM19;

        /// Output Compare Channel 1 of Timer 19 (type state)
        pub struct TIM19_CH1 {}
        /// Output Compare Channel 2 of Timer 19 (type state)
        pub struct TIM19_CH2 {}
        /// Output Compare Channel 3 of Timer 19 (type state)
        pub struct TIM19_CH3 {}
        /// Output Compare Channel 4 of Timer 19 (type state)
        pub struct TIM19_CH4 {}

        pwm_timer_basic!(
            tim19,
            TIM19,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim19rst,
            tim19en,
            [TIM19_CH1,TIM19_CH2,TIM19_CH3,TIM19_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH2, u16, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH3, u16, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH4, u16, cc4e, ccr4, ccr);

        // Pins
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH1, output_to_pa0, PA0, AF11, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH1, output_to_pb6, PB6, AF11, ccmr1_output, oc1m, oc1pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH1, output_to_pc10, PC10, AF2, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithPins, TIM19, TIM19_CH2, output_to_pa1, PA1, AF11, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH2, output_to_pb7, PB7, AF11, ccmr1_output, oc2m, oc2pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH2, output_to_pc11, PC11, AF2, ccmr1_output, oc2m, oc2pe);

        pwm_channel_pin!(WithPins, TIM19, TIM19_CH3, output_to_pa2, PA2, AF11, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH3, output_to_pb8, PB8, AF11, ccmr2_output, oc3m, oc3pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH3, output_to_pc12, PC12, AF2, ccmr2_output, oc3m, oc3pe);

        pwm_channel_pin!(WithPins, TIM19, TIM19_CH4, output_to_pa3, PA3, AF11, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH4, output_to_pb9, PB9, AF11, ccmr2_output, oc4m, oc4pe);
        pwm_channel_pin!(WithPins, TIM19, TIM19_CH4, output_to_pd0, PD0, AF2, ccmr2_output, oc4m, oc4pe);
    }
}

// TODO: This timer is also present in stm32f378
#[cfg(feature = "stm32f373")]
tim19!();


// TIM20
//
#[cfg(feature = "stm32f398")]
macro_rules! tim20 {
    () => {
        use crate::stm32::TIM20;

        /// Output Compare Channel 1 of Timer 20 (type state)
        pub struct TIM20_CH1 {}
        /// Output Compare Channel 2 of Timer 20 (type state)
        pub struct TIM20_CH2 {}
        /// Output Compare Channel 3 of Timer 20 (type state)
        pub struct TIM20_CH3 {}
        /// Output Compare Channel 4 of Timer 20 (type state)
        pub struct TIM20_CH4 {}

        pwm_timer_basic!(
            tim20,
            TIM20,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim20rst,
            tim20en,
            [TIM20_CH1,TIM20_CH2,TIM20_CH3,TIM20_CH4],
            [PwmChannel,PwmChannel,PwmChannel,PwmChannel]
        );

        // Channels
        // TODO: stm32f3 doesn't suppport registers for all 4 channels
        pwm_pin_for_pwm_n_channel!(TIM20, TIM20_CH1, u16, cc1e, cc1ne, ccr1, ccr1);

        //Pins
        pwm_channel_pin!(WithPins, TIM20, TIM20_CH1, output_to_pe2, PE2, AF6, ccmr1_output, oc1m, oc1pe);

        pwm_channel_pin!(WithNPins, TIM20, TIM20_CH1, output_to_pe4, PE4, AF6, ccmr1_output, oc1m, oc1pe);
    }
}

#[cfg(feature = "stm32f398")]
tim20!();
