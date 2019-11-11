use core::marker::PhantomData;
use crate::stm32::TIM8;
use embedded_hal::PwmPin;
use super::gpio::{AF4, AF10};
use super::gpio::gpioc::{PC8};
use super::gpio::gpiob::{PB9};
use crate::rcc::{Clocks, APB1, APB2};

//pub struct Tim1Ch1 {}
//pub struct Tim1Ch2 {}
//pub struct Tim1Ch3 {}
//pub struct Tim1Ch4 {}

//pub struct Tim3Ch1 {}
//pub struct Tim3Ch2 {}
//pub struct Tim3Ch3 {}
//pub struct Tim3Ch4 {}

//pub struct Tim8Ch1 {}
//pub struct Tim8Ch2 {}
pub struct Tim8Ch3 {}
//pub struct Tim8Ch4 {}

pub struct NoPins {}
pub struct WithPins {}

pub struct PwmChannel<X, T> {
    pub(crate) timx_chx: PhantomData<X>,
    pub(crate) pin_status: PhantomData<T>,
}

pub fn tim8(tim: TIM8, res: u16, freq: u16, apb2: &mut APB2, clocks: &Clocks) -> PwmChannel<Tim8Ch3, NoPins> {
    // Power the timer
    apb2.enr().modify(|_, w| w.tim8en().set_bit());

    // enable auto reload preloader
    tim.cr1.write(|w| w.arpe().set_bit());

    // Set the "resolution" of the duty cycle (ticks before restarting at 0)
    tim.arr.write(|w| w.arr().bits(res));
    // TODO: Use Hertz?
    // Set the pre-scaler
    tim.psc.write(|w| w.psc().bits(clocks.pclk2().0 as u16 / (res * freq)));

    // Macro friendly for later
    if true {
        // Make the settings reload immediately for TIM1/8
        tim.egr.write(|w| w.ug().set_bit());
    }

    tim.smcr.write(|w| w); // Reset the slave/master config
    tim.cr2.write(|w| w); // reset
    tim.ccmr1_output().write(|w| w
        // Select PWM Mode 1 for CH1/CH2
        .oc1m().bits(0b0110)
        .oc2m().bits(0b0110)
        // set pre-load enable so that updates to the duty cycle
        // propagate but _not_ in the middle of a cycle.
        .oc1pe().set_bit()
        .oc2pe().set_bit()
    );
    tim.ccmr2_output().write(|w| w
        // Select PWM Mode 1 for CH3/CH4
        .oc3m().bits(0b0110)
        .oc4m().bits(0b0110)
        // set pre-load enable so that updates to the duty cycle
        // propagate but _not_ in the middle of a cycle.
        .oc3pe().set_bit()
        .oc4pe().set_bit()
    );

    // Macro friendly for later
    if true {
        // Enable outputs (STM32 Break Timer Specific)
        tim.bdtr.write(|w| w.moe().set_bit());
    }

    // Enable the Timer
    tim.cr1.modify(|_, w| w.cen().set_bit());

    // TODO: This should return all four channels
    PwmChannel { timx_chx: PhantomData, pin_status: PhantomData }
}

macro_rules! pwm_channel_pin {
    ($TimiChi:ident, $output_to_pxi:ident, $PXi:ident, $AFi:ident) => {
        impl<T> PwmChannel<$TimiChi, T> {
            pub fn $output_to_pxi(self, _p: $PXi<$AFi>) -> PwmChannel<$TimiChi, WithPins> {
                PwmChannel { timx_chx: PhantomData, pin_status: PhantomData }
            }
        }
    }
}

pwm_channel_pin!(Tim8Ch3, output_to_pc8, PC8, AF4);
pwm_channel_pin!(Tim8Ch3, output_to_pb9, PB9, AF10);

impl PwmPin for PwmChannel<Tim8Ch3, WithPins> {
    type Duty = u16;

    fn disable(&mut self) {
        unsafe {
            &(*TIM8::ptr()).ccer.modify(|_, w| w.cc3e().clear_bit());
        }
    }

    fn enable(&mut self) {
        unsafe {
            &(*TIM8::ptr()).ccer.modify(|_, w| w.cc3e().set_bit());
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        unsafe {
            // TODO: should the resolution just be stored in the channel rather than read?
            // This would work if it changed, but isn't it the point that it can't be?
            (*TIM8::ptr()).arr.read().arr().bits()
        }
    }

    fn get_duty(&self) -> Self::Duty {
        unsafe {
            // TODO: This could theoretically be passed into the PwmChannel struct
            (*TIM8::ptr()).ccr3.read().ccr().bits()
        }
    }

    fn set_duty(&mut self, duty: Self::Duty) -> () {
        unsafe {
            // TODO: This could theoretically be passed into the PwmChannel struct
            // and it would then be safe to modify
            &(*TIM8::ptr()).ccr3.modify(|_, w| w.ccr().bits(duty));
        }
    }
}
