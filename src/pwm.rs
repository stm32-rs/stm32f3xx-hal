use core::marker::PhantomData;
use crate::stm32::TIM8;
use embedded_hal::PwmPin;
use super::gpio::{AF4, AF10};
use super::gpio::gpioc::{PC8};
use super::gpio::gpiob::{PB9};

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
    pub(crate) pin_status: PhantomData<T>, // TODO: use PhantomData
}

macro_rules! pwm_channel_pin {
    ($TimiChi:ident, $output_to_pxi:ident, $PXi:ident, $AFi:ident) => {
        impl<T> PwmChannel<$TimiChi, T> {
            fn $output_to_pxi(self, _p: $PXi<$AFi>) -> PwmChannel<$TimiChi, WithPins> {
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
