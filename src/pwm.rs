use core::marker::PhantomData;
use crate::stm32::TIM8;
use embedded_hal::PwmPin;
use super::gpio::{AF4, AF10};
use super::gpio::gpioc::{PC8};
use super::gpio::gpiob::{PB9};

//struct Tim1Ch1 {}
//struct Tim1Ch2 {}
//struct Tim1Ch3 {}
//struct Tim1Ch4 {}

//struct Tim3Ch1 {}
//struct Tim3Ch2 {}
//struct Tim3Ch3 {}
//struct Tim3Ch4 {}

//struct Tim8Ch1 {}
//struct Tim8Ch2 {}
struct Tim8Ch3 {}
//struct Tim8Ch4 {}

struct NoPins {}
struct WithPins {}

struct PwmChannel<X, T> {
    timx_chx: PhantomData<X>,
    pin_status: PhantomData<T>, // TODO: use PhantomData
}

impl<T> PwmChannel<Tim8Ch3, T> {
    // We just consume pc8, don't use it and return
    // a PwmChannel in a type state that's ready for usage
    // TODO: Ideally we could free the pin we accept
    fn output_to_pc8(self, _p: PC8<AF4>) -> PwmChannel<Tim8Ch3, WithPins> {
        PwmChannel { timx_chx: PhantomData, pin_status: PhantomData }
    }
}

impl<T> PwmChannel<Tim8Ch3, T> {
    fn output_to_pb9(self, _p: PB9<AF10>) -> PwmChannel<Tim8Ch3, WithPins> {
        PwmChannel { timx_chx: PhantomData, pin_status: PhantomData }
    }
}

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
