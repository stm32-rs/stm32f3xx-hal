use crate::pac::Interrupt;

/// Wrapper around interrupt types for the timers.
#[derive(Copy, Clone, Debug)]
// To make the type non-creatable but still accessable.
#[non_exhaustive]
pub struct InterruptTypes {
    /// Break Interrupt
    r#break: Interrupt,
    /// Update Interrupt
    update: Interrupt,
    /// Trigger and communication Interrupt
    trigger: Interrupt,
    /// Capture and compare interupt
    capture_compare: Interrupt,
}

// FIXME: Use conditional feature compilation to make this compialble for all chip families.
// TODO: Check if pub is needed.
pub(crate) const TIM2: Interrupt = Interrupt::TIM2;
cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f301")] {
        #[allow(unused)]
        pub(crate) const TIM3: Interrupt = Interrupt::TIM3_IRQ;
        #[allow(unused)]
        pub(crate) const TIM4: Interrupt = Interrupt::TIM4_IRQ;
    } else if #[cfg(any(feature = "svd-f303", feature = "svd-f302", feature = "svd-f373"))] {
        pub(crate) const TIM3: Interrupt = Interrupt::TIM3;
        pub(crate) const TIM4: Interrupt = Interrupt::TIM4;
    } else if #[cfg(any(feature = "svd-f3x4"))] {
        pub(crate) const TIM3: Interrupt = Interrupt::TIM3_IRQ;
    }
}
cfg_if::cfg_if! {
    if #[cfg(feature = "svd-f373")] {
        pub(crate) const TIM5: Interrupt = Interrupt::TIM5;
        pub(crate) const TIM12: Interrupt = Interrupt::TIM12;
        pub(crate) const TIM13: Interrupt = Interrupt::TIM13;
        pub(crate) const TIM14: Interrupt = Interrupt::TIM14;
        pub(crate) const TIM18: Interrupt = Interrupt::TIM18_DAC;
        pub(crate) const TIM19: Interrupt = Interrupt::TIM19;
    }
}
cfg_if::cfg_if! {
    if #[cfg(any(feature = "svd-f302", feature = "svd-f303"))] {
        pub(crate) const TIM6: Interrupt = Interrupt::TIM6_DACUNDER;
    } else if #[cfg(any(feature = "svd-f301", feature = "svd-f373", feature = "svd-f3x4"))] {
        pub(crate) const TIM6: Interrupt = Interrupt::TIM6_DAC1;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "svd-f302", feature = "svd-f303", feature = "svd-f373"))] {
        pub(crate) const TIM7: Interrupt = Interrupt::TIM7;
    } else if #[cfg(feature = "svd-f3x4")] {
        pub(crate) const TIM7: Interrupt = Interrupt::TIM7_DAC2;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "svd-f302", feature = "svd-f303", feature = "svd-f3x4"))] {
        pub(crate) const TIM15: Interrupt = Interrupt::TIM1_BRK_TIM15;
        pub(crate) const TIM16: Interrupt = Interrupt::TIM1_UP_TIM16;
        pub(crate) const TIM17: Interrupt = Interrupt::TIM1_TRG_COM_TIM17;

        pub(crate) const TIM1_TYPES: InterruptTypes = InterruptTypes {
            r#break: Interrupt::TIM1_BRK_TIM15,
            update: Interrupt::TIM1_UP_TIM16,
            trigger: Interrupt::TIM1_TRG_COM_TIM17,
            capture_compare: Interrupt::TIM1_CC,
        };
    } else if #[cfg(feature = "svd-f301")] {
        pub(crate) const TIM1: Interrupt = Interrupt::TIM1_CC;
        pub(crate) const TIM15: Interrupt = Interrupt::TIM15_IRQ;
        pub(crate) const TIM16: Interrupt = Interrupt::TIM16_IRQ;
        pub(crate) const TIM17: Interrupt = Interrupt::TIM17_IRQ;
    } else if #[cfg(feature = "svd-f373")] {
        pub(crate) const TIM15: Interrupt = Interrupt::TIM15;
        pub(crate) const TIM16: Interrupt = Interrupt::TIM16;
        pub(crate) const TIM17: Interrupt = Interrupt::TIM17;
    }
}

#[cfg(feature = "svd-f303")]
pub(crate) const TIM8_TYPES: InterruptTypes = InterruptTypes {
    r#break: Interrupt::TIM8_BRK,
    update: Interrupt::TIM8_UP,
    trigger: Interrupt::TIM8_TRG_COM,
    capture_compare: Interrupt::TIM8_CC,
};

#[cfg(all(feature = "svd-f303"))]
pub(crate) const TIM20_TYPES: InterruptTypes = InterruptTypes {
    r#break: Interrupt::TIM20_BRK,
    update: Interrupt::TIM20_UP,
    trigger: Interrupt::TIM20_TRG_COM,
    capture_compare: Interrupt::TIM20_CC,
};

macro_rules! single {
    ([ $($X:literal),+ ]) => {
        paste::paste! {
            $(
                impl crate::interrupts::InterruptNumber for crate::pac::[<TIM $X>] {
                    type Interrupt = crate::timer::interrupts::Interrupt;
                    const INTERRUPT: Self::Interrupt = crate::timer::interrupts::[<TIM $X>];
                }
            )+
        }
    };
}

#[allow(unused)]
macro_rules! multi {
    ([ $($X:literal),+ ]) => {
        paste::paste! {
            $(
                impl crate::interrupts::InterruptNumber for crate::pac::[<TIM $X>] {
                    type Interrupt = crate::timer::interrupts::InterruptTypes;
                    const INTERRUPT: Self::Interrupt = crate::timer::interrupts::[<TIM $X _TYPES>];
                }
            )+
        }
    };
}

#[cfg(feature = "svd-f301")]
single!([1, 2, 6, 15, 16, 17]);

#[cfg(feature = "svd-f302")]
single!([2, 3, 4, 6, 7, 15, 16, 17]);
#[cfg(feature = "svd-f302")]
multi!([1]);

#[cfg(feature = "svd-f303")]
single!([2, 3, 4, 6, 7, 15, 16, 17]);
#[cfg(feature = "svd-f303")]
multi!([1, 8, 20]);

#[cfg(feature = "svd-f373")]
single!([2, 3, 4, 5, 6, 7, 12, 13, 14, 15, 16, 17, 18, 19]);

#[cfg(feature = "svd-f3x4")]
single!([2, 3, 6, 7, 15, 16, 17]);
#[cfg(feature = "svd-f3x4")]
multi!([1]);
