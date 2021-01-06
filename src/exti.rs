//! Configure external interrupts for the stm32f3xx.
//! See STM32f303 reference man section 14, including Table 82.

use crate::pac::{interrupt, EXTI, SYSCFG};

#[derive(Copy, Clone, Debug)]
pub enum GpioReg {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

impl GpioReg {
    /// See ref manual section 12.1.3: each reg has an associated value
    fn cr_val(&self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
            Self::D => 3,
            Self::E => 4,
            Self::F => 5,
            Self::G => 6,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Edge {
    Rising,
    Falling,
}

/// Reference the STM32F303 reference manual, section 14.2.5 for a `Functional description`
/// of the steps we accomplish here. See Table 82.
/// 0 - 15.
pub fn setup_line(exti: &mut EXTI, line: u8, edge: Edge) {
    // Configure the Trigger Selection bits of the Interrupt line (EXTI_RTSR and EXTI_FTSR)
    let rise_trigger = match edge {
        Edge::Rising => {
            // configure EXTI line to trigger on rising edge, disable trigger on falling edge.
            true
        }
        Edge::Falling => {
            // configure EXTI$line to trigger on falling edge, disable trigger on rising edge
            false
        }
    };

    match line {
        // External, eg GPIO interrupts
        0 => {
            // Configure the corresponding mask bit in the EXTI_IMR register.
            exti.imr1.modify(|_, w| w.mr0().unmasked());
            exti.rtsr1.modify(|_, w| w.tr0().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr0().bit(!rise_trigger));

            // Configure the enable and mask bits that control the NVIC IRQ channel mapped to the
            // EXTI so that an interrupt coming from one of the EXTI line can be correctly
            // acknowledged.
            // See reference manual Table 83.
        }
        // todo: DRY, in lieu of a working macro.
        1 => {
            exti.imr1.modify(|_, w| w.mr1().unmasked());
            exti.rtsr1.modify(|_, w| w.tr1().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr1().bit(!rise_trigger));
        }
        2 => {
            exti.imr1.modify(|_, w| w.mr2().unmasked());
            exti.rtsr1.modify(|_, w| w.tr2().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr2().bit(!rise_trigger));
        }
        3 => {
            exti.imr1.modify(|_, w| w.mr3().unmasked());
            exti.rtsr1.modify(|_, w| w.tr3().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr3().bit(!rise_trigger));
        }
        4 => {
            exti.imr1.modify(|_, w| w.mr4().unmasked());
            exti.rtsr1.modify(|_, w| w.tr4().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr4().bit(!rise_trigger));
        }
        5 => {
            exti.imr1.modify(|_, w| w.mr5().unmasked());
            exti.rtsr1.modify(|_, w| w.tr5().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr5().bit(!rise_trigger));
        }
        6 => {
            exti.imr1.modify(|_, w| w.mr6().unmasked());
            exti.rtsr1.modify(|_, w| w.tr6().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr6().bit(!rise_trigger));
        }
        7 => {
            exti.imr1.modify(|_, w| w.mr7().unmasked());
            exti.rtsr1.modify(|_, w| w.tr7().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr7().bit(!rise_trigger));
        }
        8 => {
            exti.imr1.modify(|_, w| w.mr8().unmasked());
            exti.rtsr1.modify(|_, w| w.tr8().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr8().bit(!rise_trigger));
        }
        9 => {
            exti.imr1.modify(|_, w| w.mr9().unmasked());
            exti.rtsr1.modify(|_, w| w.tr9().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr9().bit(!rise_trigger));
        }
        10 => {
            exti.imr1.modify(|_, w| w.mr10().unmasked());
            exti.rtsr1.modify(|_, w| w.tr10().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr10().bit(!rise_trigger));
        }
        11 => {
            exti.imr1.modify(|_, w| w.mr11().unmasked());
            exti.rtsr1.modify(|_, w| w.tr11().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr11().bit(!rise_trigger));
        }
        12 => {
            exti.imr1.modify(|_, w| w.mr12().unmasked());
            exti.rtsr1.modify(|_, w| w.tr12().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr12().bit(!rise_trigger));
        }
        13 => {
            exti.imr1.modify(|_, w| w.mr13().unmasked());
            exti.rtsr1.modify(|_, w| w.tr13().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr13().bit(!rise_trigger));
        }
        14 => {
            exti.imr1.modify(|_, w| w.mr14().unmasked());
            exti.rtsr1.modify(|_, w| w.tr14().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr14().bit(!rise_trigger));
        }
        // Internal interrupts
        15 => {
            exti.imr1.modify(|_, w| w.mr15().unmasked());
            exti.rtsr1.modify(|_, w| w.tr15().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr15().bit(!rise_trigger));
        }
        16 => {
            exti.imr1.modify(|_, w| w.mr16().unmasked());
            exti.rtsr1.modify(|_, w| w.tr16().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr16().bit(!rise_trigger));
        }
        17 => {
            exti.imr1.modify(|_, w| w.mr17().unmasked());
            exti.rtsr1.modify(|_, w| w.tr17().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr17().bit(!rise_trigger));
        }
        18 => {
            exti.imr1.modify(|_, w| w.mr18().unmasked());
            exti.rtsr1.modify(|_, w| w.tr18().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr18().bit(!rise_trigger));
        }
        19 => {
            exti.imr1.modify(|_, w| w.mr19().unmasked());
            exti.rtsr1.modify(|_, w| w.tr19().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr19().bit(!rise_trigger));
        }
        20 => {
            exti.imr1.modify(|_, w| w.mr20().unmasked());
            exti.rtsr1.modify(|_, w| w.tr20().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr20().bit(!rise_trigger));
        }
        21 => {
            exti.imr1.modify(|_, w| w.mr21().unmasked());
            exti.rtsr1.modify(|_, w| w.tr21().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr21().bit(!rise_trigger));
        }
        22 => {
            exti.imr1.modify(|_, w| w.mr22().unmasked());
            exti.rtsr1.modify(|_, w| w.tr22().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr22().bit(!rise_trigger));
        }
        23 => {
            exti.imr1.modify(|_, w| w.mr23().unmasked());
        }
        24 => {
            exti.imr1.modify(|_, w| w.mr24().unmasked());
        }
        25 => {
            exti.imr1.modify(|_, w| w.mr25().unmasked());
        }
        26 => {
            exti.imr1.modify(|_, w| w.mr26().unmasked());
        }
        27 => {
            exti.imr1.modify(|_, w| w.mr27().unmasked());
        }
        28 => {
            exti.imr1.modify(|_, w| w.mr28().unmasked());
        }
        29 => {
            exti.imr1.modify(|_, w| w.mr29().unmasked());
            exti.rtsr1.modify(|_, w| w.tr29().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr29().bit(!rise_trigger));
        }
        30 => {
            exti.imr1.modify(|_, w| w.mr30().unmasked());
            exti.rtsr1.modify(|_, w| w.tr30().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr30().bit(!rise_trigger));
        }
        31 => {
            exti.imr1.modify(|_, w| w.mr31().unmasked());
            exti.rtsr1.modify(|_, w| w.tr31().bit(rise_trigger));
            exti.ftsr1.modify(|_, w| w.tr31().bit(!rise_trigger));
        }
        32 => {
            exti.imr2.modify(|_, w| w.mr32().unmasked());
            exti.rtsr2.modify(|_, w| w.tr32().bit(rise_trigger));
            exti.ftsr2.modify(|_, w| w.tr32().bit(!rise_trigger));
        }
        33 => {
            exti.imr2.modify(|_, w| w.mr33().unmasked());
            exti.rtsr2.modify(|_, w| w.tr33().bit(rise_trigger));
            exti.ftsr2.modify(|_, w| w.tr33().bit(!rise_trigger));
        }
        34 => {
            exti.imr2.modify(|_, w| w.mr34().unmasked());
        }
        35 => {
            exti.imr2.modify(|_, w| w.mr35().unmasked());
        }

        _ => panic!("Interrupt line must be between 0 and 35."),
    }
}

// Setup functions to bind a push button that an interrupt line

/// Configure a GPIO to work with an interrupt line.
/// The `line` parameter corresponds to both the interrupt line, and the
/// GPIO number. Eg, `3` means `line3`, and `px3`.
/// Reference man section 12.1.3 of the datasheet for a breakdown by CR
/// and line.
pub fn setup_gpio(syscfg: &mut SYSCFG, gpio_reg: GpioReg, line: u8) {
    // Select this GPIO pin as source input for EXTI line external interrupt

    match line {
        0 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti0().bits(gpio_reg.cr_val()) }),
        1 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti1().bits(gpio_reg.cr_val()) }),
        2 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti2().bits(gpio_reg.cr_val()) }),
        3 => syscfg
            .exticr1
            .modify(|_, w| unsafe { w.exti3().bits(gpio_reg.cr_val()) }),
        4 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti4().bits(gpio_reg.cr_val()) }),
        5 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti5().bits(gpio_reg.cr_val()) }),
        6 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti6().bits(gpio_reg.cr_val()) }),
        7 => syscfg
            .exticr2
            .modify(|_, w| unsafe { w.exti7().bits(gpio_reg.cr_val()) }),
        8 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti8().bits(gpio_reg.cr_val()) }),
        9 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti9().bits(gpio_reg.cr_val()) }),
        10 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti10().bits(gpio_reg.cr_val()) }),
        11 => syscfg
            .exticr3
            .modify(|_, w| unsafe { w.exti11().bits(gpio_reg.cr_val()) }),
        12 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti12().bits(gpio_reg.cr_val()) }),
        13 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti13().bits(gpio_reg.cr_val()) }),
        14 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti14().bits(gpio_reg.cr_val()) }),
        15 => syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti15().bits(gpio_reg.cr_val()) }),
        _ => panic!("Can only setup GPIO interrupts on lines 0 - 15."),
    }
}

// From the Ref manual, page 296:
// The remaining lines are connected as follows:
// • EXTI line 16 is connected to the PVD output
// • EXTI line 17 is connected to the RTC Alarm event
// • EXTI line 18 is connected to USB Device FS wakeup event (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 19 is connected to RTC tamper and Timestamps
// • EXTI line 20 is connected to RTC wakeup timer
// • EXTI line 21 is connected to Comparator 1 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 22 is connected to Comparator 2 output
// • EXTI line 23 is connected to I2C1 wakeup
// • EXTI line 24 is connected to I2C2 wakeup (STM32F303xB/C/D/E, STM32F358xC and
// STM32F398xE devices)
// • EXTI line 25 is connected to USART1 wakeup
// • EXTI line 26 is connected to USART2 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// • EXTI line 27 is connected to I2C3 wakeup)(STM32F303xD/E and STM32F398
// devices)
// • EXTI line 28 is connected to USART3 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// • EXTI line 29 is connected to Comparator 3 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 30 is connected to Comparator 4 output
// • EXTI line 31 is connected to Comparator 5 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 32 is connected to Comparator 6 output
// • EXTI line 33 is connected to Comparator 7 output (STM32F303xB/C/D/E,
// STM32F358xC and STM32F398xE devices)
// • EXTI line 34 is connected to UART4 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// • EXTI line 35 is connected to UART5 wakeup (STM32F303xB/C/D/E, STM32F358xC
// and STM32F398xE devices)
// Note: EXTI lines 23, 24, 25, 26, 27, 28, 34 and 35 are internal
