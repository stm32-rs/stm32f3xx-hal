//! This file provides an alternative way to set clocks than in the `rcc` modules`,
//! which may be less error prone, and is more opaque. It works by setting
//! scalers etc, then calculating frequencies, instead of solving for a set of scalers
//! that meet specified frequeincies.
//!
//! See STM32CubeIDE for an interactive editor that's very useful for seeing what
//! settings are available, and validating them.
//!
//! See Figure 15 of theF303 reference manual for a non-interactive visualization.

use crate::pac::{FLASH, RCC};

/// Speed out of limits.
pub struct SpeedError {}

/// Calculated clock speeds. All in Mhz
#[derive(Clone, Debug)]
pub struct Speeds {
    sysclk: f32,
    hclk: f32,    // AHB bus, core, memory and DMA
    systick: f32, // Cortex System Timer
    fclk: f32,    // FCLK Cortex clock
    pclk1: f32,   // APB1 peripheral clocks
    timer1: f32,  // APB1 timer clocks
    pclk2: f32,   // APB2 peripheral clocks
    timer2: f32,  // APB2 timer clocks
    usb: f32,
}

/// Is a set of speeds valid?
#[derive(Clone, Copy)]
pub enum Validation {
    Valid,
    NotValid,
}

#[derive(Clone, Copy)]
pub enum PllSrc {
    HsiDiv2 = 0b00,
    Hsi = 0b01,
    Hse = 0b10,
}

#[derive(Clone, Copy)]
pub enum InputSrc {
    Hsi,
    Hse,
    Pll(PllSrc),
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// RCC_cfgr2
pub enum Prediv {
    Div1 = 0b0000,
    Div2 = 0b0001,
    Div3 = 0b0010,
    Div4 = 0b0011,
    Div5 = 0b0100,
    Div6 = 0b0101,
    Div7 = 0b0110,
    Div8 = 0b0111,
    Div9 = 0b1000,
    Div10 = 0b1001,
    Div11 = 0b1010,
    Div12 = 0b1011,
    Div13 = 0b1100,
    Div14 = 0b1101,
    Div15 = 0b1110,
    Div16 = 0b1111,
}

impl Prediv {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div3 => 3,
            Self::Div4 => 4,
            Self::Div5 => 5,
            Self::Div6 => 6,
            Self::Div7 => 7,
            Self::Div8 => 8,
            Self::Div9 => 9,
            Self::Div10 => 10,
            Self::Div11 => 11,
            Self::Div12 => 12,
            Self::Div13 => 13,
            Self::Div14 => 14,
            Self::Div15 => 15,
            Self::Div16 => 16,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PllMul {
    Mul2 = 0b0000,
    Mul3 = 0b0001,
    Mul4 = 0b0010,
    Mul5 = 0b0011,
    Mul6 = 0b0100,
    Mul7 = 0b0101,
    Mul8 = 0b0110,
    Mul9 = 0b0111,
    Mul10 = 0b1000,
    Mul11 = 0b1001,
    Mul12 = 0b1010,
    Mul13 = 0b1011,
    Mul14 = 0b1100,
    Mul15 = 0b1101,
    Mul16 = 0b1110,
}

impl PllMul {
    pub fn value(&self) -> u8 {
        match self {
            Self::Mul2 => 2,
            Self::Mul3 => 3,
            Self::Mul4 => 4,
            Self::Mul5 => 5,
            Self::Mul6 => 6,
            Self::Mul7 => 7,
            Self::Mul8 => 8,
            Self::Mul9 => 9,
            Self::Mul10 => 10,
            Self::Mul11 => 11,
            Self::Mul12 => 12,
            Self::Mul13 => 13,
            Self::Mul14 => 14,
            Self::Mul15 => 15,
            Self::Mul16 => 16,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Division factor for the AHB clock.
pub enum HclkPrescaler {
    Div1 = 0b0000,
    Div2 = 0b1000,
    Div4 = 0b1001,
    Div8 = 0b1010,
    Div16 = 0b1011,
    Div64 = 0b1100,
    Div128 = 0b1101,
    Div256 = 0b1110,
    Div512 = 0b1111,
}

impl HclkPrescaler {
    pub fn value(&self) -> u16 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
            Self::Div64 => 64,
            Self::Div128 => 128,
            Self::Div256 => 256,
            Self::Div512 => 512,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum ApbPrescaler {
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
}

impl ApbPrescaler {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum UsbPrescaler {
    Div1_5 = 0,
    Div1 = 1,
}

impl UsbPrescaler {
    // Can't pass u8 to the single-bit field in sv2rust; need bool.
    pub fn bit(&self) -> bool {
        match self {
            Self::Div1_5 => false,
            Self::Div1 => true,
        }
    }

    pub fn value(&self) -> f32 {
        match self {
            Self::Div1_5 => 1.5,
            Self::Div1 => 1.,
        }
    }
}

/// Settings used to configure clocks
pub struct Clocks {
    pub input_freq: u8, // Mhz, eg HSE speed
    pub input_src: InputSrc,  //
    pub prediv: Prediv, // Input source predivision, for PLL.
    pub pll_mul: PllMul, // PLL multiplier: SYSCLK speed is input source × this value.
    pub usb_pre: UsbPrescaler, // USB prescaler, for target of 48Mhz.
    pub hclk_prescaler: HclkPrescaler, // The AHB clock divider.
    pub apb1_prescaler: ApbPrescaler,  // APB1 divider, for the low speed peripheral bus.
    pub apb2_prescaler: ApbPrescaler, // APB2 divider, for the high speed peripheral bus.
    // Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    // frees up the pin for use as GPIO.
    pub hse_bypass: bool,
}

impl Clocks {
    /// Setup clocks and return a `Valid` status if the config is valid. Return
    /// `Invalid`, and don't setup if not.
    /// https://docs.rs/stm32f3xx-hal/0.5.0/stm32f3xx_hal/rcc/struct.CFGR.html
    /// Use the STM32CubeIDE Clock Configuration tab to help.
    pub fn setup(&self, rcc: &mut RCC, flash: &mut FLASH) -> Result<(), SpeedError> {
        if let Validation::NotValid = self.validate() {
            return Err(SpeedError {});
        }

        // 303 FM, 9.2.3:
        // The internal PLL can be used to multiply the HSI or HSE output clock frequency. Refer to
        // Figure 13 and Clock control register (RCC_CR).
        // The PLL configuration (selection of the input clock, and multiplication factor) must be done
        // before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
        // To modify the PLL configuration, proceed as follows:
        // 1. Disable the PLL by setting PLLON to 0.
        // 2. Wait until PLLRDY is cleared. The PLL is now fully stopped.
        // 3. Change the desired parameter.
        // 4. Enable the PLL again by setting PLLON to 1.
        // An interrupt can be generated when the PLL is ready, if enabled in the Clock interrupt
        // register (RCC_CIR).
        // The PLL output frequency must be set in the range 16-72 MHz.
        // Set up the HSE if required.
        match self.input_src {
            InputSrc::Hse => {
                rcc.cr.modify(|_, w| w.hseon().bit(true));
                // Wait for the HSE to be ready.
                while rcc.cr.read().hserdy() == false {}
            }
            InputSrc::Hsi => (),
            InputSrc::Pll(pll_src) => {
                if let PllSrc::Hse = pll_src {
                    // todo: DRY
                    rcc.cr.modify(|_, w| w.hseon().bit(true));
                    while rcc.cr.read().hserdy() == false {}
                }
            }
        }

        rcc.cr.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            w.hsebyp().bit(self.hse_bypass);
            // Turn off the PLL: Required for modifying some of the settings below.
            w.pllon().bit(false)
        });

        while rcc.cr.read().pllrdy() == false {}

        rcc.cfgr.modify(|_, w| {
            w.usbpre().bit(self.usb_pre.bit()); // eg: Divide by 1.5: 72/1.5 = 48Mhz, required by USB clock.

            if let InputSrc::Pll(pll_src) = self.input_src {
                w.pllmul().bits(self.pll_mul as u8); // eg: 8Mhz HSE x 9 = 72Mhz
                unsafe { w.pllsrc().bits(pll_src as u8) }; // eg: Set HSE as PREDIV1 entry.
            };

            unsafe { w.ppre2().bits(self.apb2_prescaler as u8) }; // HCLK not divided for APB2.
            unsafe { w.ppre1().bits(self.apb1_prescaler as u8) }; // HCLK not divided for APB1
            unsafe { w.hpre().bits(self.hclk_prescaler as u8) } // eg: Divide SYSCLK by 2 to get HCLK of 36Mhz.
        });

        rcc.cfgr2.modify(|_, w| w.prediv().bits(self.prediv as u8));

        // Now turn PLL back on, once we're configured things that can only be set with it off.
        rcc.cr.modify(|_, w| w.pllon().bit(true));

        // Adjust flash wait states according to the HCLK frequency
        // todo: This hclk calculation is DRY from `calc_speeds`.
        let sysclk = match self.input_src {
            InputSrc::Pll(_) => self.input_freq / self.prediv.value() * self.pll_mul.value(),
            _ => self.input_freq,
        };
        let hclk = sysclk as f32 / self.hclk_prescaler.value() as f32;

        // f3 ref man section 4.5.1
        flash.acr.modify(|_, w| {
            if hclk <= 24. {
                w.latency().ws0()
            } else if hclk <= 48. {
                w.latency().ws1()
            } else {
                w.latency().ws2()
            }
        });

        Ok(())
    }

    /// Calculate clock speeds from a given config. Everything is in Mhz.
    /// todo: Handle fractions of mhz. Do floats.
    pub fn calc_speeds(&self) -> Speeds {
        let sysclk = match self.input_src {
            InputSrc::Pll(_) => {
                self.input_freq as f32 / self.prediv.value() as f32 * self.pll_mul.value() as f32
            }
            _ => self.input_freq as f32,
        };

        let usb = sysclk / self.usb_pre.value() as f32;
        let hclk = sysclk / self.hclk_prescaler.value() as f32;
        let systick = hclk; // todo the required divider is not yet implemented.
        let fclk = hclk;
        let pclk1 = hclk / self.apb1_prescaler.value() as f32;
        let timer1 = pclk1;
        let pclk2 = hclk / self.apb1_prescaler.value() as f32;
        let timer2 = pclk2;

        Speeds {
            sysclk,
            usb,
            hclk,
            systick,
            fclk,
            pclk1,
            timer1,
            pclk2,
            timer2,
        }
    }

    /// Check if valid.
    pub fn validate(&self) -> Validation {
        validate(self.calc_speeds()).0
    }

    pub fn validate_usb(&self) -> Validation {
        validate(self.calc_speeds()).1
    }
}

impl Default for Clocks {
    /// This default configures clocks with a HSE, a 48Mhz sysclck, and 24Mhz AHB and
    /// peripheral clocks. USB is set up at 48Mhz. HSE output is not bypassed.
    fn default() -> Self {
        Self {
            input_freq: 8,
            input_src: InputSrc::Pll(PllSrc::Hse),
            prediv: Prediv::Div1,
            pll_mul: PllMul::Mul6,
            usb_pre: UsbPrescaler::Div1,
            hclk_prescaler: HclkPrescaler::Div2,
            apb1_prescaler: ApbPrescaler::Div1,
            apb2_prescaler: ApbPrescaler::Div1,
            hse_bypass: false,
        }
    }
}

/// Validate resulting speeds from a given clock config
/// Main validation, USB validation
pub fn validate(speeds: Speeds) -> (Validation, Validation) {
    let mut main = Validation::Valid;
    let mut usb = Validation::Valid;

    if speeds.sysclk > 72. || speeds.sysclk < 16. {
        main = Validation::NotValid;
    }

    if speeds.hclk > 72. || speeds.sysclk < 0. {
        main = Validation::NotValid;
    }

    if speeds.pclk1 > 36. || speeds.pclk1 < 12. {
        main = Validation::NotValid;
    }

    if speeds.pclk2 > 72. || speeds.pclk1 < 0. {
        main = Validation::NotValid;
    }

    if speeds.usb as u8 != 48 {
        usb = Validation::NotValid;
    }

    (main, usb)
}
