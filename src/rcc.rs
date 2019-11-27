//! Reset and Clock Control

use core::cmp;

use crate::stm32::{rcc, RCC};
use cast::u32;

use crate::flash::ACR;
use crate::time::Hertz;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb: AHB { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB) registers
    pub ahb: AHB,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Clock configuration
    pub cfgr: CFGR,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    pub(crate) fn enr(&mut self) -> &rcc::AHBENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbenr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHBRSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahbrstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

const HSI: u32 = 8_000_000; // Hz

// some microcontrollers do not have USB
#[cfg(any(feature = "stm32f301", feature = "stm32f334",))]
mod usb_clocking {
    pub fn has_usb() -> bool {
        false
    }

    pub fn set_usbpre<W>(w: &mut W, _: bool) -> &mut W {
        w
    }
}

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
mod usb_clocking {
    use crate::stm32::rcc;

    pub fn has_usb() -> bool {
        true
    }

    pub fn set_usbpre(w: &mut rcc::cfgr::W, bit: bool) -> &mut rcc::cfgr::W {
        w.usbpre().bit(bit)
    }
}

use self::usb_clocking::{has_usb, set_usbpre};

/// Clock configuration
pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

impl CFGR {
    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    pub fn use_hse<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hse = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    /// Returns a tuple of the (pllsrclk frequency, pllmul, and pllsrc).
    #[cfg(not(any(
        feature = "stm32f302",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398"
    )))]
    fn calc_pll(&self) -> (u32, u32, rcc::cfgr::PLLSRCW) {
        let pllsrcclk = self.hse.unwrap_or(HSI / 2);
        let pllmul = self.sysclk.unwrap_or(pllsrcclk) / pllsrcclk;

        let pllsrc = if self.hse.is_some() {
            rcc::cfgr::PLLSRCW::HSE_DIV_PREDIV
        } else {
            rcc::cfgr::PLLSRCW::HSI_DIV2
        };
        (pllsrcclk, pllmul, pllsrc)
    }

    /// Returns a tuple of the (pllsrclk frequency, pllmul, and pllsrc).
    #[cfg(any(
        feature = "stm32f302",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398",
    ))]
    fn calc_pll(&self) -> (u32, u32, rcc::cfgr::PLLSRCW) {
        let mut pllsrcclk = self.hse.unwrap_or(HSI / 2);
        let mut pllmul = self.sysclk.unwrap_or(pllsrcclk) / pllsrcclk;

        let pllsrc = if self.hse.is_some() {
            rcc::cfgr::PLLSRCW::HSE_DIV_PREDIV
        } else if pllmul > 16 {
            pllmul /= 2;
            pllsrcclk *= 2;
            rcc::cfgr::PLLSRCW::HSI_DIV_PREDIV
        } else {
            rcc::cfgr::PLLSRCW::HSI_DIV2
        };
        (pllsrcclk, pllmul, pllsrc)
    }

    /// Returns a tuple containing the effective sysclk rate and optional pll settings.
    fn calc_sysclk(&self) -> (u32, Option<(u8, rcc::cfgr::PLLSRCW)>) {
        let (pllsrcclk, pllmul, pllsrc) = self.calc_pll();

        let pllmul = cmp::min(cmp::max(pllmul, 2), 16);
        let sysclk = pllmul * pllsrcclk;
        assert!(sysclk <= 72_000_000);

        let pll_options = if pllmul == 2 {
            None
        } else {
            let pllmul_bits = pllmul as u8 - 2;
            Some((pllmul_bits, pllsrc))
        };

        (sysclk, pll_options)
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let (sysclk, pll_options) = self.calc_sysclk();

        let hpre_bits = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3..=5 => 0b1001,
                6..=11 => 0b1010,
                12..=39 => 0b1011,
                40..=95 => 0b1100,
                96..=191 => 0b1101,
                192..=383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = sysclk / (1 << (hpre_bits - 0b0111));

        assert!(hclk <= 72_000_000);

        let ppre1_bits = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / u32(ppre1);

        assert!(pclk1 <= 36_000_000);

        let ppre2_bits = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / u32(ppre2);

        assert!(pclk2 <= 72_000_000);

        // adjust flash wait states
        unsafe {
            acr.acr().write(|w| {
                w.latency().bits(if sysclk <= 24_000_000 {
                    0b000
                } else if sysclk <= 48_000_000 {
                    0b001
                } else {
                    0b010
                })
            })
        }

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let usb_ok = has_usb() && self.hse.is_some() && pll_options.is_some();
        let (usbpre, usbclk_valid) = match (usb_ok, sysclk) {
            (true, 72_000_000) => (false, true),
            (true, 48_000_000) => (true, true),
            _ => (true, false),
        };

        let rcc = unsafe { &*RCC::ptr() };

        if self.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr.modify(|_, w| w.hseon().set_bit());

            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        if let Some((pllmul_bits, pllsrc)) = pll_options {
            // enable PLL and wait for it to be ready
            rcc.cfgr
                .write(|w| w.pllmul().bits(pllmul_bits).pllsrc().variant(pllsrc));

            rcc.cr.write(|w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

        // set prescalers and clock source
        rcc.cfgr.modify(|_, w| unsafe {
            set_usbpre(w, usbpre)
                .ppre2()
                .bits(ppre2_bits)
                .ppre1()
                .bits(ppre1_bits)
                .hpre()
                .bits(hpre_bits)
                .sw()
                .bits(if pll_options.is_some() {
                    // PLL
                    0b10
                } else if self.hse.is_some() {
                    // HSE
                    0b01
                } else {
                    // HSI
                    0b00
                })
        });


        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(pclk1),
            pclk2: Hertz(pclk2),
            ppre1,
            ppre2,
            sysclk: Hertz(sysclk),
            usbclk_valid,
        }
    }


}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    // TODO remove `allow`
    #[allow(dead_code)]
    ppre2: u8,
    sysclk: Hertz,
    usbclk_valid: bool,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    pub(crate) fn ppre1(&self) -> u8 {
        self.ppre1
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    pub fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}
