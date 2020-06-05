//! Reset and Clock Control

use core::cmp;

use crate::stm32::{
    rcc::{self, cfgr, cfgr2},
    RCC,
};

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
            bdcr: BDCR { _0: () },
            cfgr: CFGR {
                hse: None,
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
            cr: CR { _0: () },
            csr: CSR { _0: () },
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
    /// RCC Backup Domain
    pub bdcr: BDCR,
    /// Clock configuration
    pub cfgr: CFGR,
    /// RCC Clock Control register
    pub cr: CR,
    /// RCC Control/Status register
    pub csr: CSR,
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
    use crate::stm32::rcc::cfgr;

    pub fn is_valid(
        _sysclk: &u32,
        _hse: &Option<u32>,
        _pll_options: &Option<(cfgr::PLLMUL_A, cfgr::PLLSRC_A)>,
    ) -> (bool, bool) {
        (false, false)
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
    use crate::stm32::rcc::cfgr;

    pub fn is_valid(
        sysclk: &u32,
        hse: &Option<u32>,
        pll_options: &Option<(cfgr::PLLMUL_A, cfgr::PLLSRC_A)>,
    ) -> (cfgr::USBPRE_A, bool) {
        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let usb_ok = hse.is_some() && pll_options.is_some();
        match (usb_ok, sysclk) {
            (true, 72_000_000) => (cfgr::USBPRE_A::DIV1_5, true),
            (true, 48_000_000) => (cfgr::USBPRE_A::DIV1, true),
            _ => (cfgr::USBPRE_A::DIV1, false),
        }
    }

    pub fn set_usbpre(w: &mut cfgr::W, usb_prescale: cfgr::USBPRE_A) -> &mut cfgr::W {
        w.usbpre().variant(usb_prescale)
    }
}

/// Backup Domain Control register (RCC_BDCR)
pub struct BDCR {
    _0: (),
}

impl BDCR {
    pub(crate) fn bdcr(&mut self) -> &rcc::BDCR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).bdcr }
    }
}

/// Clock configuration
#[derive(Clone, Copy)]
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
    fn calc_pll(&self) -> (u32, u32, cfgr::PLLSRC_A) {
        let pllsrcclk = self.hse.unwrap_or(HSI / 2);
        let pllmul = self.sysclk.unwrap_or(pllsrcclk) / pllsrcclk;

        let pllsrc = if self.hse.is_some() {
            cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else {
            cfgr::PLLSRC_A::HSI_DIV2
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
    fn calc_pll(&self) -> (u32, u32, cfgr::PLLSRC_A) {
        let mut pllsrcclk = self.hse.unwrap_or(HSI / 2);
        let mut pllmul = self.sysclk.unwrap_or(pllsrcclk) / pllsrcclk;

        let pllsrc = if self.hse.is_some() {
            cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else if pllmul > 16 {
            pllmul /= 2;
            pllsrcclk *= 2;
            cfgr::PLLSRC_A::HSI_DIV_PREDIV
        } else {
            cfgr::PLLSRC_A::HSI_DIV2
        };
        (pllsrcclk, pllmul, pllsrc)
    }

    /// Returns a tuple containing the effective sysclk rate and optional pll settings.
    fn calc_sysclk(&self) -> (u32, Option<(cfgr::PLLMUL_A, cfgr::PLLSRC_A)>) {
        let (pllsrcclk, pllmul, pllsrc) = self.calc_pll();
        if pllmul == 1 {
            return (pllsrcclk, None);
        }

        let pllmul = cmp::min(cmp::max(pllmul, 2), 16);
        let sysclk = pllsrcclk * pllmul;
        assert!(sysclk <= 72_000_000);

        // NOTE From<u8> for PLLMUL_A is not implemented
        let pllmul_mul = match pllmul as u8 {
            2 => cfgr::PLLMUL_A::MUL2,
            3 => cfgr::PLLMUL_A::MUL3,
            4 => cfgr::PLLMUL_A::MUL4,
            5 => cfgr::PLLMUL_A::MUL5,
            6 => cfgr::PLLMUL_A::MUL6,
            7 => cfgr::PLLMUL_A::MUL7,
            8 => cfgr::PLLMUL_A::MUL8,
            9 => cfgr::PLLMUL_A::MUL9,
            10 => cfgr::PLLMUL_A::MUL10,
            11 => cfgr::PLLMUL_A::MUL11,
            12 => cfgr::PLLMUL_A::MUL12,
            13 => cfgr::PLLMUL_A::MUL13,
            14 => cfgr::PLLMUL_A::MUL14,
            15 => cfgr::PLLMUL_A::MUL15,
            16 => cfgr::PLLMUL_A::MUL16,
            _ => unreachable!(),
        };
        (sysclk, Some((pllmul_mul, pllsrc)))
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let (sysclk, pll_options) = self.calc_sysclk();

        let (hpre_bits, hpre) = self
            .hclk
            .map(|hclk| match sysclk / hclk {
                0 => unreachable!(),
                1 => (cfgr::HPRE_A::DIV1, 1),
                2 => (cfgr::HPRE_A::DIV2, 2),
                3..=5 => (cfgr::HPRE_A::DIV4, 4),
                6..=11 => (cfgr::HPRE_A::DIV8, 8),
                12..=39 => (cfgr::HPRE_A::DIV16, 16),
                40..=95 => (cfgr::HPRE_A::DIV64, 64),
                96..=191 => (cfgr::HPRE_A::DIV128, 128),
                192..=383 => (cfgr::HPRE_A::DIV256, 256),
                _ => (cfgr::HPRE_A::DIV512, 512),
            })
            .unwrap_or((cfgr::HPRE_A::DIV1, 1));

        let hclk: u32 = sysclk / hpre;

        assert!(hclk <= 72_000_000);

        let (ppre1_bits, ppre1) = self
            .pclk1
            .map(|pclk1| match hclk / pclk1 {
                0 => unreachable!(),
                1 => (cfgr::PPRE1_A::DIV1, 1),
                2 => (cfgr::PPRE1_A::DIV2, 2),
                3..=5 => (cfgr::PPRE1_A::DIV4, 4),
                6..=11 => (cfgr::PPRE1_A::DIV8, 8),
                _ => (cfgr::PPRE1_A::DIV16, 16),
            })
            .unwrap_or((cfgr::PPRE1_A::DIV1, 1));

        let pclk1 = hclk / u32::from(ppre1);

        assert!(pclk1 <= 36_000_000);

        let (ppre2_bits, ppre2) = self
            .pclk2
            .map(|pclk2| match hclk / pclk2 {
                0 => unreachable!(),
                1 => (cfgr::PPRE2_A::DIV1, 1),
                2 => (cfgr::PPRE2_A::DIV2, 2),
                3..=5 => (cfgr::PPRE2_A::DIV4, 4),
                6..=11 => (cfgr::PPRE2_A::DIV8, 8),
                _ => (cfgr::PPRE2_A::DIV16, 16),
            })
            .unwrap_or((cfgr::PPRE2_A::DIV1, 1));

        let pclk2 = hclk / u32::from(ppre2);

        assert!(pclk2 <= 72_000_000);

        // adjust flash wait states
        unsafe {
            acr.acr().modify(|_, w| {
                w.latency().bits(if sysclk <= 24_000_000 {
                    0b000
                } else if sysclk <= 48_000_000 {
                    0b001
                } else {
                    0b010
                })
            })
        }

        let (usbpre, usbclk_valid) = usb_clocking::is_valid(&sysclk, &self.hse, &pll_options);

        let rcc = unsafe { &*RCC::ptr() };

        if self.hse.is_some() {
            // enable HSE and wait for it to be ready
            rcc.cr.modify(|_, w| w.hseon().on());

            while rcc.cr.read().hserdy().is_not_ready() {}
        }

        if let Some((pllmul_mul, pllsrc)) = pll_options {
            // enable PLL and wait for it to be ready
            rcc.cfgr
                .modify(|_, w| w.pllmul().variant(pllmul_mul).pllsrc().variant(pllsrc));

            rcc.cr.modify(|_, w| w.pllon().on());

            while rcc.cr.read().pllrdy().is_not_ready() {}
        }

        // set prescalers and clock source
        rcc.cfgr.modify(|_, w| {
            usb_clocking::set_usbpre(w, usbpre);

            w.ppre2()
                .variant(ppre2_bits)
                .ppre1()
                .variant(ppre1_bits)
                .hpre()
                .variant(hpre_bits);

            if pll_options.is_some() {
                w.sw().pll()
            } else if self.hse.is_some() {
                w.sw().hse()
            } else {
                w.sw().hsi()
            }
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

/// RCC Clock Control register (RCC_CR)
pub struct CR {
    _0: (),
}

impl CR {
    pub(crate) fn cr(&mut self) -> &rcc::CR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).cr }
    }
}

/// RCC Control/Status register
pub struct CSR {
    _0: (),
}

impl CSR {
    pub(crate) fn csr(&mut self) -> &rcc::CSR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).csr }
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
