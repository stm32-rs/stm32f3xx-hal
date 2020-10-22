//! Reset and Clock Control

use crate::pac::{
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
        }
    }
}

/// Constrained RCC peripheral
///
/// An instance of this struct is aquired by calling the
/// [constrain](trait.RccExt.html#tymethod.constrain) function on the
/// [pac::RCC](../pac/struct.RCC.html) struct.
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// ```
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
}

/// AMBA High-performance Bus (AHB) registers
///
/// An instance of this struct is aquired from the [Rcc](../struct.Rcc.html) struct.
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// use_ahb(&mut rcc.ahb)
/// ```
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
///
/// An instance of this struct is aquired from the [Rcc](../struct.Rcc.html) struct.
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// use_apb1(&mut rcc.apb1)
/// ```
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
///
/// An instance of this struct is aquired from the [Rcc](../struct.Rcc.html) struct.
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// use_apb2(&mut rcc.apb2)
/// ```
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
#[cfg(any(feature = "stm32f301", feature = "stm32f318", feature = "stm32f334",))]
mod usb_clocking {
    use crate::rcc::PllConfig;

    pub(crate) fn is_valid(
        _sysclk: u32,
        _hse: Option<u32>,
        _pclk1: u32,
        _pll_config: &Option<PllConfig>,
    ) -> (bool, bool) {
        (false, false)
    }

    pub(crate) fn set_usbpre<W>(w: &mut W, _: bool) -> &mut W {
        w
    }
}

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
mod usb_clocking {
    use crate::pac::rcc::cfgr;
    use crate::rcc::PllConfig;

    /// Check for all clock options to be
    pub(crate) fn is_valid(
        sysclk: u32,
        hse: Option<u32>,
        pclk1: u32,
        pll_config: &Option<PllConfig>,
    ) -> (cfgr::USBPRE_A, bool) {
        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let usb_ok = hse.is_some() && pll_config.is_some();
        // The APB1 clock must have a minimum frequency of 10 MHz to avoid data overrun/underrun
        // problems. [RM0316 32.5.2]
        if pclk1 >= 10_000_000 {
            match (usb_ok, sysclk) {
                (true, 72_000_000) => (cfgr::USBPRE_A::DIV1_5, true),
                (true, 48_000_000) => (cfgr::USBPRE_A::DIV1, true),
                _ => (cfgr::USBPRE_A::DIV1, false),
            }
        } else {
            (cfgr::USBPRE_A::DIV1, false)
        }
    }

    pub(crate) fn set_usbpre(w: &mut cfgr::W, usb_prescale: cfgr::USBPRE_A) -> &mut cfgr::W {
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
///
/// An instance of this struct is aquired from the [Rcc](../struct.Rcc.html) struct.
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// use_cfgr(&mut rcc.cfgr)
/// ```
pub struct CFGR {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

pub(crate) struct PllConfig {
    src: cfgr::PLLSRC_A,
    mul: cfgr::PLLMUL_A,
    div: Option<cfgr2::PREDIV_A>,
}

/// Determine the [greatest common divisor](https://en.wikipedia.org/wiki/Greatest_common_divisor)
///
/// This function is based on the [Euclidean algorithm](https://en.wikipedia.org/wiki/Euclidean_algorithm).
fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
        let r = a % b;
        a = b;
        b = r;
    }
    a
}

/// Convert pll multiplier into equivalent register field type
fn into_pll_mul(mul: u8) -> cfgr::PLLMUL_A {
    match mul {
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
    }
}

/// Convert pll divisor into equivalent register field type
fn into_pre_div(div: u8) -> cfgr2::PREDIV_A {
    match div {
        1 => cfgr2::PREDIV_A::DIV1,
        2 => cfgr2::PREDIV_A::DIV2,
        3 => cfgr2::PREDIV_A::DIV3,
        4 => cfgr2::PREDIV_A::DIV4,
        5 => cfgr2::PREDIV_A::DIV5,
        6 => cfgr2::PREDIV_A::DIV6,
        7 => cfgr2::PREDIV_A::DIV7,
        8 => cfgr2::PREDIV_A::DIV8,
        9 => cfgr2::PREDIV_A::DIV9,
        10 => cfgr2::PREDIV_A::DIV10,
        11 => cfgr2::PREDIV_A::DIV11,
        12 => cfgr2::PREDIV_A::DIV12,
        13 => cfgr2::PREDIV_A::DIV13,
        14 => cfgr2::PREDIV_A::DIV14,
        15 => cfgr2::PREDIV_A::DIV15,
        16 => cfgr2::PREDIV_A::DIV16,
        _ => unreachable!(),
    }
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

    /// Calculate the values for the pll multiplier (PLLMUL) and the pll divisior (PLLDIV).
    ///
    /// These values are chosen depending on the chosen system clock (SYSCLK) and the frequency of the
    /// oscillator clock (HSE / HSI).
    ///
    /// For these devices, PLL_SRC can selected between the internal oscillator (HSI) and
    /// the external oscillator (HSE).
    ///
    /// HSI is divided by 2 before its transferred to PLL_SRC.
    /// HSE can be divided between 2..16, before it is transferred to PLL_SRC.
    /// After this system clock frequency (SYSCLK) can be changed via multiplier.
    /// The value can be multiplied with 2..16.
    ///
    /// To determine the optimal values, if HSE is chosen as PLL_SRC, the greatest common divisor
    /// is calculated and the limitations of the possible values are taken into consideration.
    ///
    /// HSI is simpler to calculate, but the possible system clocks are less than HSE, because the
    /// division is not configurable.
    #[cfg(not(any(
        feature = "stm32f302xd",
        feature = "stm32f302xe",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398"
    )))]
    fn calc_pll(&self, sysclk: u32) -> (u32, PllConfig) {
        let pllsrcclk = self.hse.unwrap_or(HSI / 2);
        // Get the optimal value for the pll divisor (PLL_DIV) and multiplier (PLL_MUL)
        // Only for HSE PLL_DIV can be changed
        let (pll_mul, pll_div): (u32, Option<u32>) = if self.hse.is_some() {
            // Get the optimal value for the pll divisor (PLL_DIV) and multiplier (PLL_MUL)
            // with the greatest common divisor calculation.
            let common_divisor = gcd(sysclk, pllsrcclk);
            let mut multiplier = sysclk / common_divisor;
            let mut divisor = pllsrcclk / common_divisor;

            // Check if the multiplier can be represented by PLL_MUL
            if multiplier == 1 {
                // PLL_MUL minimal value is 2
                multiplier *= 2;
                divisor *= 2;
            }

            // PLL_MUL maximal value is 16
            assert!(multiplier <= 16);

            // PRE_DIV maximal value is 16
            assert!(divisor <= 16);

            (multiplier, Some(divisor))
        }
        // HSI division is always divided by 2 and has no adjustable division
        else {
            let pll_mul = sysclk / pllsrcclk;
            assert!(pll_mul <= 16);
            (pll_mul, None)
        };

        let sysclk = (pllsrcclk / pll_div.unwrap_or(1)) * pll_mul;
        assert!(sysclk <= 72_000_000);

        let pll_src = if self.hse.is_some() {
            cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else {
            cfgr::PLLSRC_A::HSI_DIV2
        };

        // Convert into register bit field types
        let pll_mul_bits = into_pll_mul(pll_mul as u8);
        let pll_div_bits = pll_div.map(|pll_div| into_pre_div(pll_div as u8));

        (
            sysclk,
            PllConfig {
                src: pll_src,
                mul: pll_mul_bits,
                div: pll_div_bits,
            },
        )
    }

    /// Calculate the values for the pll multiplier (PLLMUL) and the pll divisor (PLLDIV).
    ///
    /// These values are chosen depending on the chosen system clock (SYSCLK) and the frequency of the oscillator
    /// clk (HSI / HSE).
    ///
    /// For these devices, PLL_SRC can be set to choose between the internal oscillator (HSI) and
    /// the external oscillator (HSE).
    /// After this the system clock frequency (SYSCLK) can be changed via a division and a
    /// multiplication block.
    /// It can be divided from with values 1..16  and multiplied from 2..16.
    ///
    /// To determine the optimal values, the greatest common divisor is calculated and the
    /// limitations of the possible values are taken into considiration.
    #[cfg(any(
        feature = "stm32f302xd",
        feature = "stm32f302xe",
        feature = "stm32f303xd",
        feature = "stm32f303xe",
        feature = "stm32f398",
    ))]
    fn calc_pll(&self, sysclk: u32) -> (u32, PllConfig) {
        let pllsrcclk = self.hse.unwrap_or(HSI);

        let (pll_mul, pll_div) = {
            // Get the optimal value for the pll divisor (PLL_DIV) and multiplcator (PLL_MUL)
            // with the greatest common divisor calculation.
            let common_divisor = gcd(sysclk, pllsrcclk);
            let mut multiplier = sysclk / common_divisor;
            let mut divisor = pllsrcclk / common_divisor;

            // Check if the multiplier can be represented by PLL_MUL
            if multiplier == 1 {
                // PLL_MUL minimal value is 2
                multiplier *= 2;
                divisor *= 2;
            }

            // PLL_MUL maximal value is 16
            assert!(multiplier <= 16);

            // PRE_DIV maximal value is 16
            assert!(divisor <= 16);

            (multiplier, divisor)
        };

        let sysclk = (pllsrcclk / pll_div) * pll_mul;
        assert!(sysclk <= 72_000_000);

        // Select hardware clock source of the PLL
        // TODO Check whether HSI_DIV2 could be useful
        let pll_src = if self.hse.is_some() {
            cfgr::PLLSRC_A::HSE_DIV_PREDIV
        } else {
            cfgr::PLLSRC_A::HSI_DIV_PREDIV
        };

        // Convert into register bit field types
        let pll_mul_bits = into_pll_mul(pll_mul as u8);
        let pll_div_bits = into_pre_div(pll_div as u8);

        (
            sysclk,
            PllConfig {
                src: pll_src,
                mul: pll_mul_bits,
                div: Some(pll_div_bits),
            },
        )
    }

    /// Get the system clock, the system clock source and the pll_options, if needed.
    ///
    /// The system clock source is determined by the chosen system clock and the provided hardware
    /// clock.
    /// This function does only chose the PLL if needed, otherwise it will use the oscillator clock as system clock.
    fn get_sysclk(&self) -> (u32, cfgr::SW_A, Option<PllConfig>) {
        // If a sysclk is given, check if the PLL has to be used,
        // else select the system clock source, which is either HSI or HSE.
        match (self.sysclk, self.hse) {
            // No need to use the PLL
            // PLL is needed for USB, but we can make this assumption, to not use PLL here,
            // because the two valid USB clocks, 72 Mhz and 48 Mhz, can't be generated
            // directly from neither the internal rc (8 Mhz)  nor the external
            // Oscillator (max 32 Mhz), without using the PLL.
            (Some(sysclk), Some(hse)) if sysclk == hse => (hse, cfgr::SW_A::HSE, None),
            // No need to use the PLL
            (Some(sysclk), None) if sysclk == HSI => (HSI, cfgr::SW_A::HSI, None),
            (Some(sysclk), _) => {
                let (sysclk, pll_config) = self.calc_pll(sysclk);
                (sysclk, cfgr::SW_A::PLL, Some(pll_config))
            }
            // Use HSE as system clock
            (None, Some(hse)) => (hse, cfgr::SW_A::HSE, None),
            // Use HSI as system clock
            (None, None) => (HSI, cfgr::SW_A::HSI, None),
        }
    }

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, acr: &mut ACR) -> Clocks {
        let (sysclk, sysclk_source, pll_config) = self.get_sysclk();

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

        // Adjust flash wait states according to the
        // HCLK frequency (cpu core clock)
        acr.acr().modify(|_, w| {
            if hclk <= 24_000_000 {
                w.latency().ws0()
            } else if hclk <= 48_000_000 {
                w.latency().ws1()
            } else {
                w.latency().ws2()
            }
        });

        let (usbpre, usbclk_valid) = usb_clocking::is_valid(sysclk, self.hse, pclk1, &pll_config);

        let rcc = unsafe { &*RCC::ptr() };

        if self.hse.is_some() {
            // enable HSE and wait for it to be ready
            rcc.cr.modify(|_, w| w.hseon().on());

            while rcc.cr.read().hserdy().is_not_ready() {}
        }

        // enable PLL and wait for it to be ready
        if let Some(pll_config) = pll_config {
            rcc.cfgr.modify(|_, w| {
                w.pllmul()
                    .variant(pll_config.mul)
                    .pllsrc()
                    .variant(pll_config.src)
            });

            if let Some(pll_div) = pll_config.div {
                rcc.cfgr2.modify(|_, w| w.prediv().variant(pll_div));
            };

            rcc.cr.modify(|_, w| w.pllon().on());

            while rcc.cr.read().pllrdy().is_not_ready() {}
        };

        // set prescalers and clock source
        rcc.cfgr.modify(|_, w| {
            usb_clocking::set_usbpre(w, usbpre);

            w.ppre2()
                .variant(ppre2_bits)
                .ppre1()
                .variant(ppre1_bits)
                .hpre()
                .variant(hpre_bits)
                .sw()
                .variant(sysclk_source)
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
