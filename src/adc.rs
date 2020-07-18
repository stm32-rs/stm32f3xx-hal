//! # Work in Progress
//! API for the ADC (Analog to Digital Converter)
//!
//! Note that the more specific your hardware selection is
//! (e.g. stm32f303**xc** instead of just stm32f303)
//! the more functionality is accessible
//! (in this case: ADC3 and ADC4 additionaly to ADC1 and ADC2).
//!
//! # Examples
//! For a simple, working example check `adc.rs` in the examples folder.
use cortex_m::asm;
use embedded_hal::adc::{Channel, OneShot};
#[cfg(feature = "stm32f303")]
use stm32f3::stm32f303::adc1_2::ccr::CKMODE_A;

use crate::rcc::{Clocks, AHB};

#[cfg(feature = "stm32f303")]
const MAX_ADVREGEN_STARTUP_US: u32 = 10;

#[cfg(feature = "stm32f303")]
use crate::gpio::{gpioa, gpiob, gpioc, Analog};

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
use crate::gpio::{gpiod, gpioe, gpiof};

#[cfg(feature = "stm32f303")]
use crate::pac::{ADC1, ADC1_2, ADC2};

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
use crate::pac::{ADC3, ADC3_4, ADC4};

/// ADC configuration
///
/// TODO: Remove `pub` from the register block once all functionalities are implemented.
/// Leave it here until then as it allows easy access to the registers.
pub struct Adc<ADC> {
    pub rb: ADC,
    clocks: Clocks,
    ckmode: CKMODE,
    operation_mode: Option<OperationMode>,
}

/// ADC sampling time
///
/// Each channel can be sampled with a different sample time.
/// For Sampletime T_n the total conversion time (in ADC clock cycles) is
/// 12.5 + (n + .5)
///
/// TODO: there are boundaries on how this can be set depending on the hardware.
/// Check them and implement a sample time setting mechanism.
pub enum SampleTime {
    T_1,
    T_2,
    T_4,
    T_7,
    T_19,
    T_61,
    T_181,
    T_601,
}

impl Default for SampleTime {
    fn default() -> Self {
        SampleTime::T_19
    }
}

impl SampleTime {
    /// Conversion to bits for SMP
    fn bitcode(&self) -> u8 {
        match self {
            SampleTime::T_1 => 0b000,
            SampleTime::T_2 => 0b001,
            SampleTime::T_4 => 0b010,
            SampleTime::T_7 => 0b011,
            SampleTime::T_19 => 0b100,
            SampleTime::T_61 => 0b101,
            SampleTime::T_181 => 0b110,
            SampleTime::T_601 => 0b111,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
/// ADC operation mode
///
/// TODO: Implement other modes (DMA, Differential,…)
pub enum OperationMode {
    OneShot,
}

#[derive(Clone, Copy, PartialEq)]
/// ADC CKMODE
///
/// TODO: Add ASYNCHRONOUS mode
pub enum CKMODE {
    // ASYNCHRONOUS = 0,
    SYNCDIV1 = 1,
    SYNCDIV2 = 2,
    SYNCDIV4 = 4,
}

impl Default for CKMODE {
    fn default() -> Self {
        CKMODE::SYNCDIV2
    }
}

// ADC3_2 returns a pointer to a adc1_2 type, so this from is ok for both.
#[cfg(feature = "stm32f303")]
impl From<CKMODE> for stm32f3::stm32f303::adc1_2::ccr::CKMODE_A {
    fn from(ckmode: CKMODE) -> Self {
        match ckmode {
            //CKMODE::ASYNCHRONOUS => CKMODE_A::ASYNCHRONOUS,
            CKMODE::SYNCDIV1 => CKMODE_A::SYNCDIV1,
            CKMODE::SYNCDIV2 => CKMODE_A::SYNCDIV2,
            CKMODE::SYNCDIV4 => CKMODE_A::SYNCDIV4,
        }
    }
}

/// ADC data register alignment
pub enum Align {
    /// Right alignment of output data
    Right,
    /// Left alignment of output data
    Left,
}

impl Default for Align {
    fn default() -> Self {
        Align::Right
    }
}

impl Align {
    /// Conversion to bits for ALIGN in ADCx_CFGR
    fn bitvalue(&self) -> bool {
        match self {
            Align::Right => false,
            Align::Left => true,
        }
    }
}

/// Maps pins to ADC Channels.
macro_rules! adc_pins {
    ($ADC:ident, $($pin:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

// # ADC1 Pin/Channel mapping
// ## f303

#[cfg(feature = "stm32f303")]
adc_pins!(ADC1,
    gpioa::PA0<Analog> => 1_u8,
    gpioa::PA1<Analog> => 2_u8,
    gpioa::PA2<Analog> => 3_u8,
    gpioa::PA3<Analog> => 4_u8,
    gpioc::PC0<Analog> => 6_u8,
    gpioc::PC1<Analog> => 7_u8,
    gpioc::PC2<Analog> => 8_u8,
    gpioc::PC3<Analog> => 9_u8,
);

#[cfg(any(feature = "stm32f303x6", feature = "stm32f303x8"))]
adc_pins!(ADC1,
    gpiob::PB0<Analog> => 11_u8,
    gpiob::PB1<Analog> => 12_u8,
    gpiob::PB13<Analog> => 13_u8,
);

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
adc_pins!(ADC1,
    gpiof::PF4<Analog> => 5_u8,
    gpiof::PF2<Analog> => 10_u8,
);

// # ADC2 Pin/Channel mapping
// ## f303

#[cfg(feature = "stm32f303")]
adc_pins!(ADC2,
    gpioa::PA4<Analog> => 1_u8,
    gpioa::PA5<Analog> => 2_u8,
    gpioa::PA6<Analog> => 3_u8,
    gpioa::PA7<Analog> => 4_u8,
    gpioc::PC4<Analog> => 5_u8,
    gpioc::PC5<Analog> => 11_u8,
    gpiob::PB2<Analog> => 12_u8,
    gpioc::PC0<Analog> => 6_u8,
    gpioc::PC1<Analog> => 7_u8,
    gpioc::PC2<Analog> => 8_u8,
    gpioc::PC3<Analog> => 9_u8,
);

#[cfg(any(feature = "stm32f303x6", feature = "stm32f303x8"))]
adc_pins!(ADC2,
    gpiob::PB12<Analog> => 13_u8,
    gpiob::PB14<Analog> => 14_u8,
    gpiob::PB15<Analog> => 15_u8,
);

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
adc_pins!(ADC2,
    gpiof::PF2<Analog> => 10_u8,
);

// # ADC3 Pin/Channel mapping
// ## f303

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
adc_pins!(ADC3,
    gpiob::PB1<Analog> => 1_u8,
    gpioe::PE9<Analog> => 2_u8,
    gpioe::PE13<Analog> => 3_u8,
    gpiob::PB13<Analog> => 5_u8,
    gpiob::PB0<Analog> => 12_u8,
    gpioe::PE7<Analog> => 13_u8,
    gpioe::PE10<Analog> => 14_u8,
    gpioe::PE11<Analog> => 15_u8,
    gpioe::PE12<Analog> => 16_u8,
    // Shared channels (i.e. ADC34_INx)
    gpioe::PE8<Analog> => 6_u8,
    gpiod::PD10<Analog> => 7_u8,
    gpiod::PD11<Analog> => 8_u8,
    gpiod::PD12<Analog> => 9_u8,
    gpiod::PD13<Analog> => 10_u8,
    gpiod::PD14<Analog> => 11_u8,
);

// # ADC4 Pin/Channel mapping
// ## f303

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
adc_pins!(ADC4,
    gpioe::PE14<Analog> => 1_u8,
    gpioe::PE15<Analog> => 2_u8,
    gpiob::PB12<Analog> => 3_u8,
    gpiob::PB14<Analog> => 4_u8,
    gpiob::PB15<Analog> => 5_u8,
    gpiob::PB8<Analog> => 12_u8,
    gpiob::PB9<Analog> => 13_u8,
    // Shared channels (i.e. ADC34_INx)
    gpioe::PE8<Analog> => 6_u8,
    gpiod::PD10<Analog> => 7_u8,
    gpiod::PD11<Analog> => 8_u8,
    gpiod::PD12<Analog> => 9_u8,
    gpiod::PD13<Analog> => 10_u8,
    gpiod::PD14<Analog> => 11_u8,
);

/// Abstract implementation of ADC functionality
///
/// Do not use directly. See adc12_hal for a applicable Macro.
/// TODO: Extend/generalize beyond f303
macro_rules! adc_hal {
    ($(
            $ADC:ident: ($adcx:ident, $ADC_COMMON:ident),
    )+) => {
        $(
            impl Adc<$ADC> {

                /// Init a new ADC
                ///
                /// Enables the clock, performs a calibration and enables the ADC
                pub fn $adcx(
                    rb: $ADC,
                    adc_common : &mut $ADC_COMMON,
                    ahb: &mut AHB,
                    ckmode: CKMODE,
                    clocks: Clocks,
                ) -> Option<Self> {
                    let mut this_adc = Self {
                        rb,
                        clocks,
                        ckmode,
                        operation_mode: None,
                    };
                    if !(this_adc.clocks_welldefined(clocks)) {
                        return None;
                    }
                    if !(this_adc.enable_clock(ahb, adc_common)){
                        return None;
                    }
                    this_adc.set_align(Align::default());
                    this_adc.calibrate();
                    // ADEN bit cannot be set during ADCAL=1
                    // and 4 ADC clock cycle after the ADCAL
                    // bit is cleared by hardware
                    this_adc.wait_adc_clk_cycles(4);
                    this_adc.enable();
                    Some(this_adc)
                }

                /// Software can use CKMODE::SYNCDIV1 only if
                /// hclk and sysclk are the same. (see reference manual 15.3.3)
                fn clocks_welldefined(&self, clocks: Clocks) -> bool {
                    !(self.ckmode == CKMODE::SYNCDIV1 && !(clocks.hclk().0 == clocks.sysclk().0))
                }

                /// sets up adc in one shot mode for a single channel
                pub fn setup_oneshot(&mut self) {
                    // stop and clear overrun events
                    self.rb.cr.modify(|_, w| w.adstp().set_bit());
                    self.rb.isr.modify(|_, w| w.ovr().clear_bit());

                    self.rb.cfgr.modify(|_, w| w
                        .cont().clear_bit()
                        .ovrmod().clear_bit()
                    );

                    self.rb.sqr1.modify(|_, w| w.l().bits(0));

                    self.operation_mode = Some(OperationMode::OneShot);
                }

                fn set_align(&self, align: Align) {
                    self.rb.cfgr.modify(|_, w| w.align().bit(align.bitvalue()));
                }

                fn enable(&mut self) {
                    self.rb.cr.modify(|_, w| w.aden().set_bit());
                    while self.rb.isr.read().adrdy().bit_is_clear() {}
                }

                fn disable(&mut self) {
                    self.rb.cr.modify(|_, w| w.aden().clear_bit());
                }


                /// Calibrate according to 15.3.8 in the Reference Manual
                fn calibrate(&mut self) {
                    if !self.rb.cr.read().advregen().is_enabled() {
                        self.advregen_enable();
                        self.wait_advregen_startup();
                    }

                    self.disable();

                    self.rb.cr.modify(|_, w| w
                        // NOTE: needs to be adopted if implementing differential input
                        .adcaldif().clear_bit()
                        .adcal()   .set_bit());

                    while self.rb.cr.read().adcal().bit_is_set() {}
                }


                fn wait_adc_clk_cycles(&self, cycles: u32) {
                    let adc_clk_cycle = self.clocks.hclk().0 / (self.ckmode as u32);
                    asm::delay(adc_clk_cycle * cycles);
                }

                fn advregen_enable(&mut self){
                    // need to go through intermediate first
                    self.rb.cr.modify(|_, w| w.advregen().intermediate());
                    self.rb.cr.modify(|_, w| w.advregen().enabled());

                }

                /// wait for the advregen to startup.
                ///
                /// This is based on the MAX_ADVREGEN_STARTUP_US of the device.
                fn wait_advregen_startup(&self) {
                        asm::delay((MAX_ADVREGEN_STARTUP_US * 1_000_000) / self.clocks.sysclk().0);
                }

                /// busy ADC read
                fn convert_one(&mut self, chan: u8) -> u16 {
                    self.ensure_oneshot();
                    self.set_chan_smps(chan, SampleTime::default());
                    self.select_single_chan(chan);

                    self.rb.cr.modify(|_, w| w.adstart().set_bit());
                    while self.rb.isr.read().eos().is_not_complete() {}
                    self.rb.isr.modify(|_, w| w.eos().clear());
                    return self.rb.dr.read().rdata().bits();
                }

                fn ensure_oneshot(&mut self) {
                    if self.operation_mode != Some(OperationMode::OneShot) {
                        self.setup_oneshot();
                    }
                }

                /// This should only be invoked with the defined channels for the particular
                /// device. (See Pin/Channel mapping above)
                fn select_single_chan(&self, chan: u8) {
                    self.rb.sqr1.modify(|_, w|
                        // NOTE(unsafe): chan is the x in ADCn_INx
                        unsafe { w.sq1().bits(chan) }
                    );
                }

                /// Note: only allowed when ADSTART = 0
                fn set_chan_smps(&self, chan: u8, smp: SampleTime) {
                    match chan {
                        1 => self.rb.smpr1.modify(|_, w| w.smp1().bits(smp.bitcode())),
                        2 => self.rb.smpr1.modify(|_, w| w.smp2().bits(smp.bitcode())),
                        3 => self.rb.smpr1.modify(|_, w| w.smp3().bits(smp.bitcode())),
                        4 => self.rb.smpr1.modify(|_, w| w.smp4().bits(smp.bitcode())),
                        5 => self.rb.smpr1.modify(|_, w| w.smp5().bits(smp.bitcode())),
                        6 => self.rb.smpr1.modify(|_, w| w.smp6().bits(smp.bitcode())),
                        7 => self.rb.smpr1.modify(|_, w| w.smp7().bits(smp.bitcode())),
                        8 => self.rb.smpr1.modify(|_, w| w.smp8().bits(smp.bitcode())),
                        9 => self.rb.smpr1.modify(|_, w| w.smp9().bits(smp.bitcode())),
                        11 => self.rb.smpr2.modify(|_, w| w.smp10().bits(smp.bitcode())),
                        12 => self.rb.smpr2.modify(|_, w| w.smp12().bits(smp.bitcode())),
                        13 => self.rb.smpr2.modify(|_, w| w.smp13().bits(smp.bitcode())),
                        14 => self.rb.smpr2.modify(|_, w| w.smp14().bits(smp.bitcode())),
                        15 => self.rb.smpr2.modify(|_, w| w.smp15().bits(smp.bitcode())),
                        16 => self.rb.smpr2.modify(|_, w| w.smp16().bits(smp.bitcode())),
                        17 => self.rb.smpr2.modify(|_, w| w.smp17().bits(smp.bitcode())),
                        18 => self.rb.smpr2.modify(|_, w| w.smp18().bits(smp.bitcode())),
                        _ => unreachable!(),
                    };
                }

            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC>
            where
                WORD: From<u16>,
                PIN: Channel<$ADC, ID = u8>,
                {
                    type Error = ();

                    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                        let res = self.convert_one(PIN::channel());
                        return Ok(res.into());
                    }
                }
        )+
    }
}

/// Macro to implement ADC functionallity for ADC1 and ADC2
///
/// TODO: Extend/differentiate beyond f303.
macro_rules! adc12_hal {
    ($(
            $ADC:ident: ($adcx:ident),
    )+) => {
        $(
            impl Adc<$ADC> {
                /// Returns true iff
                ///     the clock can be enabled with the given settings
                ///  or the clock was already enabled with the same settings
                fn enable_clock(&self, ahb: &mut AHB, adc_common: &mut ADC1_2) -> bool {
                    if ahb.enr().read().adc12en().is_enabled() {
                        return (adc_common.ccr.read().ckmode().variant() == self.ckmode.into());
                    }
                    ahb.enr().modify(|_, w| w.adc12en().enabled());
                    adc_common.ccr.modify(|_, w| w
                        .ckmode().variant(self.ckmode.into())
                    );
                    true
                }
            }
            adc_hal! {
                $ADC: ($adcx, ADC1_2),
            }
        )+
    }
}

/// Macro to implement ADC functionallity for ADC3 and ADC4
///
/// TODO: Extend/differentiate beyond f303.
#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
macro_rules! adc34_hal {
    ($(
            $ADC:ident: ($adcx:ident),
    )+) => {
        $(
            impl Adc<$ADC> {
                /// Returns true iff
                ///     the clock can be enabled with the given settings
                ///  or the clock was already enabled with the same settings
                fn enable_clock(&self, ahb: &mut AHB, adc_common: &mut ADC3_4) -> bool {
                    if ahb.enr().read().adc34en().is_enabled() {
                        return (adc_common.ccr.read().ckmode().variant() == self.ckmode.into());
                    }
                    ahb.enr().modify(|_, w| w.adc34en().enabled());
                    adc_common.ccr.modify(|_, w| w
                        .ckmode().variant(self.ckmode.into())
                    );
                    true
                }
            }
            adc_hal! {
                $ADC: ($adcx, ADC3_4),
            }
        )+
    }
}

#[cfg(feature = "stm32f303")]
adc12_hal! {
    ADC1: (adc1),
    ADC2: (adc2),
}
#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
))]
adc34_hal! {
    ADC3: (adc3),
    ADC4: (adc4),
}
