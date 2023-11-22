//! # Serial Peripheral Interface (SPI) bus
//!
//! ## Examples
//!
//! A usage example of the can peripheral can be found at [examples/spi.rs]
//!
//! [examples/spi.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.1/examples/spi.rs

use core::{fmt, marker::PhantomData, ops::Deref};

use crate::hal::blocking::spi;
use crate::hal::spi::FullDuplex;
pub use crate::hal::spi::{Mode, Phase, Polarity};
#[cfg(feature = "gpio-f303e")]
use crate::pac::SPI4;
use crate::pac::{
    self, spi1,
    spi1::cr2::{DS_A, FRXTH_A},
    Interrupt, SPI1, SPI2, SPI3,
};

use crate::{
    gpio::{self, PushPull, AF5, AF6},
    rcc::{self, Clocks},
    time::rate,
};
use num_traits::{AsPrimitive, PrimInt};

/// SPI error
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

/// SCK pin
pub trait SckPin<SPI>: crate::private::Sealed {}

/// MISO pin
pub trait MisoPin<SPI>: crate::private::Sealed {}

/// MOSI pin
pub trait MosiPin<SPI>: crate::private::Sealed {}

// TODO(Sh3Rm4n): NSS Pins
// pub trait NssPin<SPI>: crate::private::Sealed {}

impl SckPin<SPI1> for gpio::PA5<AF5<PushPull>> {}
impl MisoPin<SPI1> for gpio::PA6<AF5<PushPull>> {}
impl MosiPin<SPI1> for gpio::PA7<AF5<PushPull>> {}
impl MosiPin<SPI1> for gpio::PB5<AF5<PushPull>> {}

#[cfg(not(feature = "gpio-f373"))]
impl SckPin<SPI2> for gpio::PB13<AF5<PushPull>> {}
#[cfg(feature = "gpio-f373")]
impl SckPin<SPI2> for gpio::PB10<AF5<PushPull>> {}
impl MisoPin<SPI2> for gpio::PB14<AF5<PushPull>> {}
impl MosiPin<SPI2> for gpio::PB15<AF5<PushPull>> {}

impl MosiPin<SPI3> for gpio::PB5<AF6<PushPull>> {}

impl SckPin<SPI3> for gpio::PC10<AF6<PushPull>> {}
impl MisoPin<SPI3> for gpio::PC11<AF6<PushPull>> {}
impl MosiPin<SPI3> for gpio::PC12<AF6<PushPull>> {}

cfg_if::cfg_if! {
    if #[cfg(feature = "gpio-f373")] {
        impl SckPin<SPI2> for gpio::PB8<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PD7<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PD8<AF5<PushPull>> {}

        impl SckPin<SPI1> for gpio::PA12<AF6<PushPull>> {}
        impl MisoPin<SPI1> for gpio::PA13<AF6<PushPull>> {}

        impl MosiPin<SPI1> for gpio::PB0<AF5<PushPull>> {}
        impl MosiPin<SPI1> for gpio::PF6<AF5<PushPull>> {}

        impl SckPin<SPI1> for gpio::PC7<AF5<PushPull>> {}
        impl MisoPin<SPI1> for gpio::PC8<AF5<PushPull>> {}
        impl MosiPin<SPI1> for gpio::PC9<AF5<PushPull>> {}


        impl SckPin<SPI2> for gpio::PA8<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PA9<AF5<PushPull>> {}
        impl MosiPin<SPI2> for gpio::PA10<AF5<PushPull>> {}

        impl MisoPin<SPI2> for gpio::PC2<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PC3<AF5<PushPull>> {}

        impl MisoPin<SPI2> for gpio::PD3<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PD4<AF5<PushPull>> {}

        impl SckPin<SPI3> for gpio::PA1<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PA2<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PA3<AF6<PushPull>> {}

        impl SckPin<SPI3> for gpio::PB3<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PB4<AF6<PushPull>> {}
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e",))] {
        impl SckPin<SPI2> for gpio::PF9<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PF10<AF5<PushPull>> {}
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "gpio-f303e")] {
        impl SckPin<SPI4> for gpio::PE2<AF5<PushPull>> {}
        impl MisoPin<SPI4> for gpio::PE5<AF5<PushPull>> {}
        impl MosiPin<SPI4> for gpio::PE6<AF5<PushPull>> {}

        impl SckPin<SPI4> for gpio::PE12<AF5<PushPull>> {}
        impl MisoPin<SPI4> for gpio::PE13<AF5<PushPull>> {}
        impl MosiPin<SPI4> for gpio::PE14<AF5<PushPull>> {}
    }
}

cfg_if::cfg_if! {
    if #[cfg(not(feature = "gpio-f302"))] {
        impl SckPin<SPI1> for gpio::PB3<AF5<PushPull>> {}
        impl MisoPin<SPI1> for gpio::PB4<AF5<PushPull>> {}
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(
        not(feature = "stm32f301"),
        any(feature = "gpio-f303e", feature = "gpio-f302"),
    ))] {
        impl SckPin<SPI2> for gpio::PF1<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PA10<AF5<PushPull>> {}
        impl MosiPin<SPI2> for gpio::PA11<AF5<PushPull>> {}
    }
}

cfg_if::cfg_if! {
    if #[cfg(all(
        not(feature = "stm32f301"),
        any(feature = "gpio-f302", feature = "gpio-f303", feature = "gpio-f303e"),
    ))] {
        impl SckPin<SPI3> for gpio::PB3<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PB4<AF6<PushPull>> {}
    }
}

/// Configuration trait for the Word Size
/// used by the SPI peripheral
pub trait Word {
    /// Returns the register configuration
    /// to set the word size
    fn register_config() -> (FRXTH_A, DS_A);
}

impl Word for u8 {
    fn register_config() -> (FRXTH_A, DS_A) {
        (FRXTH_A::Quarter, DS_A::EightBit)
    }
}

impl Word for u16 {
    fn register_config() -> (FRXTH_A, DS_A) {
        (FRXTH_A::Half, DS_A::SixteenBit)
    }
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, Pins, Word = u8> {
    spi: SPI,
    pins: Pins,
    _word: PhantomData<Word>,
}

pub mod config;

impl<SPI, Sck, Miso, Mosi, WORD> Spi<SPI, (Sck, Miso, Mosi), WORD> {
    /// Configures the SPI peripheral to operate in full duplex master mode.
    ///
    /// The most convinient way to get a device is like that:
    ///
    /// ```ignore
    /// use stm32f3xx_hal::prelude::*;
    /// use stm32f3xx_hal::spi::Spi;
    ///
    /// // ...
    ///
    /// let spi = Spi::new(dp.SPI1, (sck_pin, mosi_pin, miso_pin), 1.MHz, clocks, &mut dp.abp1);
    ///
    /// ```
    ///
    /// To get a better example, look [here](https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.1/examples/spi.rs).
    ///
    // TODO(Sh3Rm4n): See alternative modes provided besides FullDuplex (as listed in Stm32CubeMx).
    pub fn new<Config>(
        spi: SPI,
        pins: (Sck, Miso, Mosi),
        config: Config,
        clocks: Clocks,
        apb: &mut <SPI as rcc::RccBus>::Bus,
    ) -> Self
    where
        SPI: Instance,
        Sck: SckPin<SPI>,
        Miso: MisoPin<SPI>,
        Mosi: MosiPin<SPI>,
        WORD: Word,
        Config: Into<config::Config>,
    {
        let config = config.into();
        SPI::enable(apb);
        SPI::reset(apb);

        let (frxth, ds) = WORD::register_config();
        spi.cr2.write(|w| {
            w.frxth().variant(frxth);
            w.ds().variant(ds);
            // Slave Select output disabled
            w.ssoe().disabled()
        });

        // CPHA: phase
        // CPOL: polarity
        // MSTR: master mode
        // BR: 1 MHz
        // SPE: SPI disabled
        // LSBFIRST: MSB first
        // SSM: enable software slave management (NSS pin free for other uses)
        // SSI: set nss high = master mode
        // CRCEN: hardware CRC calculation disabled
        // BIDIMODE: 2 line unidirectional (full duplex)
        spi.cr1.write(|w| {
            w.mstr().master();

            match config.mode.phase {
                Phase::CaptureOnFirstTransition => w.cpha().first_edge(),
                Phase::CaptureOnSecondTransition => w.cpha().second_edge(),
            };

            match config.mode.polarity {
                Polarity::IdleLow => w.cpol().idle_low(),
                Polarity::IdleHigh => w.cpol().idle_high(),
            };

            w.br()
                .variant(Self::compute_baud_rate(clocks, config.frequency));

            w.spe()
                .enabled()
                .lsbfirst()
                .msbfirst()
                .ssi()
                .slave_not_selected()
                .ssm()
                .enabled()
                .crcen()
                .disabled()
                .bidimode()
                .unidirectional()
        });

        Spi {
            spi,
            pins,
            _word: PhantomData,
        }
    }

    /// Get access to the underlying register block.
    ///
    /// # Safety
    ///
    /// This function is not _memory_ unsafe per se, but does not guarantee
    /// anything about assumptions of invariants made in this implementation.
    ///
    /// Changing specific options can lead to un-expected behavior and nothing
    /// is guaranteed.
    pub unsafe fn peripheral(&mut self) -> &mut SPI {
        &mut self.spi
    }

    /// Releases the SPI peripheral and associated pins
    pub fn free(self) -> (SPI, (Sck, Miso, Mosi)) {
        (self.spi, self.pins)
    }
}

impl<SPI, Pins, Word> Spi<SPI, Pins, Word>
where
    SPI: Instance,
{
    /// Change the baud rate of the SPI
    pub fn reclock(&mut self, freq: impl Into<rate::Generic<u32>>, clocks: Clocks) {
        self.spi.cr1.modify(|_, w| w.spe().disabled());

        self.spi.cr1.modify(|_, w| {
            w.br().variant(Self::compute_baud_rate(clocks, freq.into()));
            w.spe().enabled()
        });
    }

    fn compute_baud_rate(clocks: Clocks, freq: rate::Generic<u32>) -> spi1::cr1::BR_A {
        use spi1::cr1::BR_A;
        match SPI::clock(&clocks).0 / (freq.integer() * *freq.scaling_factor()) {
            0 => crate::unreachable!(),
            1..=2 => BR_A::Div2,
            3..=5 => BR_A::Div4,
            6..=11 => BR_A::Div8,
            12..=23 => BR_A::Div16,
            24..=39 => BR_A::Div32,
            40..=95 => BR_A::Div64,
            96..=191 => BR_A::Div128,
            _ => BR_A::Div256,
        }
    }

    /// Obtain the associated interrupt number for the serial peripheral.
    ///
    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
    /// This is useful for all `cortex_m::peripheral::INTERRUPT` functions.
    #[doc(alias = "unmask")]
    pub fn interrupt(&self) -> <SPI as crate::interrupts::InterruptNumber>::Interrupt {
        <SPI as crate::interrupts::InterruptNumber>::INTERRUPT
    }
}

impl<SPI, Sck, Miso, Mosi, Word> FullDuplex<Word> for Spi<SPI, (Sck, Miso, Mosi), Word>
where
    SPI: Instance,
    // Full Duplex needs the Miso and Mosi pins.
    // SckPin could technically be omitted, though not advisable.
    Miso: MisoPin<SPI>,
    Mosi: MosiPin<SPI>,
    Word: PrimInt + Into<u32> + 'static,
    u32: AsPrimitive<Word>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<Word, Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().is_overrun() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().is_fault() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().is_no_match() {
            nb::Error::Other(Error::Crc)
        } else if sr.rxne().is_not_empty() {
            let read_ptr = core::ptr::addr_of!(self.spi.dr) as *const Word;
            let value = unsafe { core::ptr::read_volatile(read_ptr) };
            return Ok(value);
        } else {
            nb::Error::WouldBlock
        })
    }

    fn send(&mut self, word: Word) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().is_overrun() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().is_fault() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crcerr().is_no_match() {
            nb::Error::Other(Error::Crc)
        } else if sr.txe().is_empty() {
            let write_ptr = core::ptr::addr_of!(self.spi.dr) as *mut Word;
            // NOTE(unsafe) write to register owned by this Spi struct
            unsafe { core::ptr::write_volatile(write_ptr, word) };
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<SPI, Sck, Miso, Mosi, Word> spi::transfer::Default<Word> for Spi<SPI, (Sck, Miso, Mosi), Word>
where
    SPI: Instance,
    Miso: MisoPin<SPI>,
    Mosi: MosiPin<SPI>,
    Word: PrimInt + Into<u32> + 'static,
    u32: AsPrimitive<Word>,
{
}

impl<SPI, Sck, Miso, Mosi, Word> spi::write::Default<Word> for Spi<SPI, (Sck, Miso, Mosi), Word>
where
    SPI: Instance,
    Miso: MisoPin<SPI>,
    Mosi: MosiPin<SPI>,
    Word: PrimInt + Into<u32> + 'static,
    u32: AsPrimitive<Word>,
{
}

/// SPI instance
pub trait Instance:
    Deref<Target = spi1::RegisterBlock>
    + crate::interrupts::InterruptNumber
    + crate::private::Sealed
    + rcc::Enable
    + rcc::Reset
    + rcc::BusClock
{
}

macro_rules! spi {
    ($($SPIX:ident: ($APBX:ident, $pclkX:ident),)+) => {
        $(
            impl crate::interrupts::InterruptNumber for pac::$SPIX {
                type Interrupt = Interrupt;
                const INTERRUPT: Self::Interrupt = interrupts::$SPIX;
            }

            impl Instance for pac::$SPIX { }

            #[cfg(feature = "defmt")]
            impl<Pins> defmt::Format for Spi<pac::$SPIX, Pins> {
                fn format(&self, f: defmt::Formatter) {
                    // Omitting pins makes it:
                    // 1. Easier.
                    // 2. Not to specialized to use it ergonimically for users
                    //    even in a generic context.
                    // 3. Not require specialization.
                    defmt::write!(
                        f,
                        "SPI {{ spi: {}, pins: ? }}",
                        stringify!($SPIX),
                    );
                }
            }

            impl<Pins> fmt::Debug for Spi<$SPIX, Pins> {
                fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                    f.debug_struct(stringify!(Serial))
                        .field("spi", &stringify!($USARTX))
                        .field("pins", &"?")
                        .finish()
                }
            }
        )+
    };

    ([ $(($X:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            spi!(
                $(
                    [<SPI $X>]: (
                        [<APB $APB>],
                        [<pclk $APB>]
                    ),
                )+
            );
        }
    };
}

mod interrupts {
    use crate::pac::Interrupt;

    cfg_if::cfg_if! {
        if #[cfg(feature = "svd-f301")] {
            #[allow(unused)]
            pub(crate) const SPI1: Interrupt = Interrupt::SPI1_IRQ;
            #[allow(unused)]
            pub(crate) const SPI2: Interrupt = Interrupt::SPI2_IRQ;
            #[allow(unused)]
            pub(crate) const SPI3: Interrupt = Interrupt::SPI3_IRQ;
        } else if #[cfg(feature = "svd-f3x4")] {
            pub(crate) const SPI1: Interrupt = Interrupt::SPI1;
        } else {
            #[allow(unused)]
            pub(crate) const SPI1: Interrupt = Interrupt::SPI1;
            #[allow(unused)]
            pub(crate) const SPI2: Interrupt = Interrupt::SPI2;
            #[allow(unused)]
            pub(crate) const SPI3: Interrupt = Interrupt::SPI3;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "gpio-f303e", feature = "svd-f302"))] {
            // XXX This is a hack.
            // SPI4 should have a corresponding nvic interrupt number. But the svd does not
            // generated an enum-number for that. The RM0365 does also not list it as an interrupt.
            // Strangly though, the Stm32CubeMx program let's us choose to enable the NVIC
            // interrupt. This is probably a documentation bug.
            #[allow(unused)]
            pub(crate) const SPI4: Interrupt = Interrupt::SPI3;
        } else if #[cfg(feature = "gpio-f303e")] {
            pub(crate) const SPI4: Interrupt = Interrupt::SPI4;
        }
    }
}

#[cfg(feature = "gpio-f333")]
spi!([(1, 2)]);

#[cfg(feature = "gpio-f302")]
spi!([(2, 1), (3, 1)]);

#[cfg(any(feature = "gpio-f303", feature = "gpio-f373",))]
spi!([(1, 2), (2, 1), (3, 1)]);

#[cfg(feature = "gpio-f303e")]
spi!([(1, 2), (2, 1), (3, 1), (4, 2)]);
