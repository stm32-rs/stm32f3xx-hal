//! # Serial Peripheral Interface (SPI) bus
//!
//! ## Examples
//!
//! A usage example of the can peripheral can be found at [examples/spi.rs]
//!
//! [examples/spi.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.0/examples/spi.rs

use core::{fmt, marker::PhantomData, ops::Deref, ptr};

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
    time::{fixed_point::FixedPoint, rate::Hertz},
};

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

impl SckPin<SPI1> for gpio::PA5<AF5<PushPull>> {}
impl MisoPin<SPI1> for gpio::PA6<AF5<PushPull>> {}
impl MosiPin<SPI1> for gpio::PA7<AF5<PushPull>> {}
impl MosiPin<SPI1> for gpio::PB5<AF5<PushPull>> {}

impl MisoPin<SPI2> for gpio::PB14<AF5<PushPull>> {}
impl MosiPin<SPI2> for gpio::PB15<AF5<PushPull>> {}

impl MosiPin<SPI3> for gpio::PB5<AF6<PushPull>> {}
impl SckPin<SPI3> for gpio::PC10<AF6<PushPull>> {}
impl MisoPin<SPI3> for gpio::PC11<AF6<PushPull>> {}
impl MosiPin<SPI3> for gpio::PC12<AF6<PushPull>> {}

cfg_if::cfg_if! {
    if #[cfg(feature = "gpio-f373")] {
        impl SckPin<SPI1> for gpio::PA12<AF6<PushPull>> {}
        impl SckPin<SPI1> for gpio::PC7<AF5<PushPull>> {}
        impl MisoPin<SPI1> for gpio::PA13<AF6<PushPull>> {}
        impl MisoPin<SPI1> for gpio::PC8<AF5<PushPull>> {}
        impl MosiPin<SPI1> for gpio::PB0<AF5<PushPull>> {}
        impl MosiPin<SPI1> for gpio::PC9<AF5<PushPull>> {}
        impl MosiPin<SPI1> for gpio::PF6<AF5<PushPull>> {}

        impl SckPin<SPI2> for gpio::PA8<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PB8<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PB10<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PD7<AF5<PushPull>> {}
        impl SckPin<SPI2> for gpio::PD8<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PA9<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PC2<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PD3<AF5<PushPull>> {}
        impl MosiPin<SPI2> for gpio::PA10<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PC3<AF5<PushPull>> {}
        impl MisoPin<SPI2> for gpio::PD4<AF5<PushPull>> {}

        impl SckPin<SPI3> for gpio::PA1<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PA2<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PA3<AF6<PushPull>> {}
        impl MisoPin<SPI3> for gpio::PB4<AF6<PushPull>> {}
    } else {
        impl SckPin<SPI2> for gpio::PB13<AF5<PushPull>> {}
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
        impl SckPin<SPI4> for gpio::PE12<AF5<PushPull>> {}
        impl MisoPin<SPI4> for gpio::PE5<AF5<PushPull>> {}
        impl MisoPin<SPI4> for gpio::PE13<AF5<PushPull>> {}
        impl MosiPin<SPI4> for gpio::PE6<AF5<PushPull>> {}
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

#[cfg(all(
    not(feature = "stm32f301"),
    not(feature = "gpio-f333"),
    not(feature = "gpio-f303"),
))]
impl SckPin<SPI3> for gpio::PB3<AF6<PushPull>> {}

#[cfg(all(
    not(feature = "stm32f301"),
    any(feature = "gpio-f302", feature = "gpio-f303e"),
))]
impl MisoPin<SPI3> for gpio::PB4<AF6<PushPull>> {}

/// Configuration trait for the Word Size
/// used by the SPI peripheral
pub trait Word {
    /// Returns the register configuration
    /// to set the word size
    fn register_config() -> (FRXTH_A, DS_A);
}

impl Word for u8 {
    fn register_config() -> (FRXTH_A, DS_A) {
        (FRXTH_A::QUARTER, DS_A::EIGHTBIT)
    }
}

impl Word for u16 {
    fn register_config() -> (FRXTH_A, DS_A) {
        (FRXTH_A::HALF, DS_A::SIXTEENBIT)
    }
}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, Pins, Word = u8> {
    spi: SPI,
    pins: Pins,
    _word: PhantomData<Word>,
}

impl<SPI, Sck, Miso, Mosi, WORD> Spi<SPI, (Sck, Miso, Mosi), WORD> {
    /// Configures the SPI peripheral to operate in full duplex master mode
    pub fn new(
        spi: SPI,
        pins: (Sck, Miso, Mosi),
        mode: Mode,
        freq: Hertz,
        clocks: Clocks,
        apb: &mut <SPI as Instance>::APB,
    ) -> Self
    where
        SPI: Instance,
        Sck: SckPin<SPI>,
        Miso: MisoPin<SPI>,
        Mosi: MosiPin<SPI>,
        WORD: Word,
    {
        SPI::enable_clock(apb);

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

            match mode.phase {
                Phase::CaptureOnFirstTransition => w.cpha().first_edge(),
                Phase::CaptureOnSecondTransition => w.cpha().second_edge(),
            };

            match mode.polarity {
                Polarity::IdleLow => w.cpol().idle_low(),
                Polarity::IdleHigh => w.cpol().idle_high(),
            };

            w.br().variant(Self::compute_baud_rate(clocks, freq));

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
    pub fn reclock(&mut self, freq: Hertz, clocks: Clocks) {
        self.spi.cr1.modify(|_, w| w.spe().disabled());

        self.spi.cr1.modify(|_, w| {
            w.br().variant(Self::compute_baud_rate(clocks, freq));
            w.spe().enabled()
        });
    }

    fn compute_baud_rate(clocks: Clocks, freq: Hertz) -> spi1::cr1::BR_A {
        use spi1::cr1::BR_A;
        match SPI::clock(&clocks).0 / freq.integer() {
            0 => crate::unreachable!(),
            1..=2 => BR_A::DIV2,
            3..=5 => BR_A::DIV4,
            6..=11 => BR_A::DIV8,
            12..=23 => BR_A::DIV16,
            24..=39 => BR_A::DIV32,
            40..=95 => BR_A::DIV64,
            96..=191 => BR_A::DIV128,
            _ => BR_A::DIV256,
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

impl<SPI, Pins, Word> FullDuplex<Word> for Spi<SPI, Pins, Word>
where
    SPI: Instance,
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
            let read_ptr = &self.spi.dr as *const _ as *const Word;
            // NOTE(unsafe) read from register owned by this Spi struct
            let value = unsafe { ptr::read_volatile(read_ptr) };
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
            let write_ptr = &self.spi.dr as *const _ as *mut Word;
            // NOTE(unsafe) write to register owned by this Spi struct
            unsafe { ptr::write_volatile(write_ptr, word) };
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<SPI, Pins, Word> spi::transfer::Default<Word> for Spi<SPI, Pins, Word> where SPI: Instance {}
impl<SPI, Pins, Word> spi::write::Default<Word> for Spi<SPI, Pins, Word> where SPI: Instance {}

/// SPI instance
pub trait Instance:
    Deref<Target = spi1::RegisterBlock> + crate::interrupts::InterruptNumber + crate::private::Sealed
{
    /// Peripheral bus instance which is responsible for the peripheral
    type APB;

    #[doc(hidden)]
    fn enable_clock(apb1: &mut Self::APB);
    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Hertz;
}

macro_rules! spi {
    ($($SPIX:ident: ($APBX:ident, $spiXen:ident, $spiXrst:ident, $pclkX:ident),)+) => {
        $(
            impl crate::private::Sealed for pac::$SPIX {}
            impl crate::interrupts::InterruptNumber for pac::$SPIX {
                type Interrupt = Interrupt;
                const INTERRUPT: Self::Interrupt = interrupts::$SPIX;
            }

            impl Instance for pac::$SPIX {
                type APB = rcc::$APBX;
                fn enable_clock(apb: &mut Self::APB) {
                    apb.enr().modify(|_, w| w.$spiXen().enabled());
                    apb.rstr().modify(|_, w| w.$spiXrst().reset());
                    apb.rstr().modify(|_, w| w.$spiXrst().clear_bit());
                }

                fn clock(clocks: &Clocks) -> Hertz {
                    clocks.$pclkX()
                }
            }

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
                        [<spi $X en>],
                        [<spi $X rst>],
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

    #[cfg(feature = "gpio-f303e")]
    pub(crate) const SPI4: Interrupt = Interrupt::SPI4;
}

#[cfg(any(
    feature = "stm32f303x6",
    feature = "stm32f303x8",
    feature = "stm32f328",
    feature = "stm32f334",
))]
spi!([(1, 2)]);

#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302x6",
    feature = "stm32f302x8",
    feature = "stm32f318",
))]
spi!([(2, 1), (3, 1)]);

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
))]
spi!([(1, 2), (2, 1), (3, 1)]);

#[cfg(any(
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f398",
))]
spi!([(1, 2), (2, 1), (3, 1), (4, 2)]);
