//! Serial Peripheral Interface (SPI) bus

use core::ptr;

use crate::hal::spi::FullDuplex;
pub use crate::hal::spi::{Mode, Phase, Polarity};
use crate::pac::{SPI1, SPI2, SPI3};

use crate::gpio::gpioa::{PA5, PA6, PA7};
#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::gpio::gpiob::PB13;
use crate::gpio::gpiob::{PB14, PB15, PB5};
use crate::gpio::gpioc::{PC10, PC11, PC12};
use crate::gpio::{AF5, AF6};
use crate::rcc::Clocks;
#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::rcc::APB1;
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::rcc::APB2;
use crate::time::Hertz;

/// SPI error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
}

// FIXME these should be "closed" traits
/// SCK pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SckPin<SPI> {}

/// MISO pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MisoPin<SPI> {}

/// MOSI pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MosiPin<SPI> {}

unsafe impl SckPin<SPI1> for PA5<AF5> {}
// unsafe impl SckPin<SPI1> for PB3<AF5> {}

#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
unsafe impl SckPin<SPI2> for PB13<AF5> {}

// unsafe impl SckPin<SPI3> for PB3<AF6> {}
unsafe impl SckPin<SPI3> for PC10<AF6> {}

unsafe impl MisoPin<SPI1> for PA6<AF5> {}
// unsafe impl MisoPin<SPI1> for PB4<AF5> {}

unsafe impl MisoPin<SPI2> for PB14<AF5> {}

// unsafe impl MisoPin<SPI3> for PB4<AF6> {}
unsafe impl MisoPin<SPI3> for PC11<AF6> {}

unsafe impl MosiPin<SPI1> for PA7<AF5> {}
unsafe impl MosiPin<SPI1> for PB5<AF5> {}

unsafe impl MosiPin<SPI2> for PB15<AF5> {}

unsafe impl MosiPin<SPI3> for PB5<AF6> {}
unsafe impl MosiPin<SPI3> for PC12<AF6> {}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $APBX:ident, $spiXen:ident, $spiXrst:ident, $pclkX:ident),)+) => {
        $(
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                /// Configures the SPI peripheral to operate in full duplex master mode
                pub fn $spiX<F>(
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    freq: F,
                    clocks: Clocks,
                    apb2: &mut $APBX,
                ) -> Self
                where
                    F: Into<Hertz>,
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    apb2.enr().modify(|_, w| w.$spiXen().enabled());
                    apb2.rstr().modify(|_, w| w.$spiXrst().reset());
                    apb2.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // FRXTH: RXNE event is generated if the FIFO level is greater than or equal to
                    //        8-bit
                    // DS: 8-bit data size
                    // SSOE: Slave Select output disabled
                    spi.cr2.write(|w| w.frxth().quarter().ds().eight_bit().ssoe().disabled());

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

                        match clocks.$pclkX().0 / freq.into().0 {
                            0 => unreachable!(),
                            1..=2 => w.br().div2(),
                            3..=5 => w.br().div4(),
                            6..=11 => w.br().div8(),
                            12..=23 => w.br().div16(),
                            24..=39 => w.br().div32(),
                            40..=95 => w.br().div64(),
                            96..=191 => w.br().div128(),
                            _ => w.br().div256(),
                        };

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

                    Spi { spi, pins }
                }

                /// Releases the SPI peripheral and associated pins
                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }
            }

            impl<PINS> FullDuplex<u8> for Spi<$SPIX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().is_overrun() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().is_fault() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().is_no_match() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().is_not_empty() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().is_overrun() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().is_fault() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().is_no_match() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().is_empty() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> crate::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, PINS> {}

            impl<PINS> crate::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, PINS> {}
        )+
    }
}

#[cfg(feature = "stm32f334")]
hal! {
    SPI1: (spi1, APB2, spi1en, spi1rst, pclk2),
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
hal! {
    SPI2: (spi2, APB1, spi2en, spi2rst, pclk1),
    SPI3: (spi3, APB1, spi3en, spi3rst, pclk1),
}

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
hal! {
    SPI1: (spi1, APB2, spi1en, spi1rst, pclk2),
    SPI2: (spi2, APB1, spi2en, spi2rst, pclk1),
    SPI3: (spi3, APB1, spi3en, spi3rst, pclk1),
}

// FIXME not working
// TODO measure if this actually faster than the default implementation
// impl ::hal::blocking::spi::Write<u8> for Spi {
//     type Error = Error;

//     fn write(&mut self, bytes: &[u8]) -> Result<(), Error> {
//         for byte in bytes {
//             'l: loop {
//                 let sr = self.spi.sr.read();

//                 // ignore overruns because we don't care about the incoming data
//                 // if sr.ovr().bit_is_set() {
//                 // Err(nb::Error::Other(Error::Overrun))
//                 // } else
//                 if sr.modf().bit_is_set() {
//                     return Err(Error::ModeFault);
//                 } else if sr.crcerr().bit_is_set() {
//                     return Err(Error::Crc);
//                 } else if sr.txe().bit_is_set() {
//                     // NOTE(write_volatile) see note above
//                     unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, *byte) }
//                     break 'l;
//                 } else {
//                     // try again
//                 }
//             }
//         }

//         // wait until the transmission of the last byte is done
//         while self.spi.sr.read().bsy().bit_is_set() {}

//         // clear OVR flag
//         unsafe {
//             ptr::read_volatile(&self.spi.dr as *const _ as *const u8);
//         }
//         self.spi.sr.read();

//         Ok(())
//     }
// }
