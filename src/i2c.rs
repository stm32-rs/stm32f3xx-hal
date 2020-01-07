//! Inter-Integrated Circuit (I2C) bus

use crate::stm32::{I2C1, I2C2};
use cast::u8;

use crate::gpio::gpioa::{PA10, PA9};
use crate::gpio::gpiob::{PB6, PB7, PB8, PB9};
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f334",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
    feature = "stm32f373",
    feature = "stm32f378"
))]
use crate::gpio::gpiof::PF6;
use crate::gpio::gpiof::{PF0, PF1};
use crate::gpio::AF4;
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{Clocks, APB1};
use crate::time::Hertz;

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

// unsafe impl SclPin<I2C1> for PA15<AF4> {}
unsafe impl SclPin<I2C1> for PB6<AF4> {}
unsafe impl SclPin<I2C1> for PB8<AF4> {}

unsafe impl SclPin<I2C2> for PA9<AF4> {}
unsafe impl SclPin<I2C2> for PF1<AF4> {}
#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f334",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
    feature = "stm32f373",
    feature = "stm32f378"
))]
unsafe impl SclPin<I2C2> for PF6<AF4> {}

// unsafe impl SdaPin<I2C1> for PA14<AF4> {}
unsafe impl SdaPin<I2C1> for PB7<AF4> {}
unsafe impl SdaPin<I2C1> for PB9<AF4> {}

unsafe impl SdaPin<I2C2> for PA10<AF4> {}
unsafe impl SdaPin<I2C2> for PF0<AF4> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().is_error() {
                return Err(Error::Bus);
            } else if isr.arlo().is_lost() {
                return Err(Error::Arbitration);
            } else if isr.$flag().$variant() {
                break;
            } else {
                // try again
            }
        }
    };
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    freq: F,
                    clocks: Clocks,
                    apb1: &mut APB1,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    apb1.enr().modify(|_, w| w.$i2cXen().enabled());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().reset());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());

                    let freq = freq.into().0;

                    assert!(freq <= 1_000_000);

                    // TODO review compliance with the timing requirements of I2C
                    // t_I2CCLK = 1 / PCLK1
                    // t_PRESC  = (PRESC + 1) * t_I2CCLK
                    // t_SCLL   = (SCLL + 1) * t_PRESC
                    // t_SCLH   = (SCLH + 1) * t_PRESC
                    //
                    // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
                    // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
                    let i2cclk = clocks.pclk1().0;
                    let ratio = i2cclk / freq - 4;
                    let (presc, scll, sclh, sdadel, scldel) = if freq >= 100_000 {
                        // fast-mode or fast-mode plus
                        // here we pick SCLL + 1 = 2 * (SCLH + 1)
                        let presc = ratio / 387;

                        let sclh = ((ratio / (presc + 1)) - 3) / 3;
                        let scll = 2 * (sclh + 1) - 1;

                        let (sdadel, scldel) = if freq > 400_000 {
                            // fast-mode plus
                            let sdadel = 0;
                            let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                            (sdadel, scldel)
                        } else {
                            // fast-mode
                            let sdadel = i2cclk / 8_000_000 / (presc + 1);
                            let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                            (sdadel, scldel)
                        };

                        (presc, scll, sclh, sdadel, scldel)
                    } else {
                        // standard-mode
                        // here we pick SCLL = SCLH
                        let presc = ratio / 514;

                        let sclh = ((ratio / (presc + 1)) - 2) / 2;
                        let scll = sclh;

                        let sdadel = i2cclk / 2_000_000 / (presc + 1);
                        let scldel = i2cclk / 800_000 / (presc + 1) - 1;

                        (presc, scll, sclh, sdadel, scldel)
                    };

                    let presc = u8(presc).unwrap();
                    assert!(presc < 16);
                    let scldel = u8(scldel).unwrap();
                    assert!(scldel < 16);
                    let sdadel = u8(sdadel).unwrap();
                    assert!(sdadel < 16);
                    let sclh = u8(sclh).unwrap();
                    let scll = u8(scll).unwrap();

                    // Configure for "fast mode" (400 KHz)
                    i2c.timingr.write(|w| {
                        w.presc()
                            .bits(presc)
                            .scll()
                            .bits(scll)
                            .sclh()
                            .bits(sclh)
                            .sdadel()
                            .bits(sdadel)
                            .scldel()
                            .bits(scldel)
                    });

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().enabled());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Read for I2c<$I2CX, PINS> {
                type Error = Error;
                fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // master is communicating)?

                    // START and prepare to receive `bytes`
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16::from(addr << 1))
                            .rd_wrn()
                            .read()
                            .nbytes()
                            .bits(buffer.len() as u8)
                            .start()
                            .start()
                            .autoend()
                            .software()
                    });

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne, is_not_empty);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16::from(addr << 1))
                            .rd_wrn()
                            .write()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .start()
                            .start()
                            .autoend()
                            .automatic()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis, is_empty);

                        // put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the last transmission is finished ???
                    // busy_wait!(self.i2c, busy);

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // master is communicating)?

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16::from(addr << 1))
                            .rd_wrn()
                            .write()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .start()
                            .start()
                            .autoend()
                            .software()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data (START has been ACKed or last byte
                        // when through)
                        busy_wait!(self.i2c, txis, is_empty);

                        // put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tc, is_complete);

                    // reSTART and prepare to receive bytes into `buffer`
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16::from(addr << 1))
                            .rd_wrn()
                            .read()
                            .nbytes()
                            .bits(buffer.len() as u8)
                            .start()
                            .start()
                            .autoend()
                            .automatic()
                    });

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne, is_not_empty);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }
        )+
    }
}

#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398",
))]
hal! {
    I2C1: (i2c1, i2c1en, i2c1rst),
    I2C2: (i2c2, i2c2en, i2c2rst),
}

#[cfg(feature = "stm32f334")]
hal! {
    I2C1: (i2c1, i2c1en, i2c1rst),
}
