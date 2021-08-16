//! # Inter-Integrated Circuit (I2C) bus
//!
//! ## Examples
//!
//! A usage example of the i2c peripheral can be found at [examples/i2c_scanner.rs]
//!
//! [examples/i2c_scanner.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.8.0/examples/i2c_scanner.rs

use core::{convert::TryFrom, ops::Deref};

use crate::{
    gpio::{gpioa, gpiob, OpenDrain, AF4},
    hal::blocking::i2c::{Read, Write, WriteRead},
    pac::{i2c1::RegisterBlock, rcc::cfgr3::I2C1SW_A, I2C1, RCC},
    rcc::{Clocks, APB1},
    time::rate::*,
};

#[cfg(not(feature = "gpio-f333"))]
use crate::{gpio::gpiof, pac::I2C2};

#[cfg(any(feature = "gpio-f302", feature = "gpio-f303e"))]
use crate::{
    gpio::{gpioc, AF3, AF8},
    pac::I2C3,
};

use cfg_if::cfg_if;

/// I2C error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Arbitration loss
    Arbitration,
    /// Bus error
    Bus,
    /// Bus busy
    Busy,
    /// Not Acknowledge received
    Nack,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

/// SCL pin
pub trait SclPin<I2C>: crate::private::Sealed {}

/// SDA pin
pub trait SdaPin<I2C>: crate::private::Sealed {}

impl SclPin<I2C1> for gpioa::PA15<AF4<OpenDrain>> {}
impl SclPin<I2C1> for gpiob::PB6<AF4<OpenDrain>> {}
impl SclPin<I2C1> for gpiob::PB8<AF4<OpenDrain>> {}
impl SdaPin<I2C1> for gpioa::PA14<AF4<OpenDrain>> {}
impl SdaPin<I2C1> for gpiob::PB7<AF4<OpenDrain>> {}
impl SdaPin<I2C1> for gpiob::PB9<AF4<OpenDrain>> {}

cfg_if! {
    if #[cfg(not(feature = "gpio-f333"))] {
        impl SclPin<I2C2> for gpioa::PA9<AF4<OpenDrain>> {}
        impl SclPin<I2C2> for gpiof::PF1<AF4<OpenDrain>> {}
        #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e", feature = "gpio-f373"))]
        impl SclPin<I2C2> for gpiof::PF6<AF4<OpenDrain>> {}
        impl SdaPin<I2C2> for gpioa::PA10<AF4<OpenDrain>> {}
        impl SdaPin<I2C2> for gpiof::PF0<AF4<OpenDrain>> {}
        #[cfg(feature = "gpio-f373")]
        impl SdaPin<I2C2> for gpiof::PF7<AF4<OpenDrain>> {}
    }
}

cfg_if! {
    if #[cfg(any(feature = "gpio-f302", feature = "gpio-f303e"))] {
        impl SclPin<I2C3> for gpioa::PA8<AF3<OpenDrain>> {}
        impl SdaPin<I2C3> for gpiob::PB5<AF8<OpenDrain>> {}
        impl SdaPin<I2C3> for gpioc::PC9<AF3<OpenDrain>> {}
    }
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();
            let icr = &$i2c.icr;

            if isr.arlo().is_lost() {
                icr.write(|w| w.arlocf().clear());
                return Err(Error::Arbitration);
            } else if isr.berr().is_error() {
                icr.write(|w| w.berrcf().clear());
                return Err(Error::Bus);
            } else if isr.nackf().is_nack() {
                while $i2c.isr.read().stopf().is_no_stop() {}
                icr.write(|w| w.nackcf().clear());
                icr.write(|w| w.stopcf().clear());
                return Err(Error::Nack);
            } else if isr.$flag().$variant() {
                break;
            }
        }
    };
}

impl<I2C, SCL, SDA> I2c<I2C, (SCL, SDA)> {
    /// Configures the I2C peripheral to work in master mode
    pub fn new(i2c: I2C, pins: (SCL, SDA), freq: Hertz, clocks: Clocks, apb1: &mut APB1) -> Self
    where
        I2C: Instance,
        SCL: SclPin<I2C>,
        SDA: SdaPin<I2C>,
    {
        crate::assert!(freq.integer() <= 1_000_000);

        I2C::enable_clock(apb1);

        // TODO review compliance with the timing requirements of I2C
        // t_I2CCLK = 1 / PCLK1
        // t_PRESC  = (PRESC + 1) * t_I2CCLK
        // t_SCLL   = (SCLL + 1) * t_PRESC
        // t_SCLH   = (SCLH + 1) * t_PRESC
        //
        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
        let i2cclk = I2C::clock(&clocks).0;
        let ratio = i2cclk / freq.integer() - 4;
        let (presc, scll, sclh, sdadel, scldel) = if freq >= 100.kHz() {
            // fast-mode or fast-mode plus
            // here we pick SCLL + 1 = 2 * (SCLH + 1)
            let presc = ratio / 387;

            let sclh = ((ratio / (presc + 1)) - 3) / 3;
            let scll = 2 * (sclh + 1) - 1;

            let (sdadel, scldel) = if freq > 400.kHz() {
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

        crate::assert!(presc < 16);
        crate::assert!(scldel < 16);
        crate::assert!(sdadel < 16);
        let sclh = crate::unwrap!(u8::try_from(sclh).ok());
        let scll = crate::unwrap!(u8::try_from(scll).ok());

        // Configure for "fast mode" (400 KHz)
        // NOTE(write): writes all non-reserved bits.
        i2c.timingr.write(|w| {
            w.presc()
                .bits(presc as u8)
                .sdadel()
                .bits(sdadel as u8)
                .scldel()
                .bits(scldel as u8)
                .scll()
                .bits(scll)
                .sclh()
                .bits(sclh)
        });

        // Enable the peripheral
        i2c.cr1.modify(|_, w| w.pe().set_bit());

        Self { i2c, pins }
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
    pub unsafe fn peripheral(&mut self) -> &mut I2C {
        &mut self.i2c
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, (SCL, SDA)) {
        (self.i2c, self.pins)
    }
}

impl<I2C, PINS> Read for I2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        crate::assert!(!buffer.is_empty());

        // Detect Bus busy
        if self.i2c.isr.read().busy().is_busy() {
            return Err(Error::Busy);
        }

        let end = buffer.len() / 0xFF;

        // Process 255 bytes at a time
        for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
            // Prepare to receive `bytes`
            self.i2c.cr2.modify(|_, w| {
                if i == 0 {
                    w.add10().bit7();
                    w.sadd().bits((addr << 1) as u16);
                    w.rd_wrn().read();
                    w.start().start();
                }
                w.nbytes().bits(buffer.len() as u8);
                if i != end {
                    w.reload().not_completed()
                } else {
                    w.reload().completed().autoend().automatic()
                }
            });

            for byte in buffer {
                // Wait until we have received something
                busy_wait!(self.i2c, rxne, is_not_empty);

                *byte = self.i2c.rxdr.read().rxdata().bits();
            }

            if i != end {
                // Wait until the last transmission is finished
                busy_wait!(self.i2c, tcr, is_complete);
            }
        }

        // automatic STOP
        // Wait until the last transmission is finished
        busy_wait!(self.i2c, stopf, is_stop);

        self.i2c.icr.write(|w| w.stopcf().clear());

        Ok(())
    }
}

impl<I2C, PINS> Write for I2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Detect Bus busy
        if self.i2c.isr.read().busy().is_busy() {
            return Err(Error::Busy);
        }

        if bytes.is_empty() {
            // 0 byte write
            self.i2c.cr2.modify(|_, w| {
                w.add10().bit7();
                w.sadd().bits((addr << 1) as u16);
                w.rd_wrn().write();
                w.nbytes().bits(0);
                w.reload().completed();
                w.autoend().automatic();
                w.start().start()
            });
        } else {
            let end = bytes.len() / 0xFF;

            // Process 255 bytes at a time
            for (i, bytes) in bytes.chunks(0xFF).enumerate() {
                // Prepare to send `bytes`
                self.i2c.cr2.modify(|_, w| {
                    if i == 0 {
                        w.add10().bit7();
                        w.sadd().bits((addr << 1) as u16);
                        w.rd_wrn().write();
                        w.start().start();
                    }
                    w.nbytes().bits(bytes.len() as u8);
                    if i != end {
                        w.reload().not_completed()
                    } else {
                        w.reload().completed().autoend().automatic()
                    }
                });

                for byte in bytes {
                    // Wait until we are allowed to send data
                    // (START has been ACKed or last byte went through)
                    busy_wait!(self.i2c, txis, is_empty);

                    // Put byte on the wire
                    // NOTE(write): Writes all non-reserved bits.
                    self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                }

                if i != end {
                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tcr, is_complete);
                }
            }
        }

        // automatic STOP
        // Wait until the last transmission is finished
        busy_wait!(self.i2c, stopf, is_stop);

        self.i2c.icr.write(|w| w.stopcf().clear());

        Ok(())
    }
}

impl<I2C, PINS> WriteRead for I2c<I2C, PINS>
where
    I2C: Instance,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        crate::assert!(!bytes.is_empty() && !buffer.is_empty());

        // Detect Bus busy
        if self.i2c.isr.read().busy().is_busy() {
            return Err(Error::Busy);
        }

        let end = bytes.len() / 0xFF;

        // Process 255 bytes at a time
        for (i, bytes) in bytes.chunks(0xFF).enumerate() {
            // Prepare to send `bytes`
            self.i2c.cr2.modify(|_, w| {
                if i == 0 {
                    w.add10().bit7();
                    w.sadd().bits((addr << 1) as u16);
                    w.rd_wrn().write();
                    w.start().start();
                }
                w.nbytes().bits(bytes.len() as u8);
                if i != end {
                    w.reload().not_completed()
                } else {
                    w.reload().completed().autoend().software()
                }
            });

            for byte in bytes {
                // Wait until we are allowed to send data
                // (START has been ACKed or last byte went through)
                busy_wait!(self.i2c, txis, is_empty);

                // Put byte on the wire
                // NOTE(write): Writes all non-reserved bits.
                self.i2c.txdr.write(|w| w.txdata().bits(*byte));
            }

            if i != end {
                // Wait until the last transmission is finished
                busy_wait!(self.i2c, tcr, is_complete);
            }
        }

        // Wait until the last transmission is finished
        busy_wait!(self.i2c, tc, is_complete);

        // restart

        let end = buffer.len() / 0xFF;

        // Process 255 bytes at a time
        for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
            // Prepare to receive `bytes`
            self.i2c.cr2.modify(|_, w| {
                if i == 0 {
                    w.add10().bit7();
                    w.sadd().bits((addr << 1) as u16);
                    w.rd_wrn().read();
                    w.start().start();
                }
                w.nbytes().bits(buffer.len() as u8);
                if i != end {
                    w.reload().not_completed()
                } else {
                    w.reload().completed().autoend().automatic()
                }
            });

            for byte in buffer {
                // Wait until we have received something
                busy_wait!(self.i2c, rxne, is_not_empty);

                *byte = self.i2c.rxdr.read().rxdata().bits();
            }

            if i != end {
                // Wait until the last transmission is finished
                busy_wait!(self.i2c, tcr, is_complete);
            }
        }

        // automatic STOP
        // Wait until the last transmission is finished
        busy_wait!(self.i2c, stopf, is_stop);

        self.i2c.icr.write(|w| w.stopcf().clear());

        Ok(())
    }
}

/// I2C instance
pub trait Instance: Deref<Target = RegisterBlock> + crate::private::Sealed {
    #[doc(hidden)]
    fn enable_clock(apb1: &mut APB1);
    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Hertz;
}

macro_rules! i2c {
    ($($I2CX:ident: ($i2cXen:ident, $i2cXrst:ident, $i2cXsw:ident),)+) => {
        $(
            impl crate::private::Sealed for $I2CX {}
            impl Instance for $I2CX {
                fn enable_clock(apb1: &mut APB1) {
                    apb1.enr().modify(|_, w| w.$i2cXen().enabled());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().reset());
                    apb1.rstr().modify(|_, w| w.$i2cXrst().clear_bit());
                }

                fn clock(clocks: &Clocks) -> Hertz {
                    // NOTE(unsafe) atomic read with no side effects
                    match unsafe { (*RCC::ptr()).cfgr3.read().$i2cXsw().variant() } {
                        I2C1SW_A::HSI => crate::rcc::HSI,
                        I2C1SW_A::SYSCLK => clocks.sysclk(),
                    }
                }
            }
        )+
    };

    ([ $($X:literal),+ ]) => {
        paste::paste! {
            i2c!(
                $([<I2C $X>]: ([<i2c $X en>], [<i2c $X rst>], [<i2c $X sw>]),)+
            );
        }
    };
}

#[cfg(feature = "gpio-f333")]
i2c!([1]);

#[cfg(any(feature = "gpio-f303", feature = "gpio-f373"))]
i2c!([1, 2]);

#[cfg(any(feature = "gpio-f302", feature = "gpio-f303e"))]
i2c!([1, 2, 3]);
