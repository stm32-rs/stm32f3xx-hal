//! Inter-Integrated Circuit (I2C) bus

// This document describes a correct i2c implementation and is what
// parts of this code is based on
// https://www.st.com/content/ccc/resource/technical/document/application_note/5d/ae/a3/6f/08/69/4e/9b/CD00209826.pdf/files/CD00209826.pdf/jcr:content/translations/en.CD00209826.pdf

use crate::gpio::{gpioa, gpiob, AF4, OpenDrain};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::pac::{DWT, I2C1, I2C2};
use crate::rcc::{Clocks, APB1};
use crate::time::Hertz;
use core::{ops::Deref, convert::TryFrom};
use nb::Error::{Other, WouldBlock};
use nb::{Error as NbError, Result as NbResult};

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    Acknowledge,
    /// Overrun/underrun
    Overrun,
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
    #[doc(hidden)]
    _Extensible,
}

#[derive(Debug, Eq, PartialEq)]
pub enum DutyCycle {
    Ratio2to1,
    Ratio16to9,
}

#[derive(Debug, PartialEq)]
pub enum Mode {
    Standard {
        frequency: Hertz,
    },
    Fast {
        frequency: Hertz,
        duty_cycle: DutyCycle,
    },
}

impl Mode {
    pub fn standard<F: Into<Hertz>>(frequency: F) -> Self {
        Mode::Standard {
            frequency: frequency.into(),
        }
    }

    pub fn fast<F: Into<Hertz>>(frequency: F, duty_cycle: DutyCycle) -> Self {
        Mode::Fast {
            frequency: frequency.into(),
            duty_cycle,
        }
    }

    pub fn get_frequency(&self) -> Hertz {
        match *self {
            Mode::Standard { frequency } => frequency,
            Mode::Fast { frequency, .. } => frequency,
        }
    }
}

/// Helper trait to ensure that the correct I2C pins are used for the corresponding interface
pub trait Pins<I2C> {
    const REMAP: bool;
}

impl Pins<I2C1> for (gpiob::PB6<AF4>, gpiob::PB7<AF4>) {
    const REMAP: bool = false;
}

impl Pins<I2C1> for (gpiob::PB8<AF4>, gpiob::PB9<AF4>) {
    const REMAP: bool = true;
}

impl Pins<I2C2> for (gpioa::PA15<AF4>, gpioa::PA14<AF4>) {
    const REMAP: bool = false;
}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
    mode: Mode,
    pclk1: u32,
}

/// embedded-hal compatible blocking I2C implementation
///
/// **NOTE**: Before using blocking I2C, you need to enable the DWT cycle counter using the
/// [DWT::enable_cycle_counter] method.
pub struct BlockingI2c<I2C, PINS> {
    nb: I2c<I2C, PINS>,
    start_timeout: u32,
    start_retries: u8,
    addr_timeout: u32,
    data_timeout: u32,
}

impl<PINS> I2c<I2C1, PINS> {
    /// Creates a generic I2C1 object on pins PB6 and PB7 or PB8 and PB9 (if remapped)
    pub fn i2c1(
        i2c: I2C1,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        I2c::<I2C1, _>::_i2c(i2c, pins, mode, clocks, apb)
    }
}

impl<PINS> BlockingI2c<I2C1, PINS> {
    /// Creates a blocking I2C1 object on pins PB6 and PB7 or PB8 and PB9 using the embedded-hal `BlockingI2c` trait.
    pub fn i2c1(
        i2c: I2C1,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C1>,
    {
        BlockingI2c::<I2C1, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<PINS> I2c<I2C2, PINS> {
    /// Creates a generic I2C2 object on pins PB10 and PB11 using the embedded-hal `BlockingI2c` trait.
    pub fn i2c2(i2c: I2C2, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut APB1) -> Self
    where
        PINS: Pins<I2C2>,
    {
        I2c::<I2C2, _>::_i2c(i2c, pins, mode, clocks, apb)
    }
}

impl<PINS> BlockingI2c<I2C2, PINS> {
    /// Creates a blocking I2C2 object on pins PB10 and PB1
    pub fn i2c2(
        i2c: I2C2,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self
    where
        PINS: Pins<I2C2>,
    {
        BlockingI2c::<I2C2, _>::_i2c(
            i2c,
            pins,
            mode,
            clocks,
            apb,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

/// Generates a blocking I2C instance from a universal I2C object
fn blocking_i2c<I2C, PINS>(
    i2c: I2c<I2C, PINS>,
    clocks: Clocks,
    start_timeout_us: u32,
    start_retries: u8,
    addr_timeout_us: u32,
    data_timeout_us: u32,
) -> BlockingI2c<I2C, PINS> {
    let sysclk_mhz = clocks.sysclk().0 / 1_000_000;
    BlockingI2c {
        nb: i2c,
        start_timeout: start_timeout_us * sysclk_mhz,
        start_retries,
        addr_timeout: addr_timeout_us * sysclk_mhz,
        data_timeout: data_timeout_us * sysclk_mhz,
    }
}

macro_rules! wait_for_flag {
    ($i2c:expr, $flag:ident) => {{
        let sr1 = $i2c.isr.read();

        if sr1.berr().bit_is_set() {
            $i2c.icr.write(|w| w.berrcf().set_bit());
            Err(Other(Error::Bus))
        } else if sr1.arlo().bit_is_set() {
            $i2c.icr.write(|w| w.arlocf().set_bit());
            Err(Other(Error::Arbitration))
        // } else if sr1.af().bit_is_set() { // todo
        //     $i2c.icr.write(|w| w.af().clear_bit());
        //     Err(Other(Error::Acknowledge))
        } else if sr1.ovr().bit_is_set() {
            $i2c.icr.write(|w| w.ovrcf().set_bit());
            Err(Other(Error::Overrun))
        } else if sr1.$flag().bit_is_set() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }};
}

macro_rules! busy_wait {
    ($nb_expr:expr, $exit_cond:expr) => {{
        loop {
            let res = $nb_expr;
            if res != Err(WouldBlock) {
                break res;
            }
            if $exit_cond {
                break res;
            }
        }
    }};
}

macro_rules! busy_wait_cycles {
    ($nb_expr:expr, $cycles:expr) => {{
        let started = DWT::get_cycle_count();
        let cycles = $cycles;
        busy_wait!(
            $nb_expr,
            DWT::get_cycle_count().wrapping_sub(started) >= cycles
        )
    }};
}

pub type I2cRegisterBlock = crate::pac::i2c1::RegisterBlock;

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
    // I2C::Bus: GetBusFreq,
{
    /// Configures the I2C peripheral to work in master mode
    fn _i2c(i2c: I2C, pins: PINS, mode: Mode, clocks: Clocks, apb: &mut APB1) -> Self {
        apb.enr().modify(|_, w| w.pwren().set_bit());

        // todo: Use macro for i2c2rst etc! Currently broken
        apb.rstr().modify(|_, w| w.i2c1rst().set_bit());
        apb.rstr().modify(|_, w| w.i2c1rst().clear_bit());

        let pclk1 = I2C::Bus::get_frequency(&clocks).0;

        assert!(mode.get_frequency().0 <= 400_000);

        let mut i2c = I2c {
            i2c,
            pins,
            mode,
            pclk1,
        };
        i2c.init(apb);
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    /// Initializes I2C. Configures the `I2C_TRISE`, `I2C_CRX`, and `I2C_CCR` registers
    /// according to the system frequency and I2C mode.
    fn init(&mut self, apb1: &mut APB1) {
        let freq = self.mode.get_frequency();
        let pclk1_mhz = (self.pclk1 / 1000000) as u16;

        self.i2c.cr1.write(|w| w.pe().clear_bit());

        let freq = freq.into().0;

        crate::assert!(freq <= 1_000_000);

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

    }

    /// Perform an I2C software reset
    fn reset(&mut self, apb1: &mut apb1) {
        self.i2c.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
        self.i2c.cr1.reset();
        self.init(apb1);
    }

    /// Generate START condition
    fn send_start(&mut self) {
        self.i2c.cr2.modify(|_, w| w.start().set_bit());
    }

    /// Check if START condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_after_sent_start(&mut self) -> NbResult<(), Error> {
        wait_for_flag!(self.i2c, busy)
    }

    /// Check if STOP condition is generated. If the condition is not generated, this
    /// method returns `WouldBlock` so the program can act accordingly
    /// (busy wait, async, ...)
    fn wait_for_stop(&mut self) -> NbResult<(), Error> {
        if self.i2c.cr2.read().stop().is_no_stop() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }

    /// Sends the (7-Bit) address on the I2C bus. The 8th bit on the bus is set
    /// depending on wether it is a read or write transfer.
    fn send_addr(&self, addr: u8, read: bool) {
        self.i2c
            .txdr
            .write(|w| w.txdata().bits(addr << 1 | (if read { 1 } else { 0 })));
    }

    /// Generate STOP condition
    fn send_stop(&self) {
        self.i2c.cr2.modify(|_, w| w.stop().set_bit());
    }

    /// Releases the I2C peripheral and associated pins
    pub fn free(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }
}

impl<I2C, PINS> BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
    // I2C::Bus: GetBusFreq,
{
    fn _i2c(
        i2c: I2C,
        pins: PINS,
        mode: Mode,
        clocks: Clocks,
        apb: &mut APB1,
        start_timeout_us: u32,
        start_retries: u8,
        addr_timeout_us: u32,
        data_timeout_us: u32,
    ) -> Self {
        blocking_i2c(
            I2c::<I2C, _>::_i2c(i2c, pins, mode, clocks, apb),
            clocks,
            start_timeout_us,
            start_retries,
            addr_timeout_us,
            data_timeout_us,
        )
    }
}

impl<I2C, PINS> BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    fn send_start_and_wait(&mut self) -> NbResult<(), Error> {
        // According to http://www.st.com/content/ccc/resource/technical/document/errata_sheet/f5/50/c9/46/56/db/4a/f6/CD00197763.pdf/files/CD00197763.pdf/jcr:content/translations/en.CD00197763.pdf
        // 2.14.4 Wrong behavior of I2C peripheral in master mode after a misplaced STOP
        let mut retries_left = self.start_retries;
        let mut last_ret: NbResult<(), Error> = Err(WouldBlock);
        while retries_left > 0 {
            self.nb.send_start();
            last_ret = busy_wait_cycles!(self.nb.wait_after_sent_start(), self.start_timeout);
            if let Err(_) = last_ret {
                self.nb.reset();
            } else {
                break;
            }
            retries_left -= 1;
        }
        last_ret
    }

    fn send_addr_and_wait(&mut self, addr: u8, read: bool) -> NbResult<(), Error> {
        self.nb.i2c.isr.read();
        self.nb.send_addr(addr, read);
        let ret = busy_wait_cycles!(wait_for_flag!(self.nb.i2c, addr), self.addr_timeout);
        if ret == Err(Other(Error::Acknowledge)) {
            self.nb.send_stop();
        }
        ret
    }

    fn write_bytes_and_wait(&mut self, bytes: &[u8]) -> NbResult<(), Error> {
        self.nb.i2c.isr.read();
        self.nb.i2c.isr.read();

        self.nb.i2c.txdr.write(|w| w.txdata().bits(bytes[0]));

        for byte in &bytes[1..] {
            busy_wait_cycles!(wait_for_flag!(self.nb.i2c, txe), self.data_timeout)?;
            self.nb.i2c.txdr.write(|w| w.txdata().bits(*byte));
        }
        busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tc), self.data_timeout)?;

        Ok(())
    }

    fn write_without_stop(&mut self, addr: u8, bytes: &[u8]) -> NbResult<(), Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, false)?;

        let ret = self.write_bytes_and_wait(bytes);
        if ret == Err(Other(Error::Acknowledge)) {
            self.nb.send_stop();
        }
        ret
    }
}

impl<I2C, PINS> Write for BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = NbError<Error>;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_without_stop(addr, bytes)?;
        self.nb.send_stop();
        busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;

        Ok(())
    }
}

impl<I2C, PINS> Read for BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = NbError<Error>;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.send_start_and_wait()?;
        self.send_addr_and_wait(addr, true)?;

        match buffer.len() {
            1 => {
                // self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                self.nb.i2c.isr.read();
                self.nb.i2c.isr.read();
                self.nb.send_stop();

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rxne), self.data_timeout)?;
                buffer[0] = self.nb.i2c.rxdr.read().rxdata().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                // self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
            2 => {
                // self.nb
                //     .i2c
                //     .cr1
                //     .modify(|_, w| w.pos().set_bit().ack().set_bit());
                self.nb.i2c.isr.read();
                self.nb.i2c.isr.read();
                // self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tc), self.data_timeout)?;
                self.nb.send_stop();
                buffer[0] = self.nb.i2c.rxdr.read().rxdata().bits();
                buffer[1] = self.nb.i2c.rxdr.read().rxdata().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                // self.nb
                //     .i2c
                //     .cr1
                //     .modify(|_, w| w.pos().clear_bit().ack().clear_bit());
                // self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
            buffer_len => {
                // self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
                self.nb.i2c.isr.read();
                self.nb.i2c.isr.read();

                let (first_bytes, last_two_bytes) = buffer.split_at_mut(buffer_len - 3);
                for byte in first_bytes {
                    busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rxne), self.data_timeout)?;
                    *byte = self.nb.i2c.rxdr.read().rxdata().bits();
                }

                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, tc), self.data_timeout)?;
                // self.nb.i2c.cr1.modify(|_, w| w.ack().clear_bit());
                last_two_bytes[0] = self.nb.i2c.rxdr.read().rxdata().bits();
                self.nb.send_stop();
                last_two_bytes[1] = self.nb.i2c.rxdr.read().rxdata().bits();
                busy_wait_cycles!(wait_for_flag!(self.nb.i2c, rxne), self.data_timeout)?;
                last_two_bytes[2] = self.nb.i2c.rxdr.read().rxdata().bits();

                busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
                // self.nb.i2c.cr1.modify(|_, w| w.ack().set_bit());
            }
        }

        Ok(())
    }
}

impl<I2C, PINS> WriteRead for BlockingI2c<I2C, PINS>
where
    I2C: Deref<Target = I2cRegisterBlock>,
{
    type Error = NbError<Error>;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        if !bytes.is_empty() {
            self.write_without_stop(addr, bytes)?;
        }

        if !buffer.is_empty() {
            self.read(addr, buffer)?;
        } else if !bytes.is_empty() {
            self.nb.send_stop();
            busy_wait_cycles!(self.nb.wait_for_stop(), self.data_timeout)?;
        }

        Ok(())
    }
}
