//! Inter-Integrated Circuit (I2C) bus interruption

use crate::i2c::{I2c, Instance, SclPin, SdaPin};
use crate::rcc::{self, Clocks};
use crate::time::rate::*;
use core::fmt;

/// I2c errors.
#[derive(Debug, Copy, Clone)]
pub enum I2cError {
    /// Device is busy, can't start something else
    DeviceBusy,
    /// Received a nack
    Nack,
    /// Error happened on the bus
    BusError,
    /// Arbitration loss,
    ArbitrationLoss,
    /// Overrun detected (salve mode)
    Overrun,
    /// Unable to compute the stop state because previous state was not expected
    StateError,
    /// Transfer complete status but nothing do do next
    TransferCompleteNoRead,
    /// Send buffer is empty
    TxBufferEmpty,
}

/// State of i2c communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// I2c is Idle
    Idle,
    /// Send has started
    TxStart,
    /// Ready for send
    TxReady,
    /// Data written in send buffer
    TxSent,
    /// Send is complete, but device is not stopped
    TxComplete,
    /// Send is complete
    TxStop,
    /// Receive has started
    RxStart,
    /// Ready for receive
    RxReady,
    /// Received is complete
    RxStop,
    /// Nack for send
    TxNack,
}

impl core::fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// I2c1Int provides interface to communicate with i2c devices using interruptions.
pub struct I2cInt<I2C, PINS> {
    dev: I2c<I2C, PINS>,
    state: State,
    /// Last error that happened on i2c communications.
    pub last_error: Option<I2cError>,
    current_write_addr: Option<u8>,
    tx_ind: usize,
    tx_buf: Option<&'static [u8]>,
    rx_ind: usize,
    rx_buf: [u8; 256], // for the moment use a static buffer here.
    recv: Option<(u8, usize)>,
}

impl<I2C, SCL, SDA> I2cInt<I2C, (SCL, SDA)>
where
    I2C: Instance,
{
    /// Configures the I2C peripheral to work in master mode
    pub fn new_int(
        i2c: I2C,
        pins: (SCL, SDA),
        freq: Hertz,
        clocks: Clocks,
        bus: &mut <I2C as rcc::RccBus>::Bus,
    ) -> I2cInt<I2C, (SCL, SDA)>
    where
        SCL: SclPin<I2C>,
        SDA: SdaPin<I2C>,
    {
        let i2c = I2c::new(i2c, pins, freq, clocks, bus);

        I2cInt {
            dev: i2c,
            state: State::Idle,
            last_error: None,
            current_write_addr: None,
            tx_ind: 0,
            tx_buf: None,
            rx_ind: 0,
            rx_buf: [0; 256],
            recv: None,
        }
    }

    /// Enable the i2c device.
    pub fn enable(&mut self) {
        // Enable the peripheral
        self.dev.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Disable the i2c device.
    pub fn disable(&mut self) {
        self.dev.i2c.cr1.modify(|_, w| w.pe().disabled());
        self.last_error = None;
    }

    /// Enables all interrupts for i2c device.
    pub fn enable_interrupts(&mut self) {
        self.dev.i2c.cr1.modify(|_, w| {
            w.errie()
                .enabled()
                .tcie()
                .enabled()
                .stopie()
                .enabled()
                .nackie()
                .enabled()
                .rxie()
                .enabled()
                .txie()
                .enabled()
        });
    }

    /// Disables all interrupts for i2c device.
    pub fn disable_interrupts(&mut self) {
        self.dev.i2c.cr1.modify(|_, w| {
            w.errie()
                .disabled()
                .tcie()
                .disabled()
                .stopie()
                .disabled()
                .nackie()
                .disabled()
                .rxie()
                .disabled()
                .txie()
                .disabled()
        });
    }

    /// Write bytes through i2c. Supports only write < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `bytes` - Bytes to send.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    pub fn write(&mut self, addr: u8, bytes: &'static [u8]) -> Result<(), I2cError> {
        self._write(addr, true, bytes)
    }

    /// Write bytes through i2c. Supports only write < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `auto_stop` - i2c autostop enabled.
    /// * `bytes` - Bytes to send.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    fn _write(&mut self, addr: u8, auto_stop: bool, bytes: &'static [u8]) -> Result<(), I2cError> {
        if self.is_busy() {
            self.last_error = Some(I2cError::DeviceBusy);
            return Err(I2cError::DeviceBusy);
        }
        self.tx_ind = 0;
        self.tx_buf = Some(bytes);
        self.write_start(addr, bytes.len() as u8, auto_stop);
        Ok(())
    }

    /// Start a write sequence on i2c channel.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `n_bytes` - number of bytes which will be sent.
    /// * `auto_stop` - i2c autostop enabled.
    fn write_start(&mut self, addr: u8, n_bytes: u8, auto_stop: bool) {
        self.current_write_addr = Some(addr);
        self.dev.i2c.cr2.modify(|_, w| {
            w.sadd()
                .bits(u16::from(addr << 1))
                .rd_wrn()
                .write()
                .nbytes()
                .bits(n_bytes)
                .start()
                .start()
                .autoend()
                .bit(auto_stop)
        });
        self.state = State::TxStart;
    }

    /// Reads from i2c interface. Supports only read < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `len` - number of bytes to read.
    pub fn read(&mut self, addr: u8, len: usize) {
        self.dev.i2c.cr2.modify(|_, w| {
            w.sadd()
                .bits(u16::from(addr << 1))
                .rd_wrn()
                .read()
                .nbytes()
                .bits(len as u8)
                .start()
                .start()
                .autoend()
                .automatic()
        });
        self.state = State::RxStart;
    }

    /// Write bytes through i2c. Supports only write and read < 256 bytes.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    /// * `bytes` - Bytes to send.
    /// * `recv_len` - Number of bytes to receive.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    pub fn write_read(
        &mut self,
        addr: u8,
        bytes: &'static [u8],
        recv_len: usize,
    ) -> Result<(), I2cError> {
        self.recv = Some((addr, recv_len));
        self._write(addr, false, bytes)
    }

    /// Send a stop.
    ///
    /// # Arguments
    /// * `addr` - Destination address.
    ///
    /// # Errors
    /// * `I2cError::DeviceBusy` if the device is already busy.
    pub fn stop(&mut self, addr: u8) {
        self.dev
            .i2c
            .cr2
            .modify(|_, w| w.sadd().bits(u16::from(addr << 1)).stop().stop());
    }

    /// This function must be called when there is an interruption on i2c device.
    /// It will compute the current state based on ISR register and execute work based on the state.
    pub fn interrupt(&mut self) {
        let isr_state = self.isr_state();
        match isr_state {
            Ok(State::TxReady) => {
                if let Some(buf) = self.tx_buf {
                    self.write_tx_buffer(buf[self.tx_ind]);
                    self.tx_ind += 1;
                } else {
                    self.last_error = Some(I2cError::TxBufferEmpty);
                }
            }
            Ok(State::RxReady) => {
                self.rx_buf[self.rx_ind] = self.dev.i2c.rxdr.read().rxdata().bits();
                self.rx_ind += 1;
            }
            Ok(State::TxStop) => {
                self.tx_ind = 0;
                self.current_write_addr = None;
            }
            Ok(State::TxComplete) => {
                self.tx_ind = 0;
                // When receiving Tx complete, we should read after
                // if not it should be a tx stop
                if let Some(recv) = self.recv {
                    self.read(recv.0, recv.1)
                } else {
                    self.last_error = Some(I2cError::TransferCompleteNoRead);
                }
                self.current_write_addr = None;
            }
            Ok(State::RxStop) => {
                self.rx_ind = 0;
            }
            Ok(State::TxNack) => {
                self.tx_ind = 0;
                self.last_error = Some(I2cError::Nack);
                self.current_write_addr = None;
            }
            Err(err) => {
                self.last_error = Some(err);
                self.current_write_addr = None;
            }
            _ => {}
        }
    }

    /// Computes the states based on IRS register.
    fn isr_state(&mut self) -> Result<State, I2cError> {
        let isr = self.dev.i2c.isr.read();
        if isr.berr().bit() {
            self.dev.i2c.icr.write(|w| w.berrcf().bit(true));
            return Err(I2cError::BusError);
        } else if isr.arlo().bit() {
            self.dev.i2c.icr.write(|w| w.arlocf().bit(true));
            return Err(I2cError::ArbitrationLoss);
        } else if isr.ovr().bit() {
            self.dev.i2c.icr.write(|w| w.ovrcf().bit(true));
            return Err(I2cError::Overrun);
        }
        self.state = if isr.nackf().bit() {
            self.dev.i2c.icr.write(|w| w.nackcf().bit(true));
            State::TxNack
        } else if isr.tc().bit() {
            State::TxComplete
        } else if isr.txis().bit() && isr.txe().bit() {
            State::TxReady
        } else if isr.rxne().bit() {
            State::RxReady
        } else if isr.stopf().bit() {
            // clear stop bit once read
            self.dev.i2c.icr.write(|w| w.stopcf().bit(true));
            match self.state {
                State::TxSent | State::TxComplete => State::TxStop,
                State::RxReady => State::RxStop,
                _ => return Err(I2cError::StateError),
            }
        }
        else {
            return Err(I2cError::StateError)
        };
        Ok(self.state)
    }

    /// Write a bute into the tx buffer.
    fn write_tx_buffer(&mut self, byte: u8) {
        self.dev.i2c.txdr.write(|w| w.txdata().bits(byte));
        self.state = State::TxSent;
    }

    /// Is the device Busy ?
    /// # Returns
    /// true if busy, false if not.
    pub fn is_busy(&mut self) -> bool {
        self.dev.i2c.isr.read().busy().bit()
    }

    /// Get the state of the device.
    pub fn get_tx_state(&self) -> State {
        self.state
    }

    /// Get the content of the receive buffer.
    ///
    /// # Arguments
    /// * `len` - len of the slice to get.
    ///
    /// # Returns
    /// A slice containing of the rx buffer.
    pub fn get_rx_buf(&self, len: usize) -> &[u8] {
        self.rx_buf.split_at(len).0
    }

    /// Set the device state to idle.
    pub fn reset_state(&mut self) {
        self.state = State::Idle;
        self.last_error = None;
        self.tx_buf = None;
        self.recv = None;
    }

    /// Debug function which returns the content of the ISR register.
    pub fn get_isr(&self) -> u32 {
        self.dev.i2c.isr.read().bits()
    }
}
