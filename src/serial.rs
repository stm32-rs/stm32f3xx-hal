//! # Serial
//!
//! Asynchronous serial communication using the interal USART peripherals
//!
//! The serial modules implement the [`Read`] and [`Write`] traits.
//!
//! [`Read`]: embedded_hal::serial::Read
//! [`Write`]: embedded_hal::serial::Write

use core::{convert::Infallible, ops::Deref};

use crate::{
    gpio::{gpioa, gpiob, gpioc, AF7},
    hal::{blocking, serial},
    pac::{
        self,
        rcc::cfgr3::USART1SW_A,
        usart1::{cr1::M_A, cr1::PCE_A, cr1::PS_A, RegisterBlock},
        USART1, USART2, USART3,
    },
    rcc::{Clocks, APB1, APB2},
    time::rate::*,
};

#[allow(unused_imports)]
use crate::pac::RCC;

use cfg_if::cfg_if;

use crate::dma;
use cortex_m::interrupt;

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Transmission complete
    Tc,
    /// Idle line state detected
    Idle,
}

/// Serial error
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

/// TX pin
pub trait TxPin<Usart>: crate::private::Sealed {}

/// RX pin
pub trait RxPin<Usart>: crate::private::Sealed {}

impl<Otype> TxPin<USART1> for gpioa::PA9<AF7<Otype>> {}
impl<Otype> TxPin<USART1> for gpiob::PB6<AF7<Otype>> {}
impl<Otype> TxPin<USART1> for gpioc::PC4<AF7<Otype>> {}
impl<Otype> RxPin<USART1> for gpioa::PA10<AF7<Otype>> {}
impl<Otype> RxPin<USART1> for gpiob::PB7<AF7<Otype>> {}
impl<Otype> RxPin<USART1> for gpioc::PC5<AF7<Otype>> {}

impl<Otype> TxPin<USART2> for gpioa::PA2<AF7<Otype>> {}
impl<Otype> TxPin<USART2> for gpiob::PB3<AF7<Otype>> {}
impl<Otype> RxPin<USART2> for gpioa::PA3<AF7<Otype>> {}
impl<Otype> RxPin<USART2> for gpiob::PB4<AF7<Otype>> {}

impl<Otype> TxPin<USART3> for gpiob::PB10<AF7<Otype>> {}
impl<Otype> TxPin<USART3> for gpioc::PC10<AF7<Otype>> {}
impl<Otype> RxPin<USART3> for gpioc::PC11<AF7<Otype>> {}

cfg_if! {
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e", feature = "gpio-f373"))] {
        use crate::gpio::{gpiod, gpioe};

        impl<Otype> TxPin<USART1> for gpioe::PE0<AF7<Otype>> {}
        impl<Otype> RxPin<USART1> for gpioe::PE1<AF7<Otype>> {}

        impl<Otype> TxPin<USART2> for gpiod::PD5<AF7<Otype>> {}
        impl<Otype> RxPin<USART2> for gpiod::PD6<AF7<Otype>> {}

        impl<Otype> TxPin<USART3> for gpiod::PD8<AF7<Otype>> {}
        impl<Otype> RxPin<USART3> for gpiod::PD9<AF7<Otype>> {}
        impl<Otype> RxPin<USART3> for gpioe::PE15<AF7<Otype>> {}
    }
}

cfg_if! {
    if #[cfg(not(feature = "gpio-f373"))] {
        impl<Otype> TxPin<USART2> for gpioa::PA14<AF7<Otype>> {}
        impl<Otype> RxPin<USART2> for gpioa::PA15<AF7<Otype>> {}

        impl<Otype> RxPin<USART3> for gpiob::PB11<AF7<Otype>> {}
    }
}

cfg_if! {
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e",))] {
        use crate::pac::{UART4, UART5};
        use crate::gpio::AF5;

        impl<Otype> TxPin<UART4> for gpioc::PC10<AF5<Otype>> {}
        impl<Otype> RxPin<UART4> for gpioc::PC11<AF5<Otype>> {}
        impl<Otype> TxPin<UART5> for gpioc::PC12<AF5<Otype>> {}
        impl<Otype> RxPin<UART5> for gpiod::PD2<AF5<Otype>> {}
    }
}

/// Types for configuring a serial interface.
pub mod config {
    use crate::time::rate::{Baud, Extensions};

    // Reexport stop bit enum from PAC. In case there is a breaking change,
    // provide a compatible enum from this HAL.
    pub use crate::pac::usart1::cr2::STOP_A as StopBits;

    /// Parity generation and checking. If odd or even parity is selected, the
    /// underlying USART will be configured to send/receive the parity bit in
    /// addtion to the data bits.
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Clone, Copy, PartialEq)]
    pub enum Parity {
        /// No parity bit will be added/checked.
        None,
        /// The MSB transmitted/received will be generated/checked to have a
        /// even number of bits set.
        Even,
        /// The MSB transmitted/received will be generated/checked to have a
        /// odd number of bits set.
        Odd,
    }

    /// Configuration struct for [`Serial`](super::Serial) providing all
    /// communication-related / parameters. `Serial` always uses eight data
    /// bits plus the parity bit - if selected.
    ///
    /// Create a configuration by using `default` in combination with the
    /// builder methods. The following snippet shows creating a configuration
    /// for 19,200 Baud, 8N1 by deriving it from the default value:
    /// ```
    /// # use stm32f3xx_hal::serial::config::*;
    /// # use stm32f3xx_hal::time::rate::{Baud, Extensions};
    /// let config = Config::default().baudrate(19_200.Bd());
    ///
    /// assert!(config.baudrate == 19_200.Bd());
    /// assert!(config.parity == Parity::None);
    /// assert!(config.stopbits == StopBits::STOP1);
    /// ```
    #[derive(Clone, Copy, PartialEq)]
    #[non_exhaustive]
    pub struct Config {
        /// Serial interface baud rate
        pub baudrate: Baud,
        /// Whether and how to generate/check a parity bit
        pub parity: Parity,
        /// The number of stop bits to follow the last data bit or the parity
        /// bit
        pub stopbits: StopBits,
    }

    impl Config {
        /// Sets the given baudrate.
        pub fn baudrate(mut self, baudrate: impl Into<Baud>) -> Self {
            self.baudrate = baudrate.into();
            self
        }

        /// Sets the given parity.
        pub fn parity(mut self, parity: Parity) -> Self {
            self.parity = parity;
            self
        }

        /// Sets the stop bits to `stopbits`.
        pub fn stopbits(mut self, stopbits: StopBits) -> Self {
            self.stopbits = stopbits;
            self
        }
    }

    impl Default for Config {
        /// Creates a new configuration with typically used parameters: 115,200
        /// Baud 8N1.
        fn default() -> Config {
            Config {
                baudrate: 115_200.Bd(),
                parity: Parity::None,
                stopbits: StopBits::STOP1,
            }
        }
    }

    impl<T: Into<Baud>> From<T> for Config {
        fn from(b: T) -> Config {
            Config {
                baudrate: b.into(),
                ..Default::default()
            }
        }
    }
}

/// Serial abstraction
pub struct Serial<Usart, Pins> {
    usart: Usart,
    pins: Pins,
}

mod split {
    use super::Instance;
    /// Serial receiver
    pub struct Rx<Usart, Pin> {
        usart: Usart,
        pub(crate) pin: Pin,
    }

    /// Serial transmitter
    pub struct Tx<Usart, Pin> {
        usart: Usart,
        pub(crate) pin: Pin,
    }

    impl<Usart, Pin> Tx<Usart, Pin>
    where
        Usart: Instance,
        Pin: super::TxPin<Usart>,
    {
        pub(crate) fn new(usart: Usart, pin: Pin) -> Self {
            Tx { usart, pin }
        }

        /// Get a reference to internal usart peripheral
        ///
        /// # Safety
        ///
        /// This is unsafe, because the creation of this struct
        /// is only possible by splitting the the USART peripheral
        /// into Tx and Rx, which are internally both pointing
        /// to the same peripheral.
        ///
        /// Therefor, if getting a mutuable reference to the peripheral
        /// or changing any of it's configuration, the exclusivity
        /// is no longer guaranteed by the type system.
        ///
        /// Ensure that the Tx and Rx implemtation only do things with
        /// the peripheral, which do not effect the other.
        pub(crate) unsafe fn usart(&self) -> &Usart {
            &self.usart
        }

        /// Destruct [`Tx`] to regain access to underlying USART and pin.
        pub(crate) fn free(self) -> (Usart, Pin) {
            (self.usart, self.pin)
        }
    }

    impl<Usart, Pin> Rx<Usart, Pin>
    where
        Usart: Instance,
        Pin: super::RxPin<Usart>,
    {
        pub(crate) fn new(usart: Usart, pin: Pin) -> Self {
            Rx { usart, pin }
        }

        /// Get a reference to internal usart peripheral
        ///
        /// # Safety
        ///
        /// This is unsafe, because the creation of this struct
        /// is only possible by splitting the the USART peripheral
        /// into Tx and Rx, which are internally both pointing
        /// to the same peripheral.
        ///
        /// Therefor, if getting a mutuable reference to the peripheral
        /// or changing any of it's configuration, the exclusivity
        /// is no longer guaranteed by the type system.
        ///
        /// Ensure that the Tx and Rx implemtation only do things with
        /// the peripheral, which do not effect the other.
        pub(crate) unsafe fn usart(&self) -> &Usart {
            &self.usart
        }

        /// Destruct [`Rx`] to regain access to the underlying pin.
        ///
        /// The USART is omitted, as it is returnend from Tx already to avoid
        /// beeing able to crate a duplicate reference to the same underlying
        /// peripheral.
        pub(crate) fn free(self) -> Pin {
            self.pin
        }
    }
}

pub use split::{Rx, Tx};

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance,
{
    /// Configures a USART peripheral to provide serial communication
    pub fn new<Config>(
        usart: Usart,
        pins: (Tx, Rx),
        config: Config,
        clocks: Clocks,
        apb: &mut <Usart as Instance>::APB,
    ) -> Self
    where
        Usart: Instance,
        Tx: TxPin<Usart>,
        Rx: RxPin<Usart>,
        Config: Into<config::Config>,
    {
        use self::config::*;

        let config = config.into();

        // Enable USART peripheral for any further interaction.
        Usart::enable_clock(apb);
        // Disable USART because some configuration bits could only be written
        // in this state.
        usart.cr1.modify(|_, w| w.ue().disabled());

        let brr = Usart::clock(&clocks).integer() / config.baudrate.integer();
        crate::assert!(brr >= 16, "impossible baud rate");
        usart.brr.write(|w| w.brr().bits(brr as u16));

        // We currently support only eight data bits as supporting a full-blown
        // configuration gets complicated pretty fast. The USART counts data
        // and partiy bits together so the actual amount depends on the parity
        // selection.
        let (m0, ps, pce) = match config.parity {
            Parity::None => (M_A::BIT8, PS_A::EVEN, PCE_A::DISABLED),
            Parity::Even => (M_A::BIT9, PS_A::EVEN, PCE_A::ENABLED),
            Parity::Odd => (M_A::BIT9, PS_A::ODD, PCE_A::ENABLED),
        };

        usart.cr2.modify(|_, w| w.stop().variant(config.stopbits));
        usart.cr1.modify(|_, w| {
            w.ps().variant(ps); // set parity mode
            w.pce().variant(pce); // enable parity checking/generation
            w.m().variant(m0); // set data bits
            w.re().enabled(); // enable receiver
            w.te().enabled() // enable transmitter
        });

        // Finally enable the configured UART.
        usart.cr1.modify(|_, w| w.ue().enabled());

        Self { usart, pins }
    }

    /// Starts listening for an interrupt event
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().enabled()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().enabled()),
            Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().enabled()),
            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().enabled()),
        }
    }

    /// Stops listening for an interrupt event
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().disabled()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().disabled()),
            Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().disabled()),
            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().disabled()),
        }
    }

    /// Return true if the tx register is empty (and can accept data)
    pub fn is_txe(&self) -> bool {
        self.usart.isr.read().txe().bit_is_set()
    }

    /// Return true if the rx register is not empty (and can be read)
    pub fn is_rxne(&self) -> bool {
        self.usart.isr.read().rxne().bit_is_set()
    }

    /// Return true if the transmission is complete
    pub fn is_tc(&self) -> bool {
        self.usart.isr.read().tc().bit_is_set()
    }

    /// Return true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        self.usart.isr.read().tc().bit_is_set()
    }

    /// Releases the USART peripheral and associated pins
    pub fn free(self) -> (Usart, (Tx, Rx)) {
        self.usart
            .cr1
            .modify(|_, w| w.ue().disabled().re().disabled().te().disabled());
        (self.usart, self.pins)
    }

    /// Joins previously [`Serial::split()`] serial.
    ///
    /// This is often needed to access methods only implemented for [`Serial`]
    /// but not for [`Tx`] nor [`Rx`].
    ///
    /// # Example
    ///
    /// ```
    /// let dp = pac::Peripherals::take().unwrap();
    ///
    /// (tx, rx) = Serial::new(dp.USART1, ...).split();
    ///
    /// // Do something with tx and rx
    ///
    /// serial = Serial::join(tx, rx);
    /// ```
    pub fn join(tx: split::Tx<Usart, Tx>, rx: split::Rx<Usart, Rx>) -> Self
    where
        Tx: TxPin<Usart>,
        Rx: RxPin<Usart>,
    {
        let (usart, tx_pin) = tx.free();
        let rx_pin = rx.free();
        Self {
            usart,
            pins: (tx_pin, rx_pin),
        }
    }
}

// TODO: Check if u16 for WORD is feasiable / possible
impl<Usart, Tx, Rx> serial::Read<u8> for Serial<Usart, (Tx, Rx)>
where
    Usart: Instance,
{
    type Error = Error;
    fn read(&mut self) -> nb::Result<u8, Error> {
        let isr = self.usart.isr.read();

        Err(if isr.pe().bit_is_set() {
            self.usart.icr.write(|w| w.pecf().clear());
            nb::Error::Other(Error::Parity)
        } else if isr.fe().bit_is_set() {
            self.usart.icr.write(|w| w.fecf().clear());
            nb::Error::Other(Error::Framing)
        } else if isr.nf().bit_is_set() {
            self.usart.icr.write(|w| w.ncf().clear());
            nb::Error::Other(Error::Noise)
        } else if isr.ore().bit_is_set() {
            self.usart.icr.write(|w| w.orecf().clear());
            nb::Error::Other(Error::Overrun)
        } else if isr.rxne().bit_is_set() {
            return Ok(self.usart.rdr.read().bits() as u8);
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<Usart, Pin> serial::Read<u8> for Rx<Usart, Pin>
where
    Usart: Instance,
    Pin: RxPin<Usart>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { self.usart().isr.read() };

        // NOTE(unsafe, write) write accessor for atomic writes with no side effects
        let icr = unsafe { &self.usart().icr };
        Err(if isr.pe().bit_is_set() {
            icr.write(|w| w.pecf().clear());
            nb::Error::Other(Error::Parity)
        } else if isr.fe().bit_is_set() {
            icr.write(|w| w.fecf().clear());
            nb::Error::Other(Error::Framing)
        } else if isr.nf().bit_is_set() {
            icr.write(|w| w.ncf().clear());
            nb::Error::Other(Error::Noise)
        } else if isr.ore().bit_is_set() {
            icr.write(|w| w.orecf().clear());
            nb::Error::Other(Error::Overrun)
        } else if isr.rxne().bit_is_set() {
            // NOTE(unsafe) atomic read with no side effects
            return Ok(unsafe { self.usart().rdr.read().bits() as u8 });
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl<Usart, Tx, Rx> serial::Write<u8> for Serial<Usart, (Tx, Rx)>
where
    Usart: Instance,
{
    // NOTE(Infallible) See section "29.7 USART interrupts"; the only possible errors during
    // transmission are: clear to send (which is disabled in this case) errors and
    // framing errors (which only occur in SmartCard mode); neither of these apply to
    // our hardware configuration
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Infallible> {
        if self.usart.isr.read().tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        if self.usart.isr.read().txe().bit_is_set() {
            self.usart.tdr.write(|w| w.tdr().bits(u16::from(byte)));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<USART, TX, RX> blocking::serial::write::Default<u8> for Serial<USART, (TX, RX)> where
    USART: Instance
{
}

impl<Usart, Pin> serial::Write<u8> for Tx<Usart, Pin>
where
    Usart: Instance,
    Pin: TxPin<Usart>,
{
    // NOTE(Infallible) See section "29.7 USART interrupts"; the only possible errors during
    // transmission are: clear to send (which is disabled in this case) errors and
    // framing errors (which only occur in SmartCard mode); neither of these apply to
    // our hardware configuration
    type Error = Infallible;

    fn flush(&mut self) -> nb::Result<(), Infallible> {
        let isr = unsafe { self.usart().isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { self.usart().isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            unsafe { self.usart().tdr.write(|w| w.tdr().bits(u16::from(byte))) };
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<Usart, Pin> Rx<Usart, Pin>
where
    Usart: Instance + Dma,
    Pin: RxPin<Usart>,
{
    /// Fill the buffer with received data using DMA.
    pub fn read_exact<B, C>(self, buffer: B, mut channel: C) -> dma::Transfer<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::WriteBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // NOTE(unsafe) usage of a valid peripheral address
        unsafe {
            channel.set_peripheral_address(
                &self.usart().rdr as *const _ as u32,
                dma::Increment::Disable,
            )
        };

        dma::Transfer::start_write(buffer, channel, self)
    }
}

impl<Usart, Pin> blocking::serial::write::Default<u8> for Tx<Usart, Pin>
where
    Usart: Instance,
    Pin: TxPin<Usart>,
{
}

impl<Usart, Pin> Tx<Usart, Pin>
where
    Usart: Instance + Dma,
    Pin: TxPin<Usart>,
{
    /// Transmit all data in the buffer using DMA.
    pub fn write_all<B, C>(self, buffer: B, mut channel: C) -> dma::Transfer<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::ReadBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // NOTE(unsafe) usage of a valid peripheral address
        unsafe {
            channel.set_peripheral_address(
                &self.usart().tdr as *const _ as u32,
                dma::Increment::Disable,
            )
        };

        dma::Transfer::start_read(buffer, channel, self)
    }
}

impl<Usart, Pin> dma::Target for Rx<Usart, Pin>
where
    Usart: Instance + Dma,
    Pin: RxPin<Usart>,
{
    fn enable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmar().enabled());
        });
    }

    fn disable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmar().disabled());
        });
    }
}

impl<Usart, Pin> dma::Target for Tx<Usart, Pin>
where
    Usart: Instance + Dma,
    Pin: TxPin<Usart>,
{
    fn enable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmat().enabled());
        });
    }

    fn disable_dma(&mut self) {
        // NOTE(unsafe) critical section prevents races
        interrupt::free(|_| unsafe {
            self.usart().cr3.modify(|_, w| w.dmat().disabled());
        });
    }
}

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance + Dma,
    Rx: RxPin<Usart>,
    Tx: TxPin<Usart>,
{
    /// Fill the buffer with received data using DMA.
    pub fn read_exact<B, C>(self, buffer: B, mut channel: C) -> dma::Transfer<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::WriteBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // NOTE(unsafe) usage of a valid peripheral address
        unsafe {
            channel
                .set_peripheral_address(&self.usart.rdr as *const _ as u32, dma::Increment::Disable)
        };

        dma::Transfer::start_write(buffer, channel, self)
    }

    /// Transmit all data in the buffer using DMA.
    pub fn write_all<B, C>(self, buffer: B, mut channel: C) -> dma::Transfer<B, C, Self>
    where
        Self: dma::OnChannel<C>,
        B: dma::ReadBuffer<Word = u8> + 'static,
        C: dma::Channel,
    {
        // NOTE(unsafe) usage of a valid peripheral address
        unsafe {
            channel
                .set_peripheral_address(&self.usart.tdr as *const _ as u32, dma::Increment::Disable)
        };

        dma::Transfer::start_read(buffer, channel, self)
    }
}

impl<Usart, Tx, Rx> dma::Target for Serial<Usart, (Tx, Rx)>
where
    Usart: Instance + Dma,
{
    fn enable_dma(&mut self) {
        self.usart
            .cr3
            .modify(|_, w| w.dmar().enabled().dmat().enabled())
    }

    fn disable_dma(&mut self) {
        self.usart
            .cr3
            .modify(|_, w| w.dmar().disabled().dmat().disabled())
    }
}

/// Marker trait for DMA capable UART implementations.
pub trait Dma: crate::private::Sealed {}

impl Dma for USART1 {}
impl Dma for USART2 {}
impl Dma for USART3 {}

/// UART instance
pub trait Instance: Deref<Target = RegisterBlock> + crate::private::Sealed {
    /// Peripheral bus instance which is responsible for the peripheral
    type APB;
    #[doc(hidden)]
    fn enable_clock(apb1: &mut Self::APB);
    #[doc(hidden)]
    fn clock(clocks: &Clocks) -> Hertz;
}

macro_rules! usart {
    (
        $(
            $USARTX:ident: (
                $usartXen:ident,
                $APB:ident,
                $pclkX:ident,
                $usartXrst:ident,
                $usartXsw:ident,
                $usartXclock:ident
            ),
        )+
    ) => {
        $(
            impl crate::private::Sealed for $USARTX {}
            impl Instance for $USARTX {
                type APB = $APB;
                fn enable_clock(apb: &mut Self::APB) {
                    apb.enr().modify(|_, w| w.$usartXen().enabled());
                    apb.rstr().modify(|_, w| w.$usartXrst().reset());
                    apb.rstr().modify(|_, w| w.$usartXrst().clear_bit());
                }

                fn clock(clocks: &Clocks) -> Hertz {
                    // Use the function created via another macro outside of this one,
                    // because the implementation is dependend on the type $USARTX.
                    // But macros can not differentiate between types.
                    $usartXclock(clocks)
                }
            }


            impl<Tx, Rx> Serial<$USARTX, (Tx, Rx)>
                where Tx: TxPin<$USARTX>, Rx: RxPin<$USARTX> {
                /// Splits the [`Serial`] abstraction into a transmitter and a receiver half.
                ///
                /// This allows using [`Tx`] and [`Rx`] related actions to
                /// be handled independently and even use these safely in different
                /// contexts (like interrupt routines) without needing to do synchronization work
                /// between them.
                pub fn split(self) -> (split::Tx<$USARTX, Tx>, split::Rx<$USARTX, Rx>) {
                    // NOTE(unsafe): This essentially duplicates the USART peripheral
                    //
                    // As RX and TX both do have direct access to the peripheral,
                    // they must guarantee to only do atomic operations on the peripheral
                    // registers to avoid data races.
                    //
                    // Tx and Rx won't access the same registers anyways,
                    // as they have independent responsibilities, which are NOT represented
                    // in the type system.
                    let (tx, rx) = unsafe {
                        (
                            pac::Peripherals::steal().$USARTX,
                            pac::Peripherals::steal().$USARTX,
                        )
                    };
                    (split::Tx::new(tx, self.pins.0), split::Rx::new(rx, self.pins.1))
                }
            }
        )+
    };

    ([ $(($X:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            usart!(
                $(
                    [<USART $X>]: (
                        [<usart $X en>],
                        [<APB $APB>],
                        [<pclk $APB>],
                        [<usart $X rst>],
                        [<usart $X sw>],
                        [<usart $X clock>]
                    ),
                )+
            );
        }
    };
}

/// Generates a clock function for UART Peripherals, where
/// the only clock source can be the peripheral clock
#[allow(unused_macros)]
macro_rules! usart_static_clock {
    ($($usartXclock:ident, $pclkX:ident),+) => {
        $(
            /// Return the currently set source frequency the UART peripheral
            /// depending on the clock source.
            fn $usartXclock(clocks: &Clocks) -> Hertz {
                clocks.$pclkX()
            }
        )+
    };
    ([ $(($X:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            usart_static_clock!(
                $([<usart $X clock>], [<pclk $APB>]),+
            );
        }
    };
}

/// Generates a clock function for UART Peripherals, where
/// the clock source can vary.
macro_rules! usart_var_clock {
    ($($usartXclock:ident, $usartXsw:ident, $pclkX:ident),+) => {
        $(
            /// Return the currently set source frequency the UART peripheral
            /// depending on the clock source.
            fn $usartXclock(clocks: &Clocks) -> Hertz {
                // NOTE(unsafe): atomic read with no side effects
                match unsafe {(*RCC::ptr()).cfgr3.read().$usartXsw().variant()} {
                    USART1SW_A::PCLK => clocks.$pclkX(),
                    USART1SW_A::HSI => crate::rcc::HSI,
                    USART1SW_A::SYSCLK => clocks.sysclk(),
                    USART1SW_A::LSE => crate::rcc::LSE,
                }
            }
        )+
    };
    ([ $(($X:literal, $APB:literal)),+ ]) => {
        paste::paste! {
            usart_var_clock!(
                $([<usart $X clock>], [<usart $X sw>], [<pclk $APB>]),+
            );
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "svd-f301", feature = "svd-f3x4"))] {
        usart_var_clock!([(1,2)]);
        // These are uart peripherals, where the only clock source
        // is the PCLK (peripheral clock).
        usart_static_clock!([(2,1), (3,1)]);
    } else {
        usart_var_clock!([(1, 2), (2, 1), (3, 1)]);
    }
}
usart!([(1, 2), (2, 1), (3, 1)]);

cfg_if::cfg_if! {
    // See table 29.4 RM0316
    if #[cfg(any(feature = "gpio-f303", feature = "gpio-f303e"))] {

        macro_rules! uart {
            ([ $(($X:literal, $APB:literal)),+ ]) => {
                paste::paste! {
                    usart!(
                        $(
                            [<UART $X>]: (
                                [<uart $X en>],
                                [<APB $APB>],
                                [<pclk $APB>],
                                [<uart $X rst>],
                                [<uart $X sw>],
                                [<usart $X clock>]
                            ),
                        )+
                    );
                }
            };
        }

        macro_rules! uart_var_clock {
            ([ $(($X:literal, $APB:literal)),+ ]) => {
                paste::paste! {
                    usart_var_clock!(
                        $([<usart $X clock>], [<uart $X sw>], [<pclk $APB>]),+
                    );
                }
            };
        }

        uart_var_clock!([(4,1), (5,1)]);
        uart!([(4,1), (5,1)]);

        impl Dma for UART4 {}
    }
}
