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
#[cfg(feature = "enumset")]
use enumset::{EnumSet, EnumSetType};

use crate::dma;
use cortex_m::interrupt;

/// Interrupt and status events.
///
/// All events can be cleared by [`Serial::clear_event`] or [`Serial::clear_events`].
/// Some events are also cleared on other conditions.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "enumset", derive(EnumSetType))]
#[cfg_attr(not(feature = "enumset"), derive(Copy, Clone, PartialEq, Eq))]
#[non_exhaustive]
pub enum Event {
    /// Transmit data register empty / new data can be sent.
    ///
    /// This event is set by hardware when the content of the TDR register has been transferred
    /// into the shift register. It is cleared by [`Serial`]s [`serial::Write::write()`]
    /// implementation to the TDR register.
    #[doc(alias = "TXE")]
    TransmitDataRegisterEmtpy,
    /// CTS (Clear to Send) event.
    ///
    /// This event is set by hardware when the CTS input toggles, if the CTSE bit is set.
    #[doc(alias = "CTSIF")]
    CtsInterrupt,
    /// Transmission complete
    ///
    /// This event is set by hardware if the transmission of a frame containing data is complete and
    /// if TXE is set.
    /// It is cleared by [`Serial`]s [`serial::Write::write()`] implementaiton to the USART_TDR register.
    #[doc(alias = "TC")]
    TransmissionComplete,
    /// Read data register not empty / new data has been received.
    ///
    /// This event is set by hardware when the content of the RDR shift register has been
    /// transferred to the RDR register.
    /// It is cleared by [`Serial`]s [`serial::Read::read()`] to the USART_RDR register.
    #[doc(alias = "RXNE")]
    ReceiveDataRegisterNotEmpty,
    /// Overrun Error detected.
    ///
    /// This event is set by hardware when the data currently being received in the shift register
    /// is ready to be transferred into the RDR register while
    /// [`Event::ReceiveDataRegisterNotEmpty`] is set.
    ///
    /// See [`Error::Overrun`] for a more detailed description.
    #[doc(alias = "ORE")]
    OverrunError,
    /// Idle line state detected.
    ///
    /// This event is set by hardware when an Idle Line is detected.
    Idle,
    /// Parity error detected.
    ///
    /// This event is set by hardware when a parity error occurs in receiver mode.
    ///
    /// Parity can be configured by using [`config::Parity`] to create a [`config::Config`].
    #[doc(alias = "PE")]
    ParityError,
    /// Noise error detected.
    ///
    /// This event is set by hardware when noise is detected on a received frame.
    #[doc(alias = "NF")]
    NoiseError,
    /// Framing error detected
    ///
    /// This event is set by hardware when a de-synchronization, excessive noise or a break character
    /// is detected.
    #[doc(alias = "FE")]
    FramingError,
    /// LIN break
    ///
    /// This bit is set by hardware when the LIN break is detected.
    #[doc(alias = "LBDF")]
    LinBreak,
    /// The received character matched the configured character.
    ///
    /// The matching character can be configured with [`Serial::match_character()`]
    #[doc(alias = "CMF")]
    CharacterMatch,
    /// Nothing was received since the last received character for
    /// [`Serial::receiver_timeout()`] amount of time.
    ///
    /// # Note
    ///
    /// Never set for UART peripheral, which does not have [`ReceiverTimeoutExt`]
    /// implemented.
    #[doc(alias = "RTOF")]
    ReceiverTimeout,
    // TODO: SmartCard Mode not implemented, no use as of now.
    // EndOfBlock,
    // TODO(Sh3Rm4n): The wakeup from stop mode is alittle bit more complicated:
    // - UESM has to be enabled so that it works (RM0316 29.8.1)
    // - Only works with LSI and HSI (which are not configurable yet)
    // - ...
    // /// The peripheral was woken up from "Stop Mode".
    // ///
    // /// This event is set by hardware, when a wakeup event is detected.
    // ///
    // /// The condition, when it does wake up can be configured via
    // /// [`Serial::set_wakeup_from_stopmode_reason()`]
    // #[doc(alias = "WUF")]
    // WakeupFromStopMode,
}

/// Serial error
///
/// As these are status events, they can be converted to [`Event`]s, via [`Into`].
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    ///
    /// This error is thrown by hardware when a de-synchronization, excessive noise or a break
    /// character is detected.
    Framing,
    /// Noise error
    ///
    /// This error is thrown by hardware when noise is detected on a received frame.
    Noise,
    /// RX buffer overrun
    ///
    /// # Cause
    ///
    /// An overrun error occurs when a character is received when RXNE has not been reset. Data can
    /// not be transferred from the shift register to the RDR register until the RXNE bit is
    /// cleared. The RXNE flag is set after every byte received. An overrun error occurs if RXNE
    /// flag is set when the next data is received or the previous DMA request has not been
    /// serviced.
    ///
    /// # Behavior
    ///
    /// - The RDR content will not be lost. The previous data is available when a read to USART_RDR
    ///   is performed.
    /// - The shift register will be overwritten. After that point, any data received
    ///   during overrun is lost
    Overrun,
    /// Parity check error
    ///
    /// This error is thrown by hardware when a parity error occurs in receiver mode.
    Parity,
}

impl From<Error> for Event {
    fn from(error: Error) -> Self {
        match error {
            Error::Framing => Event::FramingError,
            Error::Overrun => Event::OverrunError,
            Error::Noise => Event::NoiseError,
            Error::Parity => Event::ParityError,
        }
    }
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
///
/// This is an abstraction of the UART peripheral intended to be
/// used for standard duplex serial communication.
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

    /// Enable or disable the interrupt for the specified [`Event`].
    #[inline]
    pub fn configure_interrupt(&mut self, event: Event, enable: bool) -> &mut Self {
        match event {
            Event::TransmitDataRegisterEmtpy => self.usart.cr1.modify(|_, w| w.txeie().bit(enable)),
            Event::CtsInterrupt => self.usart.cr3.modify(|_, w| w.ctsie().bit(enable)),
            Event::TransmissionComplete => self.usart.cr1.modify(|_, w| w.tcie().bit(enable)),
            Event::ReceiveDataRegisterNotEmpty => {
                self.usart.cr1.modify(|_, w| w.rxneie().bit(enable))
            }
            Event::ParityError => self.usart.cr1.modify(|_, w| w.peie().bit(enable)),
            Event::LinBreak => self.usart.cr2.modify(|_, w| w.lbdie().bit(enable)),
            Event::NoiseError | Event::OverrunError | Event::FramingError => {
                self.usart.cr3.modify(|_, w| w.eie().bit(enable))
            }
            Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().bit(enable)),
            Event::CharacterMatch => self.usart.cr1.modify(|_, w| w.cmie().bit(enable)),
            Event::ReceiverTimeout => self.usart.cr1.modify(|_, w| w.rtoie().bit(enable)),
            // Event::EndOfBlock => self.usart.cr1.modify(|_, w| w.eobie().bit(enable)),
            // Event::WakeupFromStopMode => self.usart.cr3.modify(|_, w| w.wufie().bit(enable)),
        };
        self
    }

    /// Enable or disable interrupt for the specified [`Event`]s.
    ///
    /// Like [`Serial::configure_interrupt`], but instead using an enumset. The corresponding
    /// interrupt for every [`Event`] in the set will be enabled, every other interrupt will be
    /// **disabled**.
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn configure_interrupts(&mut self, events: EnumSet<Event>) -> &mut Self {
        for event in events.complement().iter() {
            self.configure_interrupt(event, false);
        }
        for event in events.iter() {
            self.configure_interrupt(event, true);
        }

        self
    }

    /// Get an [`EnumSet`] of all fired interrupt events.
    ///
    /// # Examples
    ///
    /// This allows disabling all fired event at once, via the enum set abstraction, like so
    ///
    /// ```rust
    /// for event in serial.events() {
    ///     serial.listen(event, false);
    /// }
    /// ```
    #[cfg(feature = "enumset")]
    #[cfg_attr(docsrs, doc(cfg(feature = "enumset")))]
    pub fn triggered_events(&self) -> EnumSet<Event> {
        let mut events = EnumSet::new();

        for event in EnumSet::<Event>::all().iter() {
            if self.is_event_triggered(event) {
                events |= event;
            }
        }

        events
    }

    /// Clear **all** interrupt events.
    #[inline]
    pub fn clear_events(&mut self) {
        // SAFETY: This atomic write clears all flags and ignores the reserverd bit fields.
        self.usart.icr.write(|w| unsafe { w.bits(u32::MAX) });
    }

    /// Clear the given interrupt event flag.
    #[inline]
    pub fn clear_event(&mut self, event: Event) {
        self.usart.icr.write(|w| match event {
            Event::CtsInterrupt => w.ctscf().clear(),
            Event::TransmissionComplete => w.tccf().clear(),
            Event::OverrunError => w.orecf().clear(),
            Event::Idle => w.idlecf().clear(),
            Event::ParityError => w.pecf().clear(),
            Event::LinBreak => w.lbdcf().clear(),
            Event::NoiseError => w.ncf().clear(),
            Event::FramingError => w.fecf().clear(),
            Event::CharacterMatch => w.cmcf().clear(),
            Event::ReceiverTimeout => w.rtocf().clear(),
            // Event::EndOfBlock => w.eobcf().clear(),
            // Event::WakeupFromStopMode => w.wucf().clear(),
            Event::ReceiveDataRegisterNotEmpty => {
                // Flush the register data queue, so that this even will not be thrown again.
                self.usart.rqr.write(|w| w.rxfrq().set_bit());
                w
            }
            // Do nothing with this event (only useful for Smartcard, which is not
            // supported right now)
            Event::TransmitDataRegisterEmtpy => w,
        });
    }

    /// Check if an interrupt event happend.
    #[inline]
    pub fn is_event_triggered(&self, event: Event) -> bool {
        let isr = self.usart.isr.read();
        match event {
            Event::TransmitDataRegisterEmtpy => isr.txe().bit(),
            Event::CtsInterrupt => isr.ctsif().bit(),
            Event::TransmissionComplete => isr.tc().bit(),
            Event::ReceiveDataRegisterNotEmpty => isr.rxne().bit(),
            Event::OverrunError => isr.ore().bit(),
            Event::Idle => isr.idle().bit(),
            Event::ParityError => isr.pe().bit(),
            Event::LinBreak => isr.lbdf().bit(),
            Event::NoiseError => isr.nf().bit(),
            Event::FramingError => isr.fe().bit(),
            Event::CharacterMatch => isr.cmf().bit(),
            Event::ReceiverTimeout => isr.rtof().bit(),
            // Event::EndOfBlock => isr.eobf().bit(),
            // Event::WakeupFromStopMode => isr.wuf().bit(),
        }
    }


    /// Configuring the UART to match each received character,
    /// with the configured one.
    ///
    /// If the character is matched [`Event::CharacterMatch`] is generated,
    /// which can fire an interrupt, if enabled via [`Serial::configure_interrupt()`]
    #[inline(always)]
    pub fn set_match_character(&mut self, char: u8) {
        // Note: This bit field can only be written when reception is disabled (RE = 0) or the
        // USART is disabled
        let enabled = self.usart.cr1.read().ue().bit_is_set();
        self.usart.cr1.modify(|_, w| w.ue().disabled());
        self.usart.cr2.modify(|_, w| w.add().bits(char));
        self.usart.cr1.modify(|_, w| w.ue().bit(enabled));
    }

    /// Read out the configured match character.
    #[inline(always)]
    pub fn match_character(&self) -> u8 {
        self.usart.cr2.read().add().bits()
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

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance + ReceiverTimeoutExt,
{
    /// Set the receiver timeout value.
    ///
    /// The RTOF flag ([`Event::ReceiverTimeout`]) is set if, after the last received character,
    /// no new start bit is detected for more than the receiver timeout value, where the value
    /// is being a counter, which is decreased by the configured baud rate.
    ///
    /// A simple calculation might be `time_per_counter_value = 1 / configured_baud_rate`
    ///
    ///
    /// ## Note
    ///
    /// - If the value is None, the receiver timeout feature is disabled.
    /// - This value must only be programmed once per received character.
    /// - Can be written on the fly. If the new value is lower than or equal to the counter,
    ///   the RTOF flag is set.
    /// - Values higher than 24 bits are truncated to 24 bit max (16_777_216).
    pub fn set_receiver_timeout(&mut self, value: Option<u32>) {
        if let Some(value) = value {
            self.usart.cr2.modify(|_, w| w.rtoen().enabled());
            self.usart.rtor.modify(|_, w| w.rto().bits(value))
        } else {
            self.usart.cr2.modify(|_, w| w.rtoen().disabled());
        }
    }

    /// Read out the currently set timeout value
    ///
    /// The relationship between the unit value and time is described in
    /// [`Serial::receiver_timeout`].
    ///
    /// - If the value is None, the receiver timeout feature is disabled.
    pub fn receiver_timeout(&self) -> Option<u32> {
        if self.usart.cr2.read().rtoen().is_enabled() {
            Some(self.usart.rtor.read().rto().bits())
        } else {
            None
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

/// Marker trait for Receiver Timeout capable UART implementations.
pub trait ReceiverTimeoutExt: crate::private::Sealed {}

impl ReceiverTimeoutExt for USART1 {}
#[cfg(not(any(feature = "gpio-f333")))]
impl ReceiverTimeoutExt for USART2 {}
#[cfg(not(any(feature = "gpio-f333")))]
impl ReceiverTimeoutExt for USART3 {}

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
    if #[cfg(any(
        feature = "stm32f301x6",
        feature = "stm32f301x8",
        feature = "stm32f318x8",
        feature = "stm32f302x6",
        feature = "stm32f302x8",
        feature = "stm32f303x6",
        feature = "stm32f303x8",
        feature = "stm32f328x8",
        feature = "stm32f334x4",
        feature = "stm32f334x6",
        feature = "stm32f334x8",
    ))] {
        // USART1 is accessed through APB2,
        // but USART1SW_A::PCLK will connect its phy to PCLK1.
        usart_var_clock!([(1,1)]);
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

        impl ReceiverTimeoutExt for UART4 {}
        impl ReceiverTimeoutExt for UART5 {}
    }
}
