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
    pac::{self, rcc::cfgr3::USART1SW_A, usart1::RegisterBlock, USART1, USART2, USART3},
    rcc::{Clocks, APB1, APB2},
    time::rate::*,
};

#[allow(unused_imports)]
use crate::pac::RCC;

use cfg_if::cfg_if;
use enumset::{EnumSet, EnumSetType};

use crate::dma;
use cortex_m::interrupt;

/// Interrupt and status events
// TODO: Sort as flags are ordered in register
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    /// New data can be sent
    TransmitDataRegisterEmtpy,
    /// CTS (Clear to Send) event
    CtsInterrupt,
    /// Transmission complete
    TransmissionComplete,
    /// New data has been received
    ReceiveDataRegisterNotEmpty,
    /// OverrunErrorDetected
    OverrunError,
    /// Idle line state detected
    Idle,
    /// Parity error detected
    ParityError,
    /// Noise error detected
    NoiseError,
    /// Framing error detected
    FramingError,
    /// LIN break
    LinBreak,
    /// The received character matched the configured character.
    ///
    /// The matching character can be configured with [`Serial::match_character()`]
    CharacterMatch,
    /// Nothing was received since the last received character for
    /// [`Serial::receiver_timeout()`] amount of time.
    ///
    /// # Note
    ///
    /// Never set for UART peripheral, which does not have [`ReceiverTimeoutFeature`]
    /// implemented.
    // TODO: Maybe exclude depending on feature set (which might be difficult,
    // as feature gating the value is not easy because it depends on the USART peripheral,
    // and the hardware does support the value, but just does not set it.
    // Maybe than configure the feature via trait of the is_event function?)
    ReceiverTimeout,
    // TODO: SmartCard Mode not implemented, no use as of now.
    // EndOfBlock,
    /// The peripheral was woken up from "Stop Mode".
    ///
    /// The condition, when it does wake up can be configured via
    /// [`Serial::wakeup_from_stopmode_reason()`]
    WakeupFromStopMode,
}

/// Serial error
///
/// As these are status events, they can be
/// converted to [`Event`]s.
#[derive(Copy, Clone, Debug)]
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

// TODO: If From is implemented, none_exhaustive on Error does not make sense?
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
    pub struct Rx<Usart> {
        usart: Usart,
    }

    /// Serial transmitter
    pub struct Tx<Usart> {
        usart: Usart,
    }

    impl<Usart> Tx<Usart>
    where
        Usart: Instance,
    {
        pub(crate) fn new(usart: Usart) -> Self {
            Tx { usart }
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
    }

    impl<Usart> Rx<Usart>
    where
        Usart: Instance,
    {
        pub(crate) fn new(usart: Usart) -> Self {
            Rx { usart }
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
    }
}

pub use split::{Rx, Tx};

type StopModeWakeup = pac::usart1::cr3::WUS_A;

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance,
{
    /// Configures a USART peripheral to provide serial communication
    pub fn new(
        usart: Usart,
        pins: (Tx, Rx),
        baud_rate: Baud,
        clocks: Clocks,
        apb: &mut <Usart as Instance>::APB,
    ) -> Self
    where
        Usart: Instance,
        Tx: TxPin<Usart>,
        Rx: RxPin<Usart>,
    {
        Usart::enable_clock(apb);

        let brr = Usart::clock(&clocks).integer() / baud_rate.integer();
        crate::assert!(brr >= 16, "impossible baud rate");
        usart.brr.write(|w| w.brr().bits(brr as u16));

        usart.cr1.modify(|_, w| {
            w.ue().enabled(); // enable USART
            w.re().enabled(); // enable receiver
            w.te().enabled() // enable transmitter
        });

        Self { usart, pins }
    }

    /// Enable or disable the interrupt for the specified [`Event`].
    // TODO: Provide enumset method
    // TODO: Rename listen, so that enumset has distiguishable name
    pub fn configure_interrupt(&mut self, event: impl Into<Event>, enable: bool) -> &mut Self {
        match event.into() {
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
            Event::WakeupFromStopMode => self.usart.cr3.modify(|_, w| w.wufie().bit(enable)),
        };
        self
    }

    /// Get an [`EnumSet`] of all fired intterupt events
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
    pub fn triggered_events(&self) -> EnumSet<Event> {
        let mut events = EnumSet::new();

        for event in EnumSet::<Event>::all().iter() {
            if self.is_event_triggered(event) {
                events |= event;
            }
        }

        events
    }

    pub fn clear_events(&mut self) {
        // SAFETY: This atomic write clears all flags and ignores the reserverd bit fields.
        self.usart.icr.write(|w| unsafe { w.bits(u32::MAX) });
    }

    /// Clear the interrupt event flag
    ///
    ///
    pub fn clear_event(&mut self, event: impl Into<Event>) {
        self.usart.icr.write(|w| match event.into() {
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
            Event::WakeupFromStopMode => w.wucf().clear(),
            Event::TransmitDataRegisterEmtpy | Event::ReceiveDataRegisterNotEmpty => w,
        });
    }

    /// Check if an interrupt event happend.
    pub fn is_event_triggered(&self, event: impl Into<Event>) -> bool {
        let isr = self.usart.isr.read();
        match event.into() {
            Event::TransmitDataRegisterEmtpy => isr.txe().bit_is_set(),
            Event::CtsInterrupt => isr.ctsif().bit_is_set(),
            Event::TransmissionComplete => isr.tc().bit_is_set(),
            Event::ReceiveDataRegisterNotEmpty => isr.rxne().bit_is_set(),
            Event::OverrunError => isr.ore().bit_is_set(),
            Event::Idle => isr.idle().bit_is_set(),
            Event::ParityError => isr.pe().bit_is_set(),
            Event::LinBreak => isr.lbdf().bit_is_set(),
            Event::NoiseError => isr.nf().bit_is_set(),
            Event::FramingError => isr.fe().bit_is_set(),
            Event::CharacterMatch => isr.cmf().bit_is_set(),
            Event::ReceiverTimeout => isr.rtof().bit_is_set(),
            // Event::EndOfBlock => isr.eobf().bit_is_set(),
            Event::WakeupFromStopMode => isr.wuf().bit_is_set(),
        }
    }


    /// Configuring the UART to match each received character,
    /// with the configured one.
    ///
    /// If the character is matched [`Event::CharacterMatch`] is generated,
    /// which can fire an intterrupt, if enabeled via [`Serial::listen()`]
    #[inline(always)]
    pub fn set_match_character(&mut self, char: u8) {
        self.usart.cr2.modify(|_, w| w.add().bits(char));
    }

    /// Read out the configured match character.
    #[inline(always)]
    pub fn match_character(&self) -> u8 {
        self.usart.cr2.read().add().bits()
    }

    #[inline(always)]
    pub fn set_wakeup_from_stopmode(&mut self, selection: StopModeWakeup) {
        self.usart.cr3.modify(|_, w| w.wus().variant(selection));
    }

    #[inline(always)]
    pub fn wakeup_from_stopmode_reason(&mut self) -> StopModeWakeup {
        match self.usart.cr3.read().wus().variant() {
            stm32f3::Variant::Val(val) => val,
            // The value is reservered and can not be configured,
            // so assume the default.
            stm32f3::Variant::Res(_) => StopModeWakeup::ADDRESS,
        }
    }

    /// Releases the USART peripheral and associated pins
    pub fn free(self) -> (Usart, (Tx, Rx)) {
        self.usart
            .cr1
            .modify(|_, w| w.ue().disabled().re().disabled().te().disabled());
        (self.usart, self.pins)
    }
}

impl<Usart, Tx, Rx> Serial<Usart, (Tx, Rx)>
where
    Usart: Instance + ReceiverTimeoutFeature,
{
    /// Set the receiver timeout value.
    ///
    /// The RTOF flag [`Event::ReceiverTimeout`] is set if, after the last received character,
    /// no new start bit is detected for more than the RTO value, where the value
    /// is beeing a counter, which is decresed by the configured baud rate.
    ///
    /// A simple calulation might be `time_per_counter_value = 1 / configured_baud_rate`
    ///
    /// ## Note
    ///
    /// - This value must only be programmed once per received character.
    /// - Can be written on the fly. If the new value is lower than or equal to the counter,
    ///   the RTOF flag is set.
    /// - Values higher than 24 bits are trunctuated to 24 bit max (16_777_216).
    // TODO: Not avaliable for STM32F303x6/8 and STM32F328x8 USART2 and USART3
    // Make it depended marker trait like DMA?
    pub fn receiver_timeout(&mut self, value: u32) {
        self.usart.rtor.modify(|_, w| w.rto().bits(value))
    }

    // TODO: Make value smarter
    /// Read out the currently
    pub fn read_receiver_timeout(&self) -> u32 {
        self.usart.rtor.read().rto().bits()
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

        // TODO: REACK TEACK???
        // TODO: Is clearing the interrupt event flag
        // a good thing or bad thing to do?
        Err(if isr.busy().bit_is_set() {
            nb::Error::WouldBlock
        } else if isr.pe().bit_is_set() {
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

impl<Usart> serial::Read<u8> for Rx<Usart>
where
    Usart: Instance,
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

impl<Usart> serial::Write<u8> for Tx<Usart>
where
    Usart: Instance,
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

impl<Usart> Rx<Usart>
where
    Usart: Instance + Dma,
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

impl<Usart> blocking::serial::write::Default<u8> for Tx<Usart> where Usart: Instance {}

impl<Usart> Tx<Usart>
where
    Usart: Instance + Dma,
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

impl<Usart> dma::Target for Rx<Usart>
where
    Usart: Instance + Dma,
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

impl<Usart> dma::Target for Tx<Usart>
where
    Usart: Instance + Dma,
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
pub trait ReceiverTimeoutFeature: crate::private::Sealed {}

impl ReceiverTimeoutFeature for USART1 {}
#[cfg(not(any(feature = "gpio-f333")))]
impl ReceiverTimeoutFeature for USART2 {}
#[cfg(not(any(feature = "gpio-f333")))]
impl ReceiverTimeoutFeature for USART3 {}

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


            impl<Tx, Rx> Serial<$USARTX, (Tx, Rx)> {
                /// Splits the [`Serial`] abstraction into a transmitter and a receiver half
                pub fn split(self) -> (split::Tx<$USARTX>, split::Rx<$USARTX>) {
                    // NOTE(unsafe): This essentially duplicates the USART peripheral
                    //
                    // As RX and TX both do have direct access to the peripheral,
                    // they must guarantee to only do atomic operations on the peripheral
                    // registers to avoid data races.
                    //
                    // Tx and Rx won't access the same registers anyways,
                    // as they have independet responbilities, which are NOT represented
                    // in the type system.
                    let (tx, rx) = unsafe {
                        (
                            pac::Peripherals::steal().$USARTX,
                            pac::Peripherals::steal().$USARTX,
                        )
                    };
                    (split::Tx::new(tx), split::Rx::new(rx))
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

        impl ReceiverTimeoutFeature for UART4 {}
        impl ReceiverTimeoutFeature for UART5 {}
    }
}
