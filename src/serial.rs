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

cfg_if! {
    if #[cfg(any(feature = "stm32f302", feature = "stm32f303"))] {
        use crate::dma;
        use cortex_m::interrupt;
    }
}

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

/// Serial abstraction
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
        /// # SAFTY
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
        /// # SAFTY
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

#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
impl<Usart> Rx<Usart>
where
    Usart: Instance,
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

#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
impl<Usart> Tx<Usart>
where
    Usart: Instance,
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

#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
impl<Usart> dma::Target for Rx<Usart>
where
    Usart: Instance,
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

#[cfg(any(feature = "stm32f302", feature = "stm32f303"))]
impl<Usart> dma::Target for Tx<Usart>
where
    Usart: Instance,
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

mod private {
    pub trait Sealed {}
}

/// UART instance
pub trait Instance: Deref<Target = RegisterBlock> + private::Sealed {
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
            impl private::Sealed for $USARTX {}
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
                /// Splits the `Serial` abstraction into a transmitter and a receiver half
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
// TODO: what about uart 4 and uart 5?
usart!([(1, 2), (2, 1), (3, 1)]);
