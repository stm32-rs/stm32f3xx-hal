#![no_std]
#![no_main]

use testsuite as _;

use enumset::EnumSet;

use stm32f3xx_hal as hal;

use hal::gpio::{OpenDrain, PushPull, AF7};
use hal::interrupts::InterruptNumber;
use hal::pac;
use hal::prelude::*;
use hal::serial::{
    config::{Config, Parity, StopBits},
    Error, Event, Instance, Serial,
};
use hal::time::rate::Baud;
use hal::{
    gpio::{
        gpioa::{PA10, PA2, PA3, PA9},
        gpiob::{PB10, PB11},
    },
    rcc::{Clocks, APB1, APB2},
};

use hal::interrupt;

use defmt::{assert, assert_eq, unwrap};

use core::sync::atomic::{AtomicBool, Ordering};

static INTERRUPT_FIRED: AtomicBool = AtomicBool::new(false);

type Serial1 = Serial<pac::USART1, (PA9<AF7<PushPull>>, PA10<AF7<PushPull>>)>;
type SerialSlow = Serial<pac::USART2, (PA2<AF7<PushPull>>, PA3<AF7<OpenDrain>>)>;
type SerialFast = Serial<pac::USART3, (PB10<AF7<PushPull>>, PB11<AF7<OpenDrain>>)>;

struct State {
    serial1: Option<Serial1>,
    serial_slow: Option<SerialSlow>,
    serial_fast: Option<SerialFast>,
    clocks: Clocks,
    apb1: APB1,
    apb2: APB2,
}

const BAUD_FAST: Baud = Baud(115_200);
const BAUD_SLOW: Baud = Baud(57_600);
const TEST_MSG: [u8; 8] = [0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0xaa, 0xbb];

fn test_test_msg_loopback(state: &mut State, config: impl Into<Config>) {
    let (usart, pins) = unwrap!(state.serial1.take()).free();
    let mut serial = Serial::new(usart, pins, config, state.clocks, &mut state.apb2);

    for i in &TEST_MSG {
        unwrap!(nb::block!(serial.write(*i)));
        let c = unwrap!(nb::block!(serial.read()));
        assert_eq!(c, *i);
    }

    state.serial1 = Some(serial);
}

fn trigger_event<Usart, Pins>(
    event: Event,
    serial: &mut Serial<Usart, Pins>,
    mut trigger: impl FnMut(&mut Serial<Usart, Pins>),
) where
    Usart: Instance,
{
    // Create an enumset of events with only one
    // event. Applying it disabled all other interrupts.
    let mut events = EnumSet::new();
    events.insert(event);
    // Clear events, so that previously triggered events do not fire
    // the now configured interupt imediatly.
    serial.clear_events();
    serial.configure_interrupts(events);
    // Check that the interrupt has not been run, since the configuration
    // and before the trigger.
    assert!(!INTERRUPT_FIRED.load(Ordering::SeqCst));
    trigger(serial);
    while !INTERRUPT_FIRED.load(Ordering::SeqCst) {}
    // Disable all configured interrupts.
    serial.configure_interrupts(EnumSet::new());
    // Only clear the particular event, which fired the interrupt
    assert!(serial.triggered_events().contains(event));
    serial.clear_event(event);
    assert!(!serial.triggered_events().contains(event));
    // TODO: Is that true?: Unpend any pending interrupts - more than one could be pending,
    // because of the late clearing of the interrupt
    cortex_m::peripheral::NVIC::unpend(<pac::USART1 as InterruptNumber>::INTERRUPT);
    // Now unmask all interrupts again, which where masks in the iterrupt rountine,
    // as a measurement to disable all interrupts.
    unsafe { cortex_m::peripheral::NVIC::unmask(<pac::USART1 as InterruptNumber>::INTERRUPT) }
    // Clear the interrupt flag again. And make double sure, that no interrupt
    // fired again.
    INTERRUPT_FIRED.store(false, Ordering::SeqCst);
    cortex_m::asm::delay(10);
    assert!(!INTERRUPT_FIRED.load(Ordering::SeqCst));
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert, assert_eq, unwrap};
    use testsuite::{CrossSerialPair1, CrossSerialPair2, SerialPair};

    #[init]
    fn init() -> super::State {
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(64.MHz())
            .freeze(&mut flash.acr);
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

        let serial_pair = SerialPair {
            0: gpioa
                .pa9
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            1: gpioa
                .pa10
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        };
        let cs_pair_1 = CrossSerialPair1 {
            0: gpioa
                .pa2
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
            1: gpiob
                .pb11
                .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh),
        };
        let cs_pair_2 = CrossSerialPair2 {
            0: gpiob
                .pb10
                .into_af_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh),
            1: gpioa
                .pa3
                .into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
        };

        let serial1 = Serial::new(
            dp.USART1,
            (serial_pair.0, serial_pair.1),
            9600.Bd(),
            clocks,
            &mut rcc.apb2,
        );

        unsafe { cortex_m::peripheral::NVIC::unmask(serial1.interrupt()) }

        super::State {
            serial1: Some(serial1),
            serial_slow: Some(Serial::new(
                dp.USART2,
                (cs_pair_1.0, cs_pair_2.1),
                BAUD_SLOW,
                clocks,
                &mut rcc.apb1,
            )),
            serial_fast: Some(Serial::new(
                dp.USART3,
                (cs_pair_2.0, cs_pair_1.1),
                BAUD_FAST,
                clocks,
                &mut rcc.apb1,
            )),
            clocks,
            apb1: rcc.apb1,
            apb2: rcc.apb2,
        }
    }

    #[test]
    fn test_overrun(state: &mut super::State) {
        let (usart, pins) = unwrap!(state.serial1.take()).free();
        let mut serial = Serial::new(usart, pins, 115200.Bd(), state.clocks, &mut state.apb2);
        // Provoke an overrun
        unwrap!(serial.bwrite_all(&TEST_MSG));

        // Very important, to have a fix blocking point.
        // Waiting for the transfer to be finished - this implementation is
        // now independent of the choosen baudrate.
        unwrap!(serial.bflush());
        let c = nb::block!(serial.read());
        assert!(matches!(c, Err(Error::Overrun)));
        // Ensure that the receiver data reigster is empty.
        // Another nb::block!(serial.read()) should block forever.
        while serial.is_busy() {}
        assert!(!serial.is_event_triggered(Event::ReceiveDataRegisterNotEmpty));
        serial.clear_events();

        state.serial1 = Some(serial);
    }

    #[test]
    fn config_builder() {
        let default = Config::default();

        let built_1 = Config::default()
            .baudrate(123_456.Bd())
            .parity(Parity::Even)
            .stopbits(StopBits::Stop0P5);
        assert!(built_1.baudrate == 123_456.Bd());
        assert!(built_1.parity == Parity::Even);
        assert!(built_1.stopbits == StopBits::Stop0P5);

        let built_1_different_order = Config::default()
            .baudrate(42.Bd())
            .stopbits(StopBits::Stop0P5)
            .parity(Parity::Even)
            .baudrate(123_456.Bd());
        assert!(built_1 == built_1_different_order);

        let built_2 = Config::default().parity(Parity::Odd);
        assert!(built_2.baudrate == default.baudrate);
        assert!(built_2.parity == Parity::Odd);
        assert!(built_2.stopbits == default.stopbits);

        assert!(built_1 != default);
        assert!(built_1 != built_2);
    }

    #[test]
    fn some_baudrates_loopback(state: &mut super::State) {
        use hal::time::rate::Baud;
        for baudrate in &[
            Baud(1200),
            Baud(9600),
            Baud(19200),
            Baud(38400),
            Baud(57600),
            Baud(115200),
            Baud(230400),
            Baud(460800),
        ] {
            test_test_msg_loopback(state, *baudrate);
        }
    }

    #[rustfmt::skip]
    #[test]
    fn parity_modes_loopback(state: &mut super::State) {
        for parity in &[
            Parity::Even,
            Parity::Odd,
            Parity::None,
        ] {
            let config = Config::default().parity(*parity);
            test_test_msg_loopback(state, config);
        }
    }

    #[test]
    fn stopbits_loopback(state: &mut super::State) {
        for stopbits in &[
            StopBits::Stop0P5,
            StopBits::Stop1,
            StopBits::Stop1P5,
            StopBits::Stop2,
        ] {
            let config = Config::default().stopbits(*stopbits);
            test_test_msg_loopback(state, config);
        }
    }

    #[test]
    fn check_parity_mismatch(state: &mut super::State) {
        // Repurpose crossover serials from state.
        let (usart_even, pins_even) = unwrap!(state.serial_slow.take()).free();
        let config_even = Config::default().parity(Parity::Even);
        let mut serial_even = Serial::new(
            usart_even,
            pins_even,
            config_even,
            state.clocks,
            &mut state.apb1,
        );
        let (usart_odd, pins_odd) = unwrap!(state.serial_fast.take()).free();
        let config_odd = Config::default().parity(Parity::Odd);
        let mut serial_odd = Serial::new(
            usart_odd,
            pins_odd,
            config_odd,
            state.clocks,
            &mut state.apb1,
        );

        // Transmit data with wrong parity in both directions.
        unwrap!(nb::block!(serial_even.write(b'x')));
        let result = nb::block!(serial_odd.read());
        assert!(matches!(result, Err(Error::Parity)));

        unwrap!(nb::block!(serial_odd.write(b'x')));
        let result = nb::block!(serial_even.read());
        // Note: I'm not really sure in what case Framing is thrown but not Parity
        // but it happens
        assert!(matches!(result, Err(Error::Parity | Error::Framing)));

        // Finally restore serial devices in state.
        let (usart_slow, pins_slow) = serial_even.free();
        let serial_slow = Serial::new(
            usart_slow,
            pins_slow,
            BAUD_SLOW,
            state.clocks,
            &mut state.apb1,
        );
        let (usart_fast, pins_fast) = serial_odd.free();
        let serial_fast = Serial::new(
            usart_fast,
            pins_fast,
            BAUD_FAST,
            state.clocks,
            &mut state.apb1,
        );
        state.serial_slow = Some(serial_slow);
        state.serial_fast = Some(serial_fast);
    }

    #[test]
    fn send_receive_wrong_baud(state: &mut super::State) {
        let mut usart_slow = unwrap!(state.serial_slow.take());
        let mut usart_fast = unwrap!(state.serial_fast.take());

        // provoke an error (framing)
        unwrap!(nb::block!(usart_slow.write(b'a')));
        let c = nb::block!(usart_fast.read());
        defmt::debug!("{}", c);
        assert!(matches!(c, Err(Error::Framing)));

        // provoke an error (this does not seem to be absolutely deterministic
        // and we've seen multiple error variants in the wild, including
        // receiving the wrong but valid character)
        unwrap!(nb::block!(usart_fast.write(b'a')));
        let result = nb::block!(usart_slow.read());
        defmt::debug!("{}", result);
        assert!(match result {
            Ok(c) => {
                defmt::warn!("Expected Err, got Ok({:x})", c);
                c != b'a'
            }
            Err(Error::Framing | Error::Noise) => true,
            Err(_) => false,
        });

        state.serial_slow = Some(usart_slow);
        state.serial_fast = Some(usart_fast);
    }

    // TODO: Currently, this is a limited test, just to see, that
    // an interrupt has fired. It does **not** test, if the correct
    // event caused the interrupt.
    //
    // This increases the implemetation effort, because
    #[test]
    fn trigger_events(state: &mut super::State) {
        let mut serial = state.serial1.take().unwrap();
        // let mut events = EnumSet::new();

        trigger_event(Event::ReceiveDataRegisterNotEmpty, &mut serial, |serial| {
            unwrap!(serial.write(b'A').ok());
        });

        trigger_event(Event::TransmissionComplete, &mut serial, |serial| {
            unwrap!(serial.write(b'A').ok());
        });

        // TODO: This is difficult to test, as the data reigster is
        // empty imidiatly, when the interrupt is configured.
        // trigger_event(Event::TransmitDataRegisterEmtpy, &mut serial, |serial| {
        //     unwrap!(serial.write(b'A').ok());
        // });

        trigger_event(Event::OverrunError, &mut serial, |serial| {
            // Imidiatly overrun, because we do not read out the received register.
            unwrap!(nb::block!(serial.write(b'A')).ok());
        });

        // FIXME: This test is sensitive to timing and the event might already be triggered by a
        // previous transmission. (More details about IDLE event RM0316 29.8.1)
        #[cfg(feature = "disabled")]
        trigger_event(Event::Idle, &mut serial, |serial| {
            // Note: The IDLE bit will not be set again until the RXNE bit has been set (i.e. a new
            // idle line occurs).
            // To provoke IDLE, send something again so that RXNE is set.
            unwrap!(nb::block!(serial.write(b'A')).ok());
        });

        serial.set_match_character(b'A');
        assert!(serial.match_character() == b'A');
        trigger_event(Event::CharacterMatch, &mut serial, |serial| {
            unwrap!(nb::block!(serial.write(b'A')).ok());
        });

        serial.set_receiver_timeout(Some(100));
        assert!(serial.receiver_timeout() == Some(100));
        trigger_event(Event::ReceiverTimeout, &mut serial, |serial| {
            unwrap!(nb::block!(serial.write(b'A')).ok());
            unwrap!(nb::block!(serial.read()));
        });

        state.serial1 = Some(serial);
    }

    #[test]
    fn test_disabled_overrun(state: &mut super::State) {
        let mut serial = state.serial1.take().unwrap();

        serial.detect_overrun(false);
        while serial.is_busy() {}

        // Writ the whole TEST_MSG content and check if the last bit
        // is in the register.
        unwrap!(serial.bwrite_all(&TEST_MSG));
        unwrap!(serial.bflush());
        assert_eq!(
            unwrap!(nb::block!(serial.read())),
            *TEST_MSG.iter().rev().next().unwrap()
        );

        // enable overrun again.
        serial.detect_overrun(true);

        state.serial1 = Some(serial);
    }

    #[test]
    fn test_busy_does_not_prevent_reading_data_register(state: &mut super::State) {
        let mut serial = state.serial1.take().unwrap();

        while serial.is_busy() {}

        let b1 = TEST_MSG[0];
        let b2 = TEST_MSG[1];

        unwrap!(nb::block!(serial.write(b1)));
        while !serial.is_busy() {} // Wait until reception b1 started
        while serial.is_busy() {} // Wait until reception b1 complete
        unwrap!(nb::block!(serial.write(b2)));
        while !serial.is_busy() {} // Wait until reception b2 started

        let c1 = unwrap!(nb::block!(serial.read())); // b1 in data register, b2 in shift register
        let c2 = unwrap!(nb::block!(serial.read())); // wait for b2 in data register

        assert_eq!(b1, c1);
        assert_eq!(b2, c2);

        state.serial1 = Some(serial);
    }
}

// TODO: This maybe can be moved into a function inside the
// mod tests, if defmt_test does allow free functions, which do not
// correspond to tests.
#[interrupt]
fn USART1_EXTI25() {
    INTERRUPT_FIRED.store(true, Ordering::SeqCst);

    // Make it easy on ourselfs and just disable all interrupts.
    // This way, the interrupt rountine dooes not have to have access to the serial peripheral,
    // which would mean to access the internally managed state of the test module,
    // which can't be accessed right now.
    //
    // This is all needed, to clear the fired interrupt.
    assert!(cortex_m::peripheral::NVIC::is_active(
        <pac::USART1 as InterruptNumber>::INTERRUPT
    ));
    cortex_m::peripheral::NVIC::mask(<pac::USART1 as InterruptNumber>::INTERRUPT);
}
