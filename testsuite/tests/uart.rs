#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::{OpenDrain, PushPull, AF7};
use hal::pac;
use hal::prelude::*;
use hal::serial::Serial;
use hal::serial::{
    config::{Config, Parity, StopBits},
    Error,
};
use hal::time::rate::Baud;
use hal::{
    gpio::{
        gpioa::{PA10, PA2, PA3, PA9},
        gpiob::{PB10, PB11},
    },
    rcc::{Clocks, APB1, APB2},
};

use core::array::IntoIter;
use defmt::{assert_eq, unwrap};

struct State {
    serial1: Option<Serial<pac::USART1, (PA9<AF7<PushPull>>, PA10<AF7<PushPull>>)>>,
    serial_slow: Option<Serial<pac::USART2, (PA2<AF7<PushPull>>, PA3<AF7<OpenDrain>>)>>,
    serial_fast: Option<Serial<pac::USART3, (PB10<AF7<PushPull>>, PB11<AF7<OpenDrain>>)>>,
    clocks: Clocks,
    apb1: APB1,
    apb2: APB2,
}

const BAUD_FAST: Baud = Baud(115_200);
const BAUD_SLOW: Baud = Baud(57_600);
const TEST_MSG: [u8; 8] = [0xD, 0xE, 0xA, 0xD, 0xB, 0xE, 0xE, 0xF];

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
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

        let serial_pair = SerialPair {
            0: gpioa
                .pa9
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            1: gpioa
                .pa10
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        };
        let cs_pair_1 = CrossSerialPair1 {
            0: gpioa
                .pa2
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
            1: gpiob
                .pb11
                .into_af7_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh),
        };
        let cs_pair_2 = CrossSerialPair2 {
            0: gpiob
                .pb10
                .into_af7_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh),
            1: gpioa
                .pa3
                .into_af7_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
        };

        super::State {
            serial1: Some(Serial::new(
                dp.USART1,
                (serial_pair.0, serial_pair.1),
                9600.Bd(),
                clocks,
                &mut rcc.apb2,
            )),
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
    fn send_receive_split(state: &mut super::State) {
        let (mut tx, mut rx) = unwrap!(state.serial1.take()).split();
        for i in IntoIter::new(TEST_MSG) {
            defmt::unwrap!(nb::block!(tx.write(i)));
            let c = unwrap!(nb::block!(rx.read()));
            assert_eq!(c, i);
        }

        // now provoke an overrun
        // send 5 u8 bytes, which do not fit in the 32 bit buffer
        for i in &TEST_MSG[..5] {
            defmt::unwrap!(nb::block!(tx.write(*i)));
        }
        let c = nb::block!(rx.read());
        assert!(matches!(c, Err(Error::Overrun)));

        state.serial1 = Some(Serial::join(tx, rx));
    }

    #[test]
    fn config_builder() {
        let default = Config::default();

        let built_1 = Config::default()
            .baudrate(123_456.Bd())
            .parity(Parity::Even)
            .stopbits(StopBits::STOP0P5);
        assert!(built_1.baudrate == 123_456.Bd());
        assert!(built_1.parity == Parity::Even);
        assert!(built_1.stopbits == StopBits::STOP0P5);

        let built_1_different_order = Config::default()
            .baudrate(42.Bd())
            .stopbits(StopBits::STOP0P5)
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
            StopBits::STOP0P5,
            StopBits::STOP1,
            StopBits::STOP1P5,
            StopBits::STOP2,
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
        assert!(matches!(result, Err(Error::Parity)));

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
        let (mut tx_slow, mut rx_slow) = unwrap!(state.serial_slow.take()).split();
        let (mut tx_fast, mut rx_fast) = unwrap!(state.serial_fast.take()).split();

        // provoke an error (framing)
        unwrap!(nb::block!(tx_slow.write(b'a')));
        let c = nb::block!(rx_fast.read());
        defmt::debug!("{}", c);
        assert!(matches!(c, Err(Error::Framing)));

        // provoke an error (this does not seem to be absolutely deterministic
        // and we've seen multiple error variants in the wild, including
        // receiving the wrong but valid character)
        unwrap!(nb::block!(tx_fast.write(b'a')));
        let result = nb::block!(rx_slow.read());
        defmt::debug!("{}", result);
        assert!(match result {
            Ok(c) => {
                defmt::warn!("Expected Err, got Ok({:x})", c);
                c != b'a'
            }
            Err(Error::Framing | Error::Noise) => true,
            Err(_) => false,
        });

        state.serial_slow = Some(Serial::join(tx_slow, rx_slow));
        state.serial_fast = Some(Serial::join(tx_fast, rx_fast));
    }

    // TODO: Test interrupts
    // #[test]
    // fn enable_interrupt_and_wait_for_fire(state: &mut super::State) {}
}
