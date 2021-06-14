#![no_std]
#![no_main]

use testsuite as _;

use stm32f3xx_hal as hal;

use hal::gpio::{OpenDrain, PushPull, AF7};
use hal::pac;
use hal::prelude::*;
use hal::serial::Serial;
use hal::{
    gpio::{
        gpioa::{PA10, PA2, PA3, PA9},
        gpiob::{PB10, PB11},
    },
    rcc::{Clocks, APB2},
};

use core::array::IntoIter;

use hal::serial::Error;

struct State {
    serial1: Option<Serial<pac::USART1, (PA9<AF7<PushPull>>, PA10<AF7<PushPull>>)>>,
    serial_slow: Option<Serial<pac::USART2, (PA2<AF7<PushPull>>, PA3<AF7<OpenDrain>>)>>,
    serial_fast: Option<Serial<pac::USART3, (PB10<AF7<PushPull>>, PB11<AF7<OpenDrain>>)>>,
    clocks: Clocks,
    apb2: APB2,
}

const TEST_MSG: [u8; 8] = [0xD, 0xE, 0xA, 0xD, 0xB, 0xE, 0xE, 0xF];

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
            serial1: Some(Serial::usart1(
                dp.USART1,
                (serial_pair.0, serial_pair.1),
                9600.Bd(),
                clocks,
                &mut rcc.apb2,
            )),
            serial_slow: Some(Serial::usart2(
                dp.USART2,
                (cs_pair_1.0, cs_pair_2.1),
                57600.Bd(),
                clocks,
                &mut rcc.apb1,
            )),
            serial_fast: Some(Serial::usart3(
                dp.USART3,
                (cs_pair_2.0, cs_pair_1.1),
                115200.Bd(),
                clocks,
                &mut rcc.apb1,
            )),
            clocks,
            apb2: rcc.apb2,
        }
    }

    #[test]
    fn test_many_baudrates(state: &mut super::State) {
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
            let (usart, pins) = unwrap!(state.serial1.take()).free();
            let mut serial = Serial::usart1(usart, pins, *baudrate, state.clocks, &mut state.apb2);
            for i in &TEST_MSG {
                unwrap!(nb::block!(serial.write(*i)));
                let c = unwrap!(nb::block!(serial.read()));
                assert_eq!(c, *i);
            }
            state.serial1 = Some(serial);
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
        for i in &TEST_MSG[..4] {
            defmt::unwrap!(nb::block!(tx.write(*i)));
        }
        let c = nb::block!(rx.read());
        assert!(matches!(c, Err(Error::Overrun)));
    }

    #[test]
    fn send_receive_wrong_baud(state: &mut super::State) {
        let (mut tx_slow, mut rx_slow) = unwrap!(state.serial_slow.take()).split();
        let (mut tx_fast, mut rx_fast) = unwrap!(state.serial_fast.take()).split();

        // provoke an error (framing)
        defmt::unwrap!(nb::block!(tx_slow.write(b'a')));
        let c = nb::block!(rx_fast.read());
        defmt::info!("{}", c);
        assert!(matches!(c, Err(Error::Framing)));

        // provoke an error (this does not seem to be absolutely deterministic
        // and we've seen multiple error variants in the wild)
        unwrap!(nb::block!(tx_fast.write(b'a')));
        let c = nb::block!(rx_slow.read());
        defmt::info!("{}", c);
        assert!(matches!(c, Err(Error::Framing) | Err(Error::Noise)));
    }

    // TODO: Check the parity. But currently, there is no way to configure the parity
    // #[test]
    // fn check_parity(state: &mut super::State) { }

    // TODO: Test interrupts
    // #[test]
    // fn enable_interrupt_and_wait_for_fire(state: &mut super::State) {}
}
