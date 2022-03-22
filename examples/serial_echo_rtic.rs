#![no_main]
#![no_std]

use panic_rtt_target as _;

// This example application prints all characters it receives via Serial and sends them back.
//
// NOTES:
// - Depending on the board you're using, `Serial` might need different Tx and Rx pins instead of
//   pa9 and pa10 (for example, the STM32f3DISCOVERY board uses pc4 and pc5).
//   Check your boards' schematic and adjust the code accordingly.
// - https://docs.rust-embedded.org/discovery/10-serial-communication/nix-tooling.html?highlight=*nix#nix-tooling
//   has instructions on how to connect to your Serial device (make sure to adjust the baud rate)
#[rtic::app(device = stm32f3xx_hal::pac, dispatchers = [TIM20_BRK, TIM20_UP, TIM20_TRG_COM])]
mod app {
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f3xx_hal::{
        gpio::{self, Output, PushPull, AF7},
        pac,
        prelude::*,
        serial::{Event, Serial},
        Switch,
    };
    use systick_monotonic::*;

    #[monotonic(binds = SysTick, default = true)]
    type AppMono = Systick<100>; // 100 Hz / 10 ms granularity

    type SerialType = Serial<pac::USART1, (gpio::PA9<AF7<PushPull>>, gpio::PA10<AF7<PushPull>>)>;
    // The LED that will light up when data is received via serial
    type DirType = gpio::PE13<Output<PushPull>>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        serial: SerialType,
        dir: DirType,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let systick = cx.core.SYST;

        rtt_init_print!(NoBlockSkip, 4096);
        rprintln!("pre init");

        // Initialize the clocks
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze(&mut flash.acr);
        let mono = Systick::new(systick, 48_000_000);

        // Initialize the peripherals
        // DIR (the LED that lights up during serial rx)
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let mut dir: DirType = gpioe
            .pe13
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        dir.set_low();

        // SERIAL
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let pins = (
            gpioa
                // Tx pin
                .pa9
                // configure this pin to make use of its `USART1_TX` alternative function
                // (AF mapping taken from table 14 "Alternate functions for port A" of the datasheet at
                //  https://www.st.com/en/microcontrollers-microprocessors/stm32f303vc.html)
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            gpioa
                // Rx pin
                .pa10
                // configure this pin to make use of its `USART1_RX` alternative function
                // (AF mapping taken from table 14 "Alternate functions for port A" of the datasheet at
                //  https://www.st.com/en/microcontrollers-microprocessors/stm32f303vc.html)
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh)
                .internal_pull_up(&mut gpioa.pupdr, true),
        );
        let mut serial: SerialType =
            Serial::new(cx.device.USART1, pins, 19200.Bd(), clocks, &mut rcc.apb2);
        serial.configure_interrupt(Event::ReceiveDataRegisterNotEmpty, Toggle::On);

        rprintln!("post init");

        // A kick of for the infitinte loop in protocol_serial_task()
        nb::block!(serial.write(b'c')).unwrap();

        task1::spawn().unwrap();

        (Shared {}, Local { serial, dir }, init::Monotonics(mono))
    }

    #[task(binds = USART1_EXTI25, local = [serial, dir])]
    fn protocol_serial_task(cx: protocol_serial_task::Context) {
        let serial = cx.local.serial;
        let dir = cx.local.dir;

        if serial.is_event_triggered(Event::ReceiveDataRegisterNotEmpty) {
            dir.set_high();
            serial.configure_interrupt(Event::ReceiveDataRegisterNotEmpty, Switch::Off);
            match serial.read() {
                Ok(byte) => {
                    serial.write(byte).unwrap();
                    rprintln!("{:?}", char::from_u32(byte.into()).unwrap_or('?'));
                    serial.configure_interrupt(Event::TransmissionComplete, Switch::On);
                }
                Err(_error) => rprintln!("irq error"),
            };
        }

        // It is perfectly viable to just use `is_event_triggered` here,
        // but this is a showcase, to also be able to used `triggered_events`
        // and other functions enabled by the "enumset" feature.
        let events = serial.triggered_events();
        if events.contains(Event::TransmissionComplete) {
            dir.set_low();
            let interrupts = {
                let mut interrupts = enumset::EnumSet::new();
                interrupts.insert(Event::ReceiveDataRegisterNotEmpty);
                interrupts
            };
            serial.clear_event(Event::TransmissionComplete);
            // Disable all interrupts, except ReceiveDataRegisterNotEmpty.
            serial.configure_interrupts(interrupts);
        }
    }

    #[task]
    fn task1(_cx: task1::Context) {
        rprintln!("task1");
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        rprintln!("idle");
        loop {
            cortex_m::asm::nop();
        }
    }
}
