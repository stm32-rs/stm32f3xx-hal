#![no_main]
#![no_std]

use panic_rtt_target as _;

#[rtic::app(device = stm32f3xx_hal::pac, dispatchers = [TIM20_BRK, TIM20_UP, TIM20_TRG_COM])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f3xx_hal::{
        gpio::{self, Output, PushPull, AF7},
        prelude::*,
        serial::{Event, Serial},
    };

    #[monotonic(binds = SysTick, default = true)]
    type DwtMono = DwtSystick<48_000_000>;

    type SerialType = Serial<
        stm32f3xx_hal::pac::USART1,
        (
            gpio::gpioa::PA9<AF7<PushPull>>,
            gpio::gpioa::PA10<AF7<PushPull>>,
        ),
    >;
    type DirType = stm32f3xx_hal::gpio::gpioe::PE13<Output<PushPull>>;
    #[resources]
    struct Resources {
        serial: SerialType,
        dir: DirType,
    }

    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut dcb = cx.core.DCB;
        let dwt = cx.core.DWT;
        let systick = cx.core.SYST;

        rtt_init_print!(NoBlockSkip, 4096);
        rprintln!("pre init");

        // Initialize the clocks
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze(&mut flash.acr);
        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        // Initialize the peripherals
        // DIR
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.ahb);
        let mut dir: DirType = gpioe
            .pe13
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
        dir.set_low().unwrap();

        // SERIAL
        let mut gpioa = cx.device.GPIOA.split(&mut rcc.ahb);
        let mut pins = (
            gpioa
                .pa9
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
            gpioa
                .pa10
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        );
        pins.1.internal_pull_up(&mut gpioa.pupdr, true);
        let mut serial: SerialType = Serial::usart1(
            cx.device.USART1,
            pins,
            19200_u32.Bd(),
            clocks,
            &mut rcc.apb2,
        );
        serial.listen(Event::Rxne);

        rprintln!("post init");

        task1::spawn().unwrap();

        (init::LateResources { dir, serial }, init::Monotonics(mono))
    }

    #[task(binds = USART1_EXTI25, resources = [serial, dir])]
    fn protocol_serial_task(cx: protocol_serial_task::Context) {
        let mut serial = cx.resources.serial;
        let mut dir = cx.resources.dir;

        serial.lock(|serial| {
            dir.lock(|dir| {
                if serial.is_rxne() {
                    dir.set_high().unwrap();
                    serial.unlisten(Event::Rxne);
                    match serial.read() {
                        Ok(byte) => {
                            serial.write(byte).unwrap();
                            serial.listen(Event::Tc);
                        }
                        Err(_error) => rprintln!("irq error"),
                    };
                }

                if serial.is_tc() {
                    dir.set_low().unwrap();
                    serial.unlisten(Event::Tc);
                    serial.listen(Event::Rxne);
                }
            })
        });
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
