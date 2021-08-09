#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use hal::interrupts::InterruptNumber;
use hal::rcc::{Clocks, APB1};
use hal::timer::{Event, MonoTimer, Timer};
use hal::{interrupt, pac, prelude::*};

use pac::TIM2;

struct State {
    timer: Option<Timer<TIM2>>,
    mono_timer: MonoTimer,
    clocks: Clocks,
    apb1: APB1,
}

static INTERRUPT_FIRED: AtomicBool = AtomicBool::new(false);

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert, unwrap};
    use hal::time::fixed_point::FixedPoint;

    #[init]
    fn init() -> State {
        let mut cp = unwrap!(pac::CorePeripherals::take());
        let dp = unwrap!(pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        // Let's use a timer, which is avaliable for every chip
        let timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
        let mono_timer = MonoTimer::new(cp.DWT, clocks, &mut cp.DCB);

        assert!(mono_timer.frequency() == clocks.hclk());

        unsafe { cortex_m::peripheral::NVIC::unmask(timer.interrupt()) };

        State {
            timer: Some(timer),
            mono_timer,
            clocks,
            apb1: rcc.apb1,
        }
    }

    #[test]
    fn test_stop_and_free(state: &mut State) {
        let timer = state.timer.take().unwrap().free();

        let timer = Timer::new(timer, state.clocks, &mut state.apb1);

        state.timer = Some(timer);
    }

    #[test]
    fn test_delay(state: &mut State) {
        let mut timer = unwrap!(state.timer.take());
        defmt::trace!("{}", state.mono_timer);
        let freqcyc = state.mono_timer.frequency().integer();

        let instant = state.mono_timer.now();
        timer.start(1.seconds());
        assert!(!timer.is_event_triggered(Event::Update));
        unwrap!(nb::block!(timer.wait()).ok());
        let elapsed = instant.elapsed();

        defmt::info!("elapsed: {}", elapsed);
        // TODO: Test multiple frequencies and change the cycle counter window to a percentage
        // to measure the overhead of start?

        // Differentiate between releas and debug build.
        #[cfg(not(debug_assertions))]
        assert!(((freqcyc - 500)..(freqcyc + 500)).contains(&elapsed));
        #[cfg(debug_assertions)]
        assert!(((freqcyc - 2000)..(freqcyc + 2000)).contains(&elapsed));

        state.timer = Some(timer);
    }

    // TODO(Sh3Rm4n): For that we have to introduce a new API. (Maybe Centihertz or Something?)
    // #[test]
    // fn test_delay_longer_then_second(state: &mut State) {

    // }

    #[test]
    fn test_interrupt(state: &mut State) {
        let mut timer = unwrap!(state.timer.take());

        assert!(!INTERRUPT_FIRED.load(Ordering::SeqCst));
        timer.enable_interrupt(Event::Update);
        timer.start(1.milliseconds());

        while !INTERRUPT_FIRED.load(Ordering::SeqCst) {}

        assert!(timer.is_event_triggered(Event::Update));
        timer.clear_event(Event::Update);
        assert!(!timer.is_event_triggered(Event::Update));

        state.timer = Some(timer);
    }
}

#[interrupt]
fn TIM2() {
    INTERRUPT_FIRED.store(true, Ordering::SeqCst);

    // Make it easy on ourselfs and just disable all interrupts.
    // This way, the interrupt rountine dooes not have to have access to the timer peripheral,
    // which would mean to access the internally managed state of the test module,
    // which can't be accessed right now.
    //
    // This is all needed, to clear the fired interrupt.
    assert!(cortex_m::peripheral::NVIC::is_active(
        <pac::TIM2 as InterruptNumber>::INTERRUPT
    ));
    cortex_m::peripheral::NVIC::mask(<pac::TIM2 as InterruptNumber>::INTERRUPT);
}
