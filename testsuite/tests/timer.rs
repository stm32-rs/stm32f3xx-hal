#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt_rtt as _;
use panic_probe as _;

use num_traits::float::FloatCore;

use stm32f3xx_hal as hal;

use hal::delay::Delay;
use hal::hal::timer::Cancel;
use hal::interrupts::InterruptNumber;
use hal::rcc::{Clocks, APB1};
use hal::time::{duration, fixed_point::FixedPoint};
use hal::timer::{AlreadyCancled, Event, MonoTimer, Timer};
use hal::{interrupt, pac, prelude::*};

use pac::TIM2;

struct State {
    timer: Option<Timer<TIM2>>,
    mono_timer: MonoTimer,
    clocks: Clocks,
    apb1: APB1,
    delay: Delay,
}

static INTERRUPT_FIRED: AtomicBool = AtomicBool::new(false);

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{self, assert, unwrap};

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
        let delay = Delay::new(cp.SYST, clocks);

        assert!(mono_timer.frequency() == clocks.hclk());

        unsafe { cortex_m::peripheral::NVIC::unmask(timer.interrupt()) };

        State {
            timer: Some(timer),
            mono_timer,
            clocks,
            apb1: rcc.apb1,
            delay,
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

        let durations: [duration::Generic<u32>; 5] = [
            100.microseconds().into(),
            1.milliseconds().into(),
            100.milliseconds().into(),
            1.seconds().into(),
            10.seconds().into(),
            // 100.seconds().into(),
        ];

        for dur in durations {
            defmt::trace!("Duration: {}", defmt::Debug2Format(&dur));

            timer.start(dur);
            assert!(!timer.is_event_triggered(Event::Update));
            // call instant after start, because starting the timer is the last thing done in the
            // start function, and therefor no overhead is added to the timing.
            let instant = state.mono_timer.now();
            unwrap!(nb::block!(timer.wait()).ok());
            let elapsed = instant.elapsed();

            defmt::debug!("elapsed: {}", elapsed);

            let ratio = f64::from(elapsed) / f64::from(freqcyc)
                * (f64::from(*dur.scaling_factor().denominator()))
                / f64::from(dur.integer());

            let deviation = (ratio - 1.).abs();

            // Deviation is high for smaller timer durations. Higher duratinons are pretty accurate.
            // TODO: Maybe the allowed deviation should changed depending on the duration?
            defmt::assert!(deviation < 11e-02);
        }
        state.timer = Some(timer);
    }

    #[test]
    fn test_cancel(state: &mut State) {
        let mut timer = unwrap!(state.timer.take());
        defmt::trace!("{}", state.mono_timer);

        timer.start(10.milliseconds());
        state.delay.delay_ms(5u32);

        assert!(matches!(timer.cancel(), Ok(())));
        assert!(matches!(timer.cancel(), Err(AlreadyCancled)));
        state.timer = Some(timer);
    }

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
