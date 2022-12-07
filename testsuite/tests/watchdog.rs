#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm;

use core::convert::TryFrom;

use hal::prelude::*;
use stm32f3xx_hal as hal;

use hal::rcc::Clocks;
use hal::watchdog::IndependentWatchDog;

struct State {
    iwdg: IndependentWatchDog,
    clocks: Clocks,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::{assert, assert_eq, unwrap};
    use hal::time::duration::{Milliseconds, Nanoseconds};
    use hal::time::fixed_point::FixedPoint;
    use hal::time::rate::{Kilohertz, Rate};

    const INTERVAL: Milliseconds = Milliseconds(100u32);

    #[init]
    fn init() -> State {
        let dp = unwrap!(hal::pac::Peripherals::take());

        let rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();

        // Watchdog makes sure this gets restarted periodically if nothing happens
        let mut iwdg = IndependentWatchDog::new(dp.IWDG);

        // iwdg.stop_on_debug(&dp.DBGMCU, true);
        iwdg.start(INTERVAL);

        State {
            iwdg,
            clocks: rcc.cfgr.freeze(&mut flash.acr),
        }
    }

    #[test]
    fn setup_equals_interval(state: &mut State) {
        state.iwdg.feed();
        assert!(state.iwdg.interval() == INTERVAL);
    }

    #[test]
    // A watchdog trigger is not cleanly oberserable other than that
    // the probe-run output is repeated many times (as the core is restareted).
    fn feed(state: &mut State) {
        // Calculate some overhead which is introduced by asm::delay
        let interval_wo_overhead = INTERVAL - 35.milliseconds();
        let delay: u32 = u32::try_from(
            Nanoseconds::from(interval_wo_overhead).integer()
                / u64::from(
                    state
                        .clocks
                        .sysclk()
                        .to_duration::<Nanoseconds>()
                        .unwrap()
                        .integer(),
                ),
        )
        .unwrap();
        defmt::debug!("Delay = {}", delay);
        for _ in 0..5 {
            state.iwdg.feed();

            asm::delay(delay);
        }
    }

    // TODO:
    #[test]
    fn test_intervals_around_maximum(state: &mut State) {
        const MAX_RELOAD: u32 = 0x0FFF;
        const MAX_PRESCALER: u32 = 256;
        const LSI: Kilohertz = Kilohertz(40);
        let max_period = Milliseconds(MAX_PRESCALER * MAX_RELOAD / LSI.integer());
        let mut expected_interval = Milliseconds(15000);
        while expected_interval < max_period + 100.milliseconds() {
            expected_interval = expected_interval + 100.milliseconds();

            state.iwdg.feed();
            state.iwdg.start(expected_interval);
            let interval = state.iwdg.interval();
            if interval < max_period {
                // Test if the approximate value is reached (it is slightly lower most of the time)
                assert!(
                    interval.integer()
                        >= expected_interval
                            .integer()
                            .saturating_sub(10.milliseconds().integer())
                );
            }
        }
        assert_eq!(state.iwdg.interval().integer(), max_period.integer());
    }

    // It takes some time, until defmt_test exits.
    // In this time, the watchdog can not be fed.
    // set the value high enough, so this does not happen.
    #[test]
    fn finish(state: &mut State) {
        state.iwdg.start(1000.milliseconds());
    }
}
