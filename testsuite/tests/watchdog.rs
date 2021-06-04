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
    use defmt::{assert_eq, unwrap};
    use hal::time::duration::*; // imports all duration-related types and traits
    use hal::time::duration::{Duration, Milliseconds, Nanoseconds, Seconds};
    use hal::time::rate::Rate;
    use hal::time::rate::*; // imports all rate-related types and traits
    use hal::time::TimeInt;

    const INTERVAL: Milliseconds = Milliseconds(100u32);

    #[init]
    fn init() -> State {
        let dp = unwrap!(hal::pac::Peripherals::take());

        let mut rcc = dp.RCC.constrain();
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
        let delay: u32 = (u32::try_from(
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
        .unwrap());
        defmt::info!("Delay = {}", delay);
        for _ in 0..5 {
            state.iwdg.feed();

            asm::delay(delay);
        }
    }

    // It takes some time, until defmt_test exits.
    // In this time, the wathcodg  can not be fed. So disable it in the last test.
    #[test]
    fn disable(state: &mut State) {
        state.iwdg.stop();
    }
}
