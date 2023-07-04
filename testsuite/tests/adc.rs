#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use hal::adc::{
    self,
    config::{self, Config},
    Adc, CommonAdc, CommonInstance, Event, TemperatureSensor, VoltageInternalReference,
};
use hal::gpio::{Analog, Output, PushPull};
use hal::pac;
use hal::prelude::*;
use hal::{
    gpio::gpioc::{PC0, PC1},
    rcc::{Clocks, AHB},
};

use hal::interrupt;

// TODO:
// [ ] Test conversion modes
// [ ] Test against DMA
// [ ] Test overrun?
// [ ] Test continous mode between mutliple channels
// [ ] Test interrupts
// [ ] Test external trigger
// [ ] Test internal sensor reading (e.g. Temperature)
// [ ] Test stopping a conversion.

struct State {
    adc: Option<Adc<pac::ADC1>>,
    adc2: Option<pac::ADC2>,
    adc_common: Option<CommonAdc<pac::ADC1_2>>,
    analog: PC0<Analog>,
    output: PC1<Output<PushPull>>,
    ts: TemperatureSensor<pac::ADC1_2>,
    v_ref: VoltageInternalReference<pac::ADC1_2>,
    ahb: AHB,
    clocks: Clocks,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use cortex_m::asm;
    use defmt::{self, assert, assert_eq, unwrap};
    use testsuite::GenericPair;

    #[init]
    fn init() -> State {
        let dp = unwrap!(pac::Peripherals::take());

        // This is a workaround, so that the debugger will not disconnect imidiatly on asm::wfi();
        // https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
        dp.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        dp.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(8.MHz())
            .use_hse(8.MHz())
            .use_pll()
            .freeze(&mut flash.acr);
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

        // Slow down ADC
        unsafe {
            (*pac::RCC::ptr())
                .cfgr2
                .modify(|_, w| w.adc12pres().variant(pac::rcc::cfgr2::ADC12PRES_A::Div64))
        };
        dp.ADC1_2
            .ccr
            .modify(|_, w| w.ckmode().variant(pac::adc1_2::ccr::CKMODE_A::Asynchronous));
        let mut common_adc = CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);

        defmt::info!(
            "{} Hz",
            defmt::Debug2Format(unsafe { &common_adc.peripheral().clock(&clocks) })
        );

        let mut adc_dp = (dp.ADC1, dp.ADC2);
        let ts = TemperatureSensor::new(&mut common_adc, &mut adc_dp);
        let v_ref = VoltageInternalReference::new(&mut common_adc, &mut adc_dp);

        let pair = GenericPair {
            0: gpioc.pc0.into_analog(&mut gpioc.moder, &mut gpioc.pupdr),
            1: gpioc
                .pc1
                .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper),
        };

        let adc = Adc::new(adc_dp.0, Config::default(), &clocks, &common_adc);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(adc.interrupt());
        }

        State {
            adc: Some(adc),
            adc2: Some(adc_dp.1),
            adc_common: Some(common_adc),
            analog: pair.0,
            output: pair.1,
            ts,
            v_ref,
            ahb: rcc.ahb,
            clocks,
        }
    }

    #[test]
    fn measure_pin_high_low(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());
        adc.set_sample_time(&state.analog, config::SampleTime::Cycles61C5);
        for _ in 0..10 {
            state.output.set_high();
            asm::delay(100);
            let adc_level: u16 = defmt::unwrap!(adc.read(&mut state.analog).ok());
            defmt::debug!("{}", adc_level);
            state.output.set_low();
            asm::delay(100);
            // Vref is 3V so output should reach the maximum.
            assert!((3500..4100).contains(&adc_level));
            let adc_level: u16 = defmt::unwrap!(adc.read(&mut state.analog).ok());
            defmt::debug!("{}", adc_level);
            // nearly zero (always zero can not be guaranteed)
            assert!(adc_level <= 500);
        }

        // put adc back in place
        state.adc.replace(adc);
    }

    #[test]
    fn default_is_hardware_default(state: &mut State) {
        let adc = defmt::unwrap!(state.adc2.take());

        let adc = Adc::new_disabled(adc);

        assert_eq!(adc.config(), Config::default());

        state.adc2.replace(adc.free());
    }

    #[test]
    fn free_and_reconfigure(state: &mut State) {
        let adc = defmt::unwrap!(state.adc.take());
        let common_adc = defmt::unwrap!(state.adc_common.take());
        defmt::debug!("Free");
        let adcs = (adc.free(), defmt::unwrap!(state.adc2.take()));

        let adc1_2 = common_adc.free(&adcs);

        defmt::debug!("Reconfigure");
        let new_common_adc = adc::CommonAdc::new(adc1_2, &state.clocks, &mut state.ahb);
        // Check that Enabled is also the default.
        let new_adc: Adc<_, adc::Enabled> =
            Adc::new(adcs.0, Config::default(), &state.clocks, &new_common_adc);

        defmt::assert_eq!(new_adc.config(), Config::default());

        defmt::debug!("Replace");
        // put adc back in place
        state.adc.replace(new_adc);
        state.adc2.replace(adcs.1);
        state.adc_common.replace(new_common_adc);
    }

    #[test]
    fn state_conversions(state: &mut State) {
        let adc = defmt::unwrap!(state.adc.take());

        // FIXME: Introducing these log prints fixes the test beeing stuck
        // Timing issue?
        defmt::trace!("Disable");
        let adc: adc::Adc<_, adc::Disabled> = adc.into_disabled();
        // defmt::trace!("Enable");
        let adc: adc::Adc<_, adc::Enabled> = adc.into_enabled();
        // defmt::trace!("Oneshot");
        let adc: adc::Adc<_, adc::OneShot> = adc.into_oneshot();
        // defmt::trace!("Enable");
        let adc: adc::Adc<_, adc::Enabled> = adc.into_enabled();

        state.adc.replace(adc);
    }

    #[test]
    fn test_continuous(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());

        adc.set_conversion_mode(config::ConversionMode::Continuous);
        assert_eq!(adc.conversion_mode(), config::ConversionMode::Continuous);

        adc.set_pin_sequence_position(config::Sequence::One, &mut state.ts);
        adc.set_pin_sequence_position(config::Sequence::Two, &mut state.v_ref);
        adc.set_pin_sequence_position(config::Sequence::Three, &mut state.analog);
        assert_eq!(adc.sequence_length(), config::Sequence::Three);

        // Slow down the sample time of the inputs
        adc.set_sample_time(&state.ts, config::SampleTime::Cycles601C5);
        adc.set_sample_time(&state.v_ref, config::SampleTime::Cycles601C5);
        adc.set_sample_time(&state.analog, config::SampleTime::Cycles601C5);
        assert_eq!(adc.sample_time(&state.ts), config::SampleTime::Cycles601C5);

        assert!(!adc.is_conversion_ongoing());

        // Ensure we have no pending events, which could trigger an interrupt.
        adc.clear_events();
        assert_eq!(0, adc.triggered_events().len());

        adc.enable_interrupt(Event::EndOfSequence);
        adc.enable_interrupt(Event::EndOfConversion);

        // Trigger the conversion manually
        adc.start_conversion();
        let mut samples: [u16; 9] = [0, 0, 0, 0, 0, 0, 0, 0, 0];
        for sample in samples.iter_mut() {
            // Wait for the next interrupt to read from the data register again.
            // Techincally a general interrupt does not ensure, that the DATA register has content
            // again, but we assume so, as an EOC Event should occur any time.
            asm::wfi();
            *sample = adc.data_register();
        }

        // Stop the continuous conversion and disable interrupts to ensure, that the ADC is
        // doing nothing anymore.
        adc.stop_conversion();
        adc.disable_interrupt(Event::EndOfSequence);
        adc.disable_interrupt(Event::EndOfConversion);

        // If we catched every data via interrupt, no overrun event should've occured.
        assert!(!adc.is_event_triggered(Event::Overrun));

        defmt::dbg!(&samples);
        assert_eq!(
            (3, 9),
            (
                COUNTER.0.load(Ordering::SeqCst),
                COUNTER.1.load(Ordering::SeqCst)
            )
        );

        // Reset the counter for next tests.
        COUNTER.0.store(0, Ordering::SeqCst);
        COUNTER.1.store(0, Ordering::SeqCst);

        // Reset ADC to default config.
        adc.set_config(Config::default());
        state.adc.replace(adc);
    }

    #[test]
    fn test_discontinuous(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());
        // Stop at every sample conversion.
        adc.set_conversion_mode(config::ConversionMode::Discontinuous(0));
        assert_eq!(
            adc.conversion_mode(),
            config::ConversionMode::Discontinuous(0)
        );

        adc.set_pin_sequence_position(config::Sequence::One, &mut state.ts);
        adc.set_pin_sequence_position(config::Sequence::Two, &mut state.v_ref);
        adc.set_pin_sequence_position(config::Sequence::Three, &mut state.analog);
        assert_eq!(adc.sequence_length(), config::Sequence::Three);

        // Ensure we have no pending events, which could trigger an interrupt.
        adc.clear_events();
        assert_eq!(0, adc.triggered_events().len());

        adc.enable_interrupt(Event::EndOfSequence);
        adc.enable_interrupt(Event::EndOfConversion);

        // Trigger each conversion manually
        for i in 1..=9 {
            adc.start_conversion();
            asm::wfi();
            let _ = adc.data_register();
            assert_eq!(
                (i / 3, i),
                (
                    COUNTER.0.load(Ordering::SeqCst),
                    COUNTER.1.load(Ordering::SeqCst)
                )
            );
            assert_eq!(0, adc.triggered_events().len());
            // Wait sometime to prove, that the conversion is not ongoing until next manual trigger
            asm::delay(10000);
        }

        adc.disable_interrupt(Event::EndOfSequence);
        adc.disable_interrupt(Event::EndOfConversion);

        // Reset the counter for next tests.
        COUNTER.0.store(0, Ordering::SeqCst);
        COUNTER.1.store(0, Ordering::SeqCst);

        adc.set_config(Config::default());
        state.adc.replace(adc);
    }

    #[test]
    fn test_single_conversion_mode(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());
        // Stop after the sequence is finished
        adc.set_conversion_mode(config::ConversionMode::Single);
        assert_eq!(adc.conversion_mode(), config::ConversionMode::Single);

        adc.set_pin_sequence_position(config::Sequence::One, &mut state.ts);
        adc.set_pin_sequence_position(config::Sequence::Two, &mut state.v_ref);
        adc.set_pin_sequence_position(config::Sequence::Three, &mut state.analog);
        assert_eq!(adc.sequence_length(), config::Sequence::Three);

        // Ensure we have no pending events, which could trigger an interrupt.
        adc.clear_events();
        assert_eq!(0, adc.triggered_events().len());

        adc.enable_interrupt(Event::EndOfSequence);
        adc.enable_interrupt(Event::EndOfConversion);

        // Trigger the conversion manually
        adc.start_conversion();
        for _ in 0..3 {
            asm::wfi();
            let _ = adc.data_register();
        }
        assert_eq!(
            (1, 3),
            (
                COUNTER.0.load(Ordering::SeqCst),
                COUNTER.1.load(Ordering::SeqCst)
            )
        );
        adc.disable_interrupt(Event::EndOfSequence);
        adc.disable_interrupt(Event::EndOfConversion);

        // Wait some time to prove no event is occuring anymore
        asm::delay(10000);
        assert_eq!(0, adc.triggered_events().len());

        // Reset the counter for next tests.
        COUNTER.0.store(0, Ordering::SeqCst);
        COUNTER.1.store(0, Ordering::SeqCst);

        adc.set_config(Config::default());
        state.adc.replace(adc);
    }

    // TODO(Sh3rm4n): Add external trigger tests, when timer situation has improved.
    // At least we need the Capture Compare Event.
    // #[test]
    // fn test_external_trigger(state: &mut State) {
    //     let mut adc = defmt::unwrap!(state.adc.take());
    //     adc.set_conversion_mode(config::ConversionMode::Single);
    //     adc.set_external_trigger(Some(config::ExternalTrigger::Exti11(config::TriggerMode::RisingEdge)));
    //     adc.set_pin_sequence_position(config::Sequence::One, &mut state.ts);
    //     adc.set_sequence_length(config::Sequence::One);
    //
    //     state.adc.replace(adc);
    // }

    // Check that a runnning conversion can be reliably stopped (as stop_conversion will only take
    // action, when a conversion is ongoing.)
    #[test]
    fn test_stop_conversion(state: &mut State) {
        let mut adc = defmt::unwrap!(state.adc.take());
        adc.set_conversion_mode(config::ConversionMode::Continuous);
        assert_eq!(adc.conversion_mode(), config::ConversionMode::Continuous);
        adc.start_conversion();
        while !adc.is_event_triggered(Event::Overrun) {}
        adc.stop_conversion();
        state.adc.replace(adc);
    }
}

static COUNTER: (AtomicU32, AtomicU32) = (AtomicU32::new(0), AtomicU32::new(0));

#[interrupt]
fn ADC1_2() {
    // SAFETY: Mutable access to the ADC1_2 peripheral, while knowing that
    // no-one can pre-empt the interrupt.
    let (eos, eoc) = unsafe {
        let isr = &(*pac::ADC1::ptr()).isr.read();
        (*pac::ADC1::ptr())
            .isr
            .modify(|_, w| w.eos().clear().eoc().clear());
        (isr.eos(), isr.eoc())
    };
    COUNTER.0.fetch_add(u32::from(eos.bit()), Ordering::Acquire);
    COUNTER.1.fetch_add(u32::from(eoc.bit()), Ordering::Release);
}
