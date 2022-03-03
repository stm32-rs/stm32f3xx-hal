#![no_std]
#![no_main]

//! Example usage for ADC on STM32F303

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use embedded_hal::adc::OneShot;
use stm32f3xx_hal::{
    adc,
    pac::{self, interrupt},
    prelude::*,
    timer,
};

/// That's a mouthful, so explain it:
///
/// 1. Wrap the Timer in a Mutex, so it can be safely shared between
///    main loop and interrupt or rather used in functions, which could theoretically
///    be preempted.
/// 2. Wrap the Timer in a RefCell to be able obtain a mutable reference to the Timer itself.
///    E.g. the interrupt can't take ownership of the timer, it is shared between main-loop and
///    interrupt context.
/// 3. Wrap the Timer in an Option, so that it can be "lazily initialized". Statics have to be
///    initialized with const values, which the timer itself is obviously not, but None is.
///
/// This could all be done just with a static mut && unsafe as
/// the usecase it pretty clear, but this is to show the definitely safe
/// alternative.
static TIMER: Mutex<RefCell<Option<timer::Timer<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Get peripherals, clocks and freeze them
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    // This is a workaround, so that the debugger will not disconnect imidiatly on asm::wfi();
    // https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });

    // Create a Common ADC instance, which is shared between ADC1 and ADC2 in this case,
    // and for example is in control of the clock of both of these peripherals.
    let mut adc_common = adc::CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);

    // We have to pack these peripherals in a tuple, so that `TemperatureSensor` can
    // get a mutable reference to both of these as a singular argument.
    //
    // This is needed, as both ADC1 and ADC2 have to be of to enable the `TemperatureSensor`.
    let mut tuple = (dp.ADC1, dp.ADC2);
    let mut ts = adc::TemperatureSensor::new(&mut adc_common, &mut tuple);

    // Set up ADC1
    let mut adc = adc::Adc::new(
        tuple.0, // The ADC we are going to control
        adc::config::Config::default(),
        // The following is only needed to make sure the clock signal for the ADC is set up
        // correctly.
        &clocks,
        &adc_common,
    )
    // Convert the ADC into `OneShot` mode.
    //
    // This is not necessary, as the normal ADC does also implement the `OneShot` trait, which can
    // call `read`. But this conversion implies less overhead for the trait implementation. The
    // consequence is, that in this mode nothing can be configured manually.
    .into_oneshot();

    // You can also create an ADC, which is initially disabled and uncalibrated and calibrate
    // it later:
    let mut adc2 = adc::Adc::new_disabled(tuple.1);
    adc2.calibrate(&clocks, &adc_common);
    adc2.set_config(adc::config::Config::default());
    let _ = adc2.into_enabled();

    // Set up pin PA0 as analog pin.
    // This pin is connected to the user button on the stm32f3discovery board.
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut analog_pin = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    let mut timer = timer::Timer::new(dp.TIM2, clocks, &mut rcc.apb1);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(timer.interrupt());
    }
    timer.enable_interrupt(timer::Event::Update);
    // Start a timer which fires regularly to wake up from `asm::wfi`
    timer.start(500.milliseconds());
    // Put the timer in the global context.
    cortex_m::interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
    });

    // Be aware that the values in the table below depend on the input of VREF.
    // To have a stable VREF input, put a condensator and a volt limiting diode in front of it.
    //
    // Also know that integer division and the ADC hardware unit always round down.
    // To make up for those errors, see this forum entry:
    // [https://forum.allaboutcircuits.com/threads/why-adc-1024-is-correct-and-adc-1023-is-just-plain-wrong.80018/]
    defmt::info!("
    The ADC has a 12 bit resolution, i.e. if your reference Value is 3V:
        approx. ADC value | approx. volt value
        ==================+===================
                        0 |        0 mV
                     2048 |     1500 mV
                     4095 |     3000 mV

    If you are using a STM32F3Discovery, PA0 is connected to the User Button.
    Pressing it should connect the user Button to to HIGH and the value should change from 0 to 4095.
    ");

    loop {
        let adc_data: u16 = adc.read(&mut analog_pin).unwrap();
        defmt::trace!("PA0 reads {}", adc_data);
        let adc_data: u16 = adc.read(&mut ts).unwrap();
        defmt::trace!("TemperatureSensor reads {}", adc_data);
        asm::wfi();
    }
}

#[interrupt]
fn TIM2() {
    // Just handle the pending interrupt event.
    cortex_m::interrupt::free(|cs| {
        TIMER
            // Unlock resource for use in critical section
            .borrow(cs)
            // Get a mutable reference from the RefCell
            .borrow_mut()
            // Make the inner Option<T> -> Option<&mut T>
            .as_mut()
            // Unwrap the option, we know, that it has Some()!
            .unwrap()
            // Finally operate on the timer itself.
            .clear_event(timer::Event::Update);
    })
}
