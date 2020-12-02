//! Example of using a number of timer channels in PWM mode.
//! Target board: STM32F3DISCOVERY
#![no_std]
#![no_main]

use panic_semihosting as _;

use stm32f3xx_hal as hal;

use cortex_m::asm;
use cortex_m_rt::entry;

//use cortex_m_semihosting::hprintln;
use hal::hal::PwmPin;

use hal::flash::FlashExt;
use hal::gpio::GpioExt;
use hal::pac;
use hal::pwm::{tim16, tim2, tim3, tim8};
use hal::rcc::RccExt;
use hal::time::U32Ext;

#[entry]
fn main() -> ! {
    // Get our peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Configure our clocks
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(16.mhz()).freeze(&mut flash.acr);

    // Prep the pins we need in their correct alternate function
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let pa4 = gpioa.pa4.into_af2(&mut gpioa.moder, &mut gpioa.afrl);
    let pa6 = gpioa.pa6.into_af2(&mut gpioa.moder, &mut gpioa.afrl);
    let pa7 = gpioa.pa7.into_af2(&mut gpioa.moder, &mut gpioa.afrl);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let pb0 = gpiob.pb0.into_af2(&mut gpiob.moder, &mut gpiob.afrl);
    let pb1 = gpiob.pb1.into_af2(&mut gpiob.moder, &mut gpiob.afrl);
    let pb4 = gpiob.pb4.into_af2(&mut gpiob.moder, &mut gpiob.afrl);
    let pb5 = gpiob.pb5.into_af2(&mut gpiob.moder, &mut gpiob.afrl);
    let pb8 = gpiob.pb8.into_af1(&mut gpiob.moder, &mut gpiob.afrh);
    let pb10 = gpiob.pb10.into_af1(&mut gpiob.moder, &mut gpiob.afrh);

    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
    let pc10 = gpioc.pc10.into_af4(&mut gpioc.moder, &mut gpioc.afrh);

    // TIM3
    //
    // A four channel general purpose timer that's broadly available
    let tim3_channels = tim3(
        dp.TIM3,
        1280,    // resolution of duty cycle
        50.hz(), // frequency of period
        &clocks, // To get the timer's clock speed
    );

    // Channels without pins cannot be enabled, so we can't forget to
    // connect a pin.
    //
    // DOES NOT COMPILE
    // tim3_channels.0.enable();

    // Each channel can be used with a different duty cycle and have many pins
    let mut tim3_ch1 = tim3_channels.0.output_to_pa6(pa6).output_to_pb4(pb4);
    tim3_ch1.set_duty(tim3_ch1.get_max_duty() / 20); // 5% duty cyle
    tim3_ch1.enable();

    let mut tim3_ch2 = tim3_channels
        .1
        .output_to_pa4(pa4)
        .output_to_pa7(pa7)
        .output_to_pb5(pb5);
    tim3_ch2.set_duty(tim3_ch2.get_max_duty() / 40 * 3); // 7.5% duty cyle
    tim3_ch2.enable();

    let mut tim3_ch3 = tim3_channels.2.output_to_pb0(pb0);
    tim3_ch3.set_duty(tim3_ch3.get_max_duty() / 50 * 3); // 6% duty cyle
    tim3_ch3.enable();

    let mut tim3_ch4 = tim3_channels.3.output_to_pb1(pb1);
    tim3_ch4.set_duty(tim3_ch4.get_max_duty() / 10); // 10% duty cyle
    tim3_ch4.enable();

    // We can only add valid pins, so we can't do this:
    //
    // DOES NOT COMPILE
    // tim3_ch1.output_to_pb8(pb8);

    // The pins that we've used are given away so they can't be
    // accidentaly modified.  This line would "disconnect" our pin
    // from the channel.
    //
    // DOES NOT COMPILE
    // pb0.into_af15(&mut gpiob.moder, &mut gpiob.afrl);

    // TIM2
    //
    // A 32-bit timer, so we can set a larger resolution
    let tim2_channels = tim2(
        dp.TIM2,
        160000,  // resolution of duty cycle
        50.hz(), // frequency of period
        &clocks, // To get the timer's clock speed
    );

    let mut tim2_ch3 = tim2_channels.2.output_to_pb10(pb10);
    tim2_ch3.set_duty(tim2_ch3.get_max_duty() / 20); // 5% duty cyle
    tim2_ch3.enable();

    // TIM16
    //
    // A single channel timer, so it doesn't return a tuple.  We can
    // just use it directly
    let mut tim16_ch1 = tim16(
        dp.TIM16,
        1280,    // resolution of duty cycle
        50.hz(), // frequency of period
        &clocks, // To get the timer's clock speed
    )
    .output_to_pb8(pb8);
    tim16_ch1.set_duty(tim16_ch1.get_max_duty() / 20); // 5% duty cyle
    tim16_ch1.enable();

    // TIM8
    //
    // An advanced timer with complementary outputs, so we can output
    // to complementary pins (works just like standard pins)
    let tim8_channels = tim8(
        dp.TIM8,
        1280,    // resolution of duty cycle
        50.hz(), // frequency of period
        &clocks, // To get the timer's clock speed
    );

    let mut tim8_ch1 = tim8_channels.0.output_to_pc10(pc10);
    tim8_ch1.set_duty(tim8_ch1.get_max_duty() / 10); // 10% duty cyle
    tim8_ch1.enable();

    // Once we select PB3, we can only use complementary pins (such as
    // PC10).  These are pins with alternate functions with an 'N' at
    // the end of the channel (such as TIM8_CH1N) in the reference
    // manual.  If we had selected a non-complementary pin first, we
    // would not be able to use PB3 or PC10 (or PA7 which is aready in
    // use).
    //
    // DOES NOT COMPILE
    // tim8_ch1.output_to_pc6(gpioc.pc6.into_af4(&mut gpioc.moder, &mut gpioc.afrl));

    loop {
        asm::wfi();
    }
}
