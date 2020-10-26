//! Timers

use crate::hal::timer::{CountDown, Periodic};
#[cfg(any(
    feature = "stm32f301",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f318",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::pac::TIM1;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
use crate::pac::TIM20;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::pac::TIM4;
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::pac::TIM8;
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
use crate::pac::{TIM12, TIM13, TIM14, TIM18, TIM19, TIM5};
// use crate::pac::{TIM15, TIM16, TIM17, TIM2, TIM6};
use crate::pac::{TIM16, TIM17, TIM2, TIM6};
#[cfg(any(
    feature = "stm32f303",
    feature = "stm32f328",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f398"
))]
use crate::pac::{TIM3, TIM7};

use cast::{u16, u32};
use void::Void;

use crate::rcc::{Clocks, APB1, APB2};
use crate::time::Hertz;

/// Associated clocks with timers
pub trait PclkSrc {
    fn get_clk(clocks: &Clocks) -> Hertz;
}

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    Update,
}

/// Output alignment
#[derive(Clone, Copy, Debug)]
pub enum Alignment {
    Edge,
    Center1,
    Center2,
    Center3,
}

#[derive(Clone, Copy, Debug)]
pub enum Channel {
    One,
    Two,
    Three,
    Four,
}

/// Capture/Compare selection.
/// This bit-field defines the direction of the channel (input/output) as well as the used input.
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum CaptureCompare {
    Output = 0b00,
    InputTi1 = 0b01,
    InputTi2 = 0b10,
    InputTrc = 0b11,
}

/// Capture/Compare output polarity. Defaults to `ActiveHigh` in hardware.
#[derive(Clone, Copy, Debug)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

impl Polarity {
    /// For use with `set_bit()`.
    fn bit(&self) -> bool {
        match self {
            Self::ActiveHigh => false,
            Self::ActiveLow => true,
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
/// See F303 ref man, section 21.4.7.
/// These bits define the behavior of the output reference signal OC1REF from which OC1 and
/// OC1N are derived. OC1REF is active high whereas OC1 and OC1N active level depends
/// on CC1P and CC1NP bits.
/// 0000: Frozen - The comparison between the output compare register TIMx_CCR1 and the
/// counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing
/// base).
/// 0001: Set channel 1 to active level on match. OC1REF signal is forced high when the
/// counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
/// 0010: Set channel 1 to inactive level on match. OC1REF signal is forced low when the
/// counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
/// 0011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.
/// 0100: Force inactive level - OC1REF is forced low.
/// 0101: Force active level - OC1REF is forced high.
/// 0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
/// else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0) as long as
/// TIMx_CNT>TIMx_CCR1 else active (OC1REF=1).
/// 0111: PWM mode 2 - In upcounting, channel 1 is inactive as long as
/// TIMx_CNT<TIMx_CCR1 else active. In downcounting, channel 1 is active as long as
/// TIMx_CNT>TIMx_CCR1 else inactive.
/// 1000: Retriggerable OPM mode 1 - In up-counting mode, the channel is active until a trigger
/// event is detected (on TRGI signal). Then, a comparison is performed as in PWM mode 1
/// and the channels becomes inactive again at the next update. In down-counting mode, the
/// channel is inactive until a trigger event is detected (on TRGI signal). Then, a comparison is
/// performed as in PWM mode 1 and the channels becomes inactive again at the next update.
/// 1001: Retriggerable OPM mode 2 - In up-counting mode, the channel is inactive until a
/// trigger event is detected (on TRGI signal). Then, a comparison is performed as in PWM
/// mode 2 and the channels becomes inactive again at the next update. In down-counting
/// mode, the channel is active until a trigger event is detected (on TRGI signal). Then, a
/// comparison is performed as in PWM mode 1 and the channels becomes active again at the
/// next update.
/// 1010: Reserved,
/// 1011: Reserved,
/// 1100: Combined PWM mode 1 - OC1REF has the same behavior as in PWM mode 1.
/// OC1REFC is the logical OR between OC1REF and OC2REF.
/// 1101: Combined PWM mode 2 - OC1REF has the same behavior as in PWM mode 2.
/// OC1REFC is the logical AND between OC1REF and OC2REF.
/// 1110: Asymmetric PWM mode 1 - OC1REF has the same behavior as in PWM mode 1.
/// OC1REFC outputs OC1REF when the counter is counting up, OC2REF when it is counting
/// down.
/// 1111: Asymmetric PWM mode 2 - OC1REF has the same behavior as in PWM mode 2.
/// OC1REFC outputs OC1REF when the counter is counting up, OC2REF when it is counting
/// down
pub enum OutputCompare {
    Frozen = 0b0000,
    Active = 0b0001,
    Inactive = 0b0010,
    ForceInactive = 0b0100,
    ForceActive = 0b0101,
    Pwm1 = 0b0110,
    Pwm2 = 0b0111,
    RetriggerableOpmMode1 = 0b1000,
    RetriggerableOpmMode2 = 0b1001,
    CombinedPwm1 = 0b1100,
    CombinedPwm2 = 0b1101,
    AsymmetricPwm1 = 0b1110,
    AsymmetricPwm2 = 0b1111,
}

macro_rules! basic_timer {
    ($({
        $TIMX:ident: ($tim:ident, $timXen:ident, $timXrst:ident),
        $APB:ident: ($apb:ident, $pclkX:ident),
        $res:ident,
    },)+) => {
        $(
            impl PclkSrc for $TIMX {
                fn get_clk(clocks: &Clocks) -> Hertz {
                    clocks.$pclkX()
                }
            }

            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.stop();

                    let frequency = timeout.into().0;
                    let timer_clock = $TIMX::get_clk(&self.clocks);
                    let ticks = timer_clock.0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    // NOTE(write): uses all bits in this register.
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    // TODO (sh3rm4n)
                    // self.tim.arr.write(|w| { w.arr().bits(arr) });
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register.
                    self.tim.egr.write(|w| w.ug().update());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().enabled());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
                        Ok(())
                    }
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIMX, timeout: T, clocks: Clocks, $apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    $apb.enr().modify(|_, w| w.$timXen().enabled());
                    $apb.rstr().modify(|_, w| w.$timXrst().reset());
                    $apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer { clocks, tim };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().enabled()),
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().disabled()),
                    }
                }

                /// Stops the timer
                pub fn stop(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().disabled());
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear());
                }

                /// Releases the TIM peripheral
                pub fn release(mut self) -> $TIMX {
                    self.stop();
                    self.tim
                }

                /// Set the value of the auto-reload resolution. Use a lower value to increase
                /// precision. If you wish for a precise tick speed, multiply the system clock
                /// speed by the desired frequency, then round to the nearest integer.
                pub fn set_resolution(&mut self, word: $res) {
                    self.tim.arr.write(|w| w.arr().bits(word) );
                }

                /// Return the integer associated with the maximum duty period.
                /// todo: Duty could be u16 for low-precision timers.
                pub fn get_max_duty(&self) -> $res {
                    self.tim.arr.read().arr().bits()
                }
            }
        )+
    }
}

macro_rules! advanced_timer {
    ($({
        $TIMX:ident: ($tim:ident, $timXen:ident, $timXrst:ident),
        $APB:ident: ($apb:ident, $pclkX:ident),
        $res:ident,
    },)+) => {
        $(
            impl PclkSrc for $TIMX {
                fn get_clk(clocks: &Clocks) -> Hertz {
                    clocks.$pclkX()
                }
            }

            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.stop();

                    let frequency = timeout.into().0;
                    let timer_clock = $TIMX::get_clk(&self.clocks);
                    let ticks = timer_clock.0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    // NOTE(write): uses all bits in this register.
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    // TODO (sh3rm4n)
                    // self.tim.arr.write(|w| { w.arr().bits(arr) });
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register.
                    self.tim.egr.write(|w| w.ug().update());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().enabled());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
                        Ok(())
                    }
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIMX, timeout: T, clocks: Clocks, $apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    $apb.enr().modify(|_, w| w.$timXen().enabled());
                    $apb.rstr().modify(|_, w| w.$timXrst().reset());
                    $apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer { clocks, tim };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().enabled()),
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().disabled()),
                    }
                }

                /// Stops the timer
                pub fn stop(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().disabled());
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear());
                }

                /// Releases the TIM peripheral
                pub fn release(mut self) -> $TIMX {
                    self.stop();
                    self.tim
                }

                /// Set timer alignment to Edge, or one of 3 center modes.
                /// STM32F303 ref man, section 21.4.1:
                /// Bits 6:5 CMS: Center-aligned mode selection
                /// 00: Edge-aligned mode. The counter counts up or down depending on the direction bit
                /// (DIR).
                /// 01: Center-aligned mode 1. The counter counts up and down alternatively. Output compare
                /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                /// only when the counter is counting down.
                /// 10: Center-aligned mode 2. The counter counts up and down alternatively. Output compare
                /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                /// only when the counter is counting up.
                /// 11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
                /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                /// both when the counter is counting up or down.
                pub fn set_alignment(&mut self, alignment: Alignment) {
                    let word = match alignment {
                        Alignment::Edge => 0b00,
                        Alignment::Center1 => 0b01,
                        Alignment::Center2 => 0b10,
                        Alignment::Center3 => 0b11,
                    };
                    self.tim.cr1.modify(|_, w| w.cms().bits(word));
                }

                /// Set the value of the auto-reload resolution. Use a lower value to increase
                /// precision. If you wish for a precise tick speed, multiply the system clock
                /// speed by the desired frequency, then round to the nearest integer.
                pub fn set_resolution(&mut self, word: $res) {
                    self.tim.arr.write(|w| w.arr().bits(word));
                }

                /// Return the integer associated with the maximum duty period.
                /// todo: Duty could be u16 for low-precision timers.
                pub fn get_max_duty(&self) -> $res {
                    self.tim.arr.read().arr().bits()
                }

                /// Set output polarity. See docs on the `Polarity` enum.
                pub fn set_polarity(&mut self, channel: Channel, polarity: Polarity) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1p().bit(polarity.bit())),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2p().bit(polarity.bit())),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3p().bit(polarity.bit())),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4p().bit(polarity.bit())),
                    }
                }

                /// Disables the timer.
                pub fn disable(&mut self, channel: Channel) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1e().clear_bit()),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2e().clear_bit()),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3e().clear_bit()),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4e().clear_bit()),
                    }
                }

                /// Enables the timer.
                pub fn enable(&mut self, channel: Channel) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1e().set_bit()),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2e().set_bit()),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3e().set_bit()),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4e().set_bit()),
                    }
                }
            }
        )+
    }
}

/// General purpose timers. F303 ref man, Chapter 21. For Tim 2, 3, and 4.
macro_rules! gp_timer {
    ($({
        $TIMX:ident: ($tim:ident, $timXen:ident, $timXrst:ident),
        $APB:ident: ($apb:ident, $pclkX:ident),
        $res:ident,
    },)+) => {
        $(
            impl PclkSrc for $TIMX {
                fn get_clk(clocks: &Clocks) -> Hertz {
                    clocks.$pclkX()
                }
            }

            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.stop();

                    let frequency = timeout.into().0;
                    let timer_clock = $TIMX::get_clk(&self.clocks);
                    let ticks = timer_clock.0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                        / frequency;
                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                    // NOTE(write): uses all bits in this register.
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = u16(ticks / u32(psc + 1)).unwrap();

                    // TODO (sh3rm4n)
                    // self.tim.arr.write(|w| { w.arr().bits(arr) });
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                    // Trigger an update event to load the prescaler value to the clock
                    // NOTE(write): uses all bits in this register.
                    self.tim.egr.write(|w| w.ug().update());
                    // The above line raises an update event which will indicate
                    // that the timer is already finished. Since this is not the case,
                    // it should be cleared
                    self.clear_update_interrupt_flag();

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().enabled());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_update_interrupt_flag();
                        Ok(())
                    }
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIMX, timeout: T, clocks: Clocks, $apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    $apb.enr().modify(|_, w| w.$timXen().enabled());
                    $apb.rstr().modify(|_, w| w.$timXrst().reset());
                    $apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = Timer { clocks, tim };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().enabled()),
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Update => self.tim.dier.write(|w| w.uie().disabled()),
                    }
                }

                /// Stops the timer
                pub fn stop(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().disabled());
                }

                /// Clears Update Interrupt Flag
                pub fn clear_update_interrupt_flag(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear());
                }

                /// Releases the TIM peripheral
                pub fn release(mut self) -> $TIMX {
                    self.stop();
                    self.tim
                }

                /// Set timer alignment to Edge, or one of 3 center modes.
                /// STM32F303 ref man, section 21.4.1:
                /// Bits 6:5 CMS: Center-aligned mode selection
                /// 00: Edge-aligned mode. The counter counts up or down depending on the direction bit
                /// (DIR).
                /// 01: Center-aligned mode 1. The counter counts up and down alternatively. Output compare
                /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                /// only when the counter is counting down.
                /// 10: Center-aligned mode 2. The counter counts up and down alternatively. Output compare
                /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                /// only when the counter is counting up.
                /// 11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
                /// interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
                /// both when the counter is counting up or down.
                pub fn set_alignment(&mut self, alignment: Alignment) {
                    let word = match alignment {
                        Alignment::Edge => 0b00,
                        Alignment::Center1 => 0b01,
                        Alignment::Center2 => 0b10,
                        Alignment::Center3 => 0b11,
                    };
                    self.tim.cr1.modify(|_, w| w.cms().bits(word));
                }

                /// Set the value of the auto-reload resolution. Use a lower value to increase
                /// precision. If you wish for a precise tick speed, multiply the system clock
                /// speed by the desired frequency, then round to the nearest integer.
                pub fn set_resolution(&mut self, word: $res) {
                    self.tim.arr.write(|w| w.arr().bits(word.into()));
                }

                /// Return the integer associated with the maximum duty period.
                /// todo: Duty could be u16 for low-precision timers.
                pub fn get_max_duty(&self) -> $res {
                    self.tim.arr.read().arr().bits() as $res
                }

                /// Set output polarity. See docs on the `Polarity` enum.
                pub fn set_polarity(&mut self, channel: Channel, polarity: Polarity) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1p().bit(polarity.bit())),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2p().bit(polarity.bit())),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3p().bit(polarity.bit())),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4p().bit(polarity.bit())),
                    }
                }

                /// Set complementary output polarity. See docs on the `Polarity` enum.
                pub fn set_complementary_polarity(&mut self, channel: Channel, polarity: Polarity) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1np().bit(polarity.bit())),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2np().bit(polarity.bit())),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3np().bit(polarity.bit())),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4np().bit(polarity.bit())),
                    }
                }

                /// Disables the timer.
                pub fn disable(&mut self, channel: Channel) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1e().clear_bit()),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2e().clear_bit()),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3e().clear_bit()),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4e().clear_bit()),
                    }
                }

                /// Enables the timer.
                pub fn enable(&mut self, channel: Channel) {
                    match channel {
                        Channel::One => self.tim.ccer.modify(|_, w| w.cc1e().set_bit()),
                        Channel::Two => self.tim.ccer.modify(|_, w| w.cc2e().set_bit()),
                        Channel::Three => self.tim.ccer.modify(|_, w| w.cc3e().set_bit()),
                        Channel::Four => self.tim.ccer.modify(|_, w| w.cc4e().set_bit()),
                    }
                }

                /// Set Capture Compare Mode. See docs on the `CaptureCompare` enum.
                pub fn set_capture_compare(&mut self, channel: Channel, mode: CaptureCompare) {
                    match channel {
                        // Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER)
                        Channel::One => self.tim.ccmr1_output().modify( unsafe { |_, w| w.cc1s().bits(mode as u8)} ),
                        Channel::Two => self.tim.ccmr1_output().modify(unsafe {|_, w| w.cc2s().bits(mode as u8)}),
                        Channel::Three => self.tim.ccmr2_output().modify(unsafe {|_, w| w.cc3s().bits(mode as u8)}),
                        Channel::Four => self.tim.ccmr2_output().modify(unsafe {|_, w| w.cc4s().bits(mode as u8)}),
                    }
                }

                /// Set Output Compare Mode. See docs on the `OutputCompare` enum.
                pub fn set_output_compare(&mut self, channel: Channel, mode: OutputCompare) {
                    match channel {
                        Channel::One => self.tim.ccmr1_output().modify(|_, w| w.oc1m().bits(mode as u8)),
                        Channel::Two => self.tim.ccmr1_output().modify(|_, w| w.oc2m().bits(mode as u8)),
                        Channel::Three => self.tim.ccmr2_output().modify(|_, w| w.oc3m().bits(mode as u8)),
                        Channel::Four => self.tim.ccmr2_output().modify(|_, w| w.oc4m().bits(mode as u8)),
                    }
                }

                /// Set preload mode.
                /// OC1PE: Output Compare 1 preload enable
                /// 0: Preload register on TIMx_CCR1 disabled. TIMx_CCR1 can be written at anytime, the
                /// new value is taken in account immediately.
                /// 1: Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload
                /// register. TIMx_CCR1 preload value is loaded in the active register at each update event.
                /// Note: 1: These bits can not be modified as long as LOCK level 3 has been programmed
                /// (LOCK bits in TIMx_BDTR register) and CC1S=’00’ (the channel is configured in
                /// output).
                /// 2: The PWM mode can be used without validating the preload register only in one
                /// pulse mode (OPM bit set in TIMx_CR1 register). Else the behavior is not guaranteed.
                pub fn set_preload(&mut self, channel: Channel, value: bool) {
                    match channel {
                        Channel::One => self.tim.ccmr1_output().modify(|_, w| w.oc1pe().bit(value)),
                        Channel::Two => self.tim.ccmr1_output().modify(|_, w| w.oc2pe().bit(value)),
                        Channel::Three => self.tim.ccmr2_output().modify(|_, w| w.oc3pe().bit(value)),
                        Channel::Four => self.tim.ccmr2_output().modify(|_, w| w.oc4pe().bit(value)),
                    }
                }

                /// Return the set duty period for a given channel. Divide by `get_max_duty()`
                /// to find the portion of the duty cycle used.
                pub fn get_duty(&self, channel: Channel) -> $res {
                    match channel {
                        Channel::One => self.tim.ccr1.read().ccr().bits() as $res,
                        Channel::Two => self.tim.ccr2.read().ccr().bits() as $res,
                        Channel::Three => self.tim.ccr3.read().ccr().bits() as $res,
                        Channel::Four => self.tim.ccr4.read().ccr().bits() as $res,
                    }
                }

                /// Set the duty cycle, as a portion of `get_max_duty()`.
                pub fn set_duty(&mut self, channel: Channel, duty: $res) {
                    match channel {
                        Channel::One => self.tim.ccr1.write(|w| w.ccr().bits(duty as u32)),
                        Channel::Two => self.tim.ccr2.write(|w| w.ccr().bits(duty as u32)),
                        Channel::Three => self.tim.ccr3.write(|w| w.ccr().bits(duty as u32)),
                        Channel::Four => self.tim.ccr4.write(|w| w.ccr().bits(duty as u32)),
                    }
                }
            }
        )+
    }


    }
macro_rules! gp_timer2 {
        ($({
            $TIMX:ident: ($tim:ident, $timXen:ident, $timXrst:ident),
            $APB:ident: ($apb:ident, $pclkX:ident),
            $res:ident,
        },)+) => {
            $(
                impl PclkSrc for $TIMX {
                    fn get_clk(clocks: &Clocks) -> Hertz {
                        clocks.$pclkX()
                    }
                }

                impl Periodic for Timer<$TIMX> {}

                impl CountDown for Timer<$TIMX> {
                    type Time = Hertz;

                    fn start<T>(&mut self, timeout: T)
                    where
                        T: Into<Hertz>,
                    {
                        self.stop();

                        let frequency = timeout.into().0;
                        let timer_clock = $TIMX::get_clk(&self.clocks);
                        let ticks = timer_clock.0 * if self.clocks.ppre1() == 1 { 1 } else { 2 }
                            / frequency;
                        let psc = u16((ticks - 1) / (1 << 16)).unwrap();

                        // NOTE(write): uses all bits in this register.
                        self.tim.psc.write(|w| w.psc().bits(psc));

                        let arr = u16(ticks / u32(psc + 1)).unwrap();

                        // TODO (sh3rm4n)
                        // self.tim.arr.write(|w| { w.arr().bits(arr) });
                        self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });

                        // Trigger an update event to load the prescaler value to the clock
                        // NOTE(write): uses all bits in this register.
                        self.tim.egr.write(|w| w.ug().update());
                        // The above line raises an update event which will indicate
                        // that the timer is already finished. Since this is not the case,
                        // it should be cleared
                        self.clear_update_interrupt_flag();

                        // start counter
                        self.tim.cr1.modify(|_, w| w.cen().enabled());
                    }

                    fn wait(&mut self) -> nb::Result<(), Void> {
                        if self.tim.sr.read().uif().is_clear() {
                            Err(nb::Error::WouldBlock)
                        } else {
                            self.clear_update_interrupt_flag();
                            Ok(())
                        }
                    }
                }

                impl Timer<$TIMX> {
                    /// Configures a TIM peripheral as a periodic count down timer
                    pub fn $tim<T>(tim: $TIMX, timeout: T, clocks: Clocks, $apb: &mut $APB) -> Self
                    where
                        T: Into<Hertz>,
                    {
                        // enable and reset peripheral to a clean slate state
                        $apb.enr().modify(|_, w| w.$timXen().enabled());
                        $apb.rstr().modify(|_, w| w.$timXrst().reset());
                        $apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                        let mut timer = Timer { clocks, tim };
                        timer.start(timeout);

                        timer
                    }

                    /// Starts listening for an `event`
                    pub fn listen(&mut self, event: Event) {
                        match event {
                            Event::Update => self.tim.dier.write(|w| w.uie().enabled()),
                        }
                    }

                    /// Stops listening for an `event`
                    pub fn unlisten(&mut self, event: Event) {
                        match event {
                            Event::Update => self.tim.dier.write(|w| w.uie().disabled()),
                        }
                    }

                    /// Stops the timer
                    pub fn stop(&mut self) {
                        self.tim.cr1.modify(|_, w| w.cen().disabled());
                    }

                    /// Clears Update Interrupt Flag
                    pub fn clear_update_interrupt_flag(&mut self) {
                        self.tim.sr.modify(|_, w| w.uif().clear());
                    }

                    /// Releases the TIM peripheral
                    pub fn release(mut self) -> $TIMX {
                        self.stop();
                        self.tim
                    }

                /// Set the value of the auto-reload resolution. Use a lower value to increase
                /// precision. If you wish for a precise tick speed, multiply the system clock
                /// speed by the desired frequency, then round to the nearest integer.
                pub fn set_resolution(&mut self, word: $res) {
                    self.tim.arr.write(|w| unsafe { w.arr().bits(word) } );
                }

                /// Return the integer associated with the maximum duty period.
                /// todo: Duty could be u16 for low-precision timers.
                pub fn get_max_duty(&self) -> $res {
                    self.tim.arr.read().arr().bits()
                }

                /// Set output polarity. See docs on the `Polarity` enum.
                pub fn set_polarity(&mut self, polarity: Polarity) {
                    self.tim.ccer.modify(|_, w| w.cc1p().bit(polarity.bit()))
                }

                /// Set complementary output polarity. See docs on the `Polarity` enum.
                pub fn set_complementary_polarity(&mut self, polarity: Polarity) {
                    self.tim.ccer.modify(|_, w| w.cc1np().bit(polarity.bit()))
                }

                /// Disables the timer.
                pub fn disable(&mut self) {
                    self.tim.ccer.modify(|_, w| w.cc1e().clear_bit())
                }

                /// Enables the timer.
                pub fn enable(&mut self) {
                    self.tim.ccer.modify(|_, w| w.cc1e().set_bit())
                }
            }
            )+
        }
}

// Basic timers: Tim6, 7. Count up only, no CC channels.
// General purpose timers: Tim2, 3, 4. Count up and down, and have 4 CC channels
// General purpose timers: Tim15, 16, 17. Count up only, and have 2, 1, 1 CC channels
// Advanced-control timers: Tim1, 8, 20. Count up and down, and have 4 CC channels.
// Tim 5, 13, 14, 18, 19? What are these rare timers? Currently unhandled

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
basic_timer! {
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(feature = "stm32f302")]
basic_timer! {
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1,pclk1),
        u16,
    },
}

#[cfg(feature = "stm32f303")]
basic_timer! {
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(feature = "stm32f334")]
basic_timer! {
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
basic_timer! {
    // todo TIM5??
    // {
    //     TIM5: (tim5, tim5en, tim5rst),
    //     APB1: (apb1, pclk1),
    // },
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
        u16,
    },
    // todo: 18/19??

    // {
    //     TIM18: (tim18, tim18en, tim18rst),
    //     APB1: (apb1, pclk1),
    // },
    // {
    //     TIM19: (tim19, tim19en, tim19rst),
    //     APB2: (apb2, pclk2),
    // },
}

#[cfg(any(feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
basic_timer! {
    {
        TIM6: (tim6, tim6en, tim6rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM7: (tim7, tim7en, tim7rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
advanced_timer! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(feature = "stm32f302")]
advanced_timer! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(feature = "stm32f303")]
advanced_timer! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM8: (tim8, tim8en, tim8rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM20: (tim20, tim20en, tim20rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(feature = "stm32f334")]
advanced_timer! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
advanced_timer! {
    // { todo
    //     TIM5: (tim5, tim5en, tim5rst),
    //     APB1: (apb1, pclk1),
    // },
    // {
    //     TIM12: (tim12, tim12en, tim12rst),
    //     APB1: (apb1, pclk1),
    // },
    // {
    //     TIM13: (tim13, tim13en, tim13rst),
    //     APB1: (apb1, pclk1),
    // },
    // {
    //     TIM14: (tim14, tim14en, tim14rst),
    //     APB1: (apb1, pclk1),
    // },
    // {
    //     TIM18: (tim18, tim18en, tim18rst),
    //     APB1: (apb1, pclk1),
    // },
    // {
    //     TIM19: (tim19, tim19en, tim19rst),
    //     APB2: (apb2, pclk2),
    // },
}

#[cfg(any(feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
advanced_timer! {
    {
        TIM1: (tim1, tim1en, tim1rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM8: (tim8, tim8en, tim8rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM20: (tim20, tim20en, tim20rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
gp_timer! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(feature = "stm32f302")]
gp_timer! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(feature = "stm32f303")]
gp_timer! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
        u32,
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM4: (tim4, tim4en, tim4rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(feature = "stm32f334")]
gp_timer! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
gp_timer! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM4: (tim4, tim4en, tim4rst),
        APB1: (apb1, pclk1),
        u16,
    },
}

#[cfg(any(feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
gp_timer! {
    {
        TIM2: (tim2, tim2en, tim2rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM3: (tim3, tim3en, tim3rst),
        APB1: (apb1, pclk1),
        u16,
    },
    {
        TIM4: (tim4, tim4en, tim4rst),
        APB1: (apb1, pclk1),
        u32,
    },
}

#[cfg(any(feature = "stm32f301", feature = "stm32f318"))]
gp_timer2! {
    // {
    //     TIM15: (tim15, tim15en, tim15rst),
    //     APB2: (apb2, pclk2),
    // },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(feature = "stm32f302")]
gp_timer2! {
    // {
    //     TIM15: (tim15, tim15en, tim15rst),
    //     APB2: (apb2, pclk2),
    // },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(feature = "stm32f303")]
gp_timer2! {
    // {
    //     TIM15: (tim15, tim15en, tim15rst),
    //     APB2: (apb2, pclk2),
    // },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
        u16,
    },
}

#[cfg(feature = "stm32f334")]
gp_timer2! {
    // {
    //     TIM15: (tim15, tim15en, tim15rst),
    //     APB2: (apb2, pclk2),
    // },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
        u16,
    }
}

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
gp_timer2! {
    // todo: TIM15 is lumped in with the others, except for in most registesr...
    // {
    //     TIM15: (tim15, tim15en, tim15rst),
    //     APB2: (apb2, pclk2),
    // },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
        u16,
    }
}

#[cfg(any(feature = "stm32f328", feature = "stm32f358", feature = "stm32f398"))]
gp_timer2! {
    // {
    //     TIM15: (tim15, tim15en, tim15rst),
    //     APB2: (apb2, pclk2),
    // },
    {
        TIM16: (tim16, tim16en, tim16rst),
        APB2: (apb2, pclk2),
        u16,
    },
    {
        TIM17: (tim17, tim17en, tim17rst),
        APB2: (apb2, pclk2),
        u16,
    }
}
