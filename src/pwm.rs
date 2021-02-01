/*!
  # Pulse width modulation

  Numerous stm32 timers can be used to output pulse width modulated
  signals on a variety of pins.  The timers support up to 4
  simultaneous pwm outputs in separate `Channels`.  These channels
  share a period and resolution, but can have a different duty cycle.
  All pins on a shared channel have the exact same output.

  ## Creating the (unconfigured) channels

  Before we connect any pins, we need to convert our timer peripheral
  into a set of channels.  We may only be interested in using one or
  two of these channels, so we can simply ignore them with `_` when we
  destructure.

  ```
    // (Other imports omitted)
    use stm32f3xx-hal::pwm::tim3;

    let dp = stm32f303::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Set the resolution of our duty cycle to 9000 and our period to
    // 50hz.
    let mut (c1_no_pins, _, _, c4_no_pins) =
        tim3(device.TIM3, 9000, 50.hz(), clocks);
  ```

  In this case, we're only going to use channel 1 and channel 4.
  Currently we can't enable these timers, because they don't have any
  pins, so the following wouldn't compile.


  ```
    // DOES NOT COMPILE
    c1_no_pins.enable();
    c4_no_pins.enable();
  ```

  ## Connecting our pins and enabling the channels

  From here we can connect as many compatible pins as we like.  Once
  the channels have pins connected they can be enabled.

  ```
    let mut gpioa = dp.GPIOB.split(&mut rcc.ahb);
    let pa6 = gpioa.pa6.into_af2(&mut gpioa.moder, &mut gpioa.afrl);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let pb1 = gpiob.pb1.into_af2(&mut gpiob.moder, &mut gpiob.afrl);
    let pb4 = gpiob.pb4.into_af2(&mut gpiob.moder, &mut gpiob.afrl);

    let mut ch1 = ch1_no_pins
        .output_to_pa6(pa6)
        .output_to_pb4(pb4);

    let mut ch4 = ch4_no_pins
        .output_to_pb1(pb1);

    ch1.enable();
    ch4.enable();
  ```

  All three pins will output a 50hz period. PA6 and PB4 will share a
  duty cycle, but the duty cycle for PB1 can be controlled
  independently.

  ```
    // Affect PA6 and PB4
    ch1.set_duty_cycle(1000);

    // Affect only PB1
    ch4.set_duty_cycle(2000);
  ```

  ## Single channel timers

  Timers that only have only one channel do not return a tuple, and
  instead return the (unconfigured) channel directly.

  ```
    // (Other imports omitted)
    use stm32f3xx-hal::pwm::tim16;

    let dp = stm32f303::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Set the resolution of our duty cycle to 9000 and our period to
    // 50hz.
    let mut c1_no_pins = tim16(device.TIM3, 9000, 50.hz(), clocks);
  ```

  ## Complementary timers

  Certain timers have complementary outputs.  Currently, channels can
  output to _either_ pins used for standard or complementary pins (and
  do not exhibit complementary behaviors).  Most of the time this will
  be totally invisible.

  In this example, we use a complementary pin in the same way we'd use
  any other pwm channel.

  ```
    // (Other imports omitted)
    use stm32f3xx-hal::pwm::tim1;

    let dp = stm32f303::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Set the resolution of our duty cycle to 9000 and our period to
    // 50hz.
    let mut (ch1_no_pins, _, _, _) = tim1(device.TIM3, 9000, 50.hz(), clocks);

    let mut gpioa = dp.GPIOB.split(&mut rcc.ahb);
    let pa7 = gpioa.pa7.into_af6(&mut gpioa.moder, &mut gpioa.afrl);

    let mut ch1 = ch1_no_pins.output_to(pa7);
    ch1.enable();
  ```

  We used this channel/pin exactly like any previous example.

  However, we cannot use standard and complementary pins
  simultaneously.  Luckily, typestates enforce this for us.

  ```
    ...

    let mut gpioa = dp.GPIOB.split(&mut rcc.ahb);
    let pa7 = gpioa.pa7.into_af6(&mut gpioa.moder, &mut gpioa.afrl);
    let pa8 = gpioa.pa8.into_af6(&mut gpioa.moder, &mut gpioa.afrl);

    let mut ch1 = ch1_no_pins
        .output_to(pa7)
        // DOES NOT COMPILE
        .output_to(pa8);
  ```

  Once we've connected a complementary pin (PA7) we are now _only_
  allowed to use other complementary pins.  PA8 is a valid choice if
  we have no pins in use, but it cannot be used once we've used PA7.

  A usage example can be found at [examples/pwm.rs]

  [examples/pwm.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.6.0/examples/pwm.rs
*/

use crate::{
    gpio::{self, gpioa, gpiob},
    hal::PwmPin,
    pac::{RCC, TIM15, TIM16, TIM17, TIM2},
    rcc::Clocks,
    time::Hertz,
};
use core::marker::PhantomData;
use core::u16;

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::gpio::gpiod;
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::gpio::gpioe;
#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398",
))]
use crate::gpio::{gpioc, gpiof};

/// Output Compare Channel 1 of Timer 1 (type state)
pub struct TIM2_CH1 {}
/// Output Compare Channel 2 of Timer 1 (type state)
pub struct TIM2_CH2 {}
/// Output Compare Channel 3 of Timer 1 (type state)
pub struct TIM2_CH3 {}
/// Output Compare Channel 4 of Timer 1 (type state)
pub struct TIM2_CH4 {}

/// Output Compare Channel 1 of Timer 15 (type state)
pub struct TIM15_CH1 {}
/// Output Compare Channel 2 of Timer 15 (type state)
pub struct TIM15_CH2 {}

/// Output Compare Channel 1 of Timer 16 (type state)
pub struct TIM16_CH1 {}

/// Output Compare Channel 1 of Timer 17 (type state)
pub struct TIM17_CH1 {}

/// Type state used to represent a channel that has no pins yet
pub struct NoPins {}
/// Type state used to represent a channel is using regular pins
pub struct WithPins {}
/// Type state used to represent a channel is using (only) complementary pins
pub struct WithNPins {}

/// Representation of a Channel for an abritary timer channel,
/// that also holds a type state for whether or not this channel
/// is using any pins yet.
///
/// If there are no pins supplied, it cannot be enabled.
pub struct PwmChannel<X, T> {
    timx_chy: PhantomData<X>,
    pin_status: PhantomData<T>,
}

pub struct ExactArrAndPsc<r> {
    pub arr: r,
    pub psc: u16,
}

pub trait ClocksToArrAndPsc {
    type Arr;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr>;
}

impl<arr> ClocksToArrAndPsc for ExactArrAndPsc<arr> {
    type Arr = arr;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr> {
        self
    }
}

pub struct ResAndFreq<r> {
    resolution: r,
    frequency: Hertz,
}

impl<arr: Into<u32>> ClocksToArrAndPsc for ResAndFreq<arr> {
    type Arr = arr;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr> {
        // calculate the pre-scaler
        let prescale_factor = timer_freq / self.resolution.into() / self.frequency.0;
        ExactArrAndPsc {
            arr: self.resolution,
            psc: prescale_factor as u16 - 1,
        }
    }
}

// Picks the greatest resolution that yields a reasonably approximate frequency.
// In many cases (the higher the frequency), the frequency will be exact or have
// maximum possible accuracy regardless (as not all frequencies _can_ be
// perfectly expressed).
// Error is bounded by: `(Fi + m * Fo + 2 Fo) / ((m - 2) * (Fi + Fo))` where
// Fi is the input clock frequency
// Fo is the output clock frequency
// m is the maximum resolution
//
// Example:
//   - Fi = 72MHz
//   - Fo = 50Hz
//   - Max Resolution = 2^16 (standard timer)
//
// Then:
//   - Worst Case Error ~= 0.00146%
//   - Worst Case Frequency ~= 50.000728Hz
//   - Actual Error ~= 0.000833%
//   - Actual Frequency ~= 50.0004167Hz
//
// From:
// `Fi / Fo = (a - 1) * m + Rm` such that `Rm >= 0`, and `Rm < m`
// `Fi / Fo = b * a + Ra` such that `Ra >= 0`, and `Ra < a`
// `b <= m`
//
// `Fi / Fo = b * a + Ra`
// Worst case Ra = (a - 1)
//
// `Fi / Fo = b * a + (a - 1)`
// `Fi / Fo - (a - 1) = b * a`
// `Fi / Fo - a + 1 = b * a`
// `(Fi / Fo - a + 1) / a = b`
// `(Fi / Fo - a + 1) / a <= m`
// `Fi / Fo - a + 1 <= m * a`
// `Fi / Fo + 1 <= m * a - a`
// `Fi / Fo + 1 <= a * (m - 1)`
// `(Fi / Fo + 1) / (m - 1) <= a`
//
// error:
// (Fo - (Fi / (b * a))) / Fo
// (Fo - (Fi / (Fi / Fo - Ra))) / Fo
//
// Worst case Ra = (a - 1)
// (Fo - (Fi / (Fi / Fo - (a - 1)))) / Fo
//
// worst case a = (Fi / Fo + 1) / (m - 1)
// (Fo - (Fi / (Fi / Fo - ((Fi / Fo + 1) / (m - 1) - 1)))) / Fo
//
// simplified:
// (Fi + m * Fo + 2 Fo) / ((m - 2) * (Fi + Fo))
pub struct PrioritizeResolution<r> {
    resolution: PhantomData<r>,
    frequency: Hertz,
}

impl ClocksToArrAndPsc for PrioritizeResolution<u16> {
    type Arr = u16;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr> {
        // calculate the pre-scaler
        // We need the resolution * prescale factor to equal the total_scaling
        let total_scaling = timer_freq / self.frequency.0;
        // We find the smallest factor that will result in the other factor
        // being less than 2^16.
        let prescale_factor =
            total_scaling / 2 ^ 16 + if total_scaling % 2 ^ 16 == 0 { 1 } else { 0 };
        // Given the smallest factor, we can now find its complement
        // Together, the two give an error bounded approximate of the total
        // scaling target.
        let resolution = total_scaling / prescale_factor;
        ExactArrAndPsc {
            arr: (resolution - 1) as u16,
            psc: (prescale_factor - 1) as u16,
        }
    }
}

impl ClocksToArrAndPsc for PrioritizeResolution<u32> {
    type Arr = u32;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr> {
        // A high-resolution timer will always have enough bits to divide out even to the lowest
        // frequency for the Hertz type (1hz).
        ExactArrAndPsc {
            arr: (timer_freq / self.frequency.0) - 1,
            psc: 1,
        }
    }
}

struct Wheel {
    n: u32,
    pre_division: Option<u32>,
    k: u32,
    i: usize,
}

// Gaps between the primes <= 31 excluding 2 and 3
// AKA: the wheel in our wheel.
const inc: [u32; 9] = [2, 4, 2, 4, 2, 4, 4, 2, 2];

impl Iterator for Wheel {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match self.pre_division {
            Some(i) => {
                if self.n % i == 0 {
                    self.n = self.n / i;
                    Some(i)
                } else {
                    self.pre_division = match i {
                        2 => Some(3),
                        2 => Some(5),
                        5 => None,
                    };
                    self.next()
                }
            }
            None => {
                if self.k * self.k > self.n {
                    None
                } else if self.n % self.k == 0 {
                    self.n = self.n / self.k;
                    Some(self.k)
                } else {
                    self.k += inc[self.i];
                    self.i = if self.i == 7 { 0 } else { self.i + 1 };
                    self.next()
                }
            }
        }
    }
}

fn factor(n: u32) -> Wheel {
    Wheel {
        n: n,
        pre_division: Some(2),
        k: 7,
        i: 0,
    }
}

// PrioritizeFreq will give the most accurate frequency possible, while
// attempting to give a reasonably large resolution (but not necessarily
// optimal).
//
// While it is theoretically possible that frequency will not be optimal if a
// specific combination of factors must be used (rather than simply the product
// of the smallest) to fit the values into the dividing registers, these are
// exceedingly rare and strange cases that require ratios (such as a period of
// 3284894596/72000000s).
//
// Some inputs are pathalogical in time when the total clock division must be
// factored, but it and its neighbors do not have factors that fit into the
// dividing registers.
pub struct PrioritizeFreq<r> {
    resolution: PhantomData<r>,
    frequency: Hertz,
}

impl ClocksToArrAndPsc for PrioritizeFreq<u16> {
    type Arr = u16;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr> {
        // Our target is the total clock division.  We now must find the closest
        // value that factors nicely into our two registers.
        let target = timer_freq / self.frequency.0;

        // If our target fails to factor, we're start alternating incrementing
        // up and down from our target.  This tells us which way we're hunting
        // currently.
        let mut searchUp = true;

        // Our search offset tells us how far we've strayed from our target
        let mut searchOffset = 1;

        let mut resolution = target;
        let mut prescale_factor;

        loop {
            prescale_factor = 1;
            // If needed, start shifting factors from our resolution to our
            // prescaler
            if resolution > u16::MAX as u32 {
                for x in factor(resolution) {
                    prescale_factor *= x;
                    resolution = resolution / x;
                    if resolution <= u16::MAX as u32 {
                        break;
                    }
                }
            }

            // We've completed shifting factors, but there's a few possibilies:
            //   1. We now have an acceptable divide and we're done
            //   2. Our factors didn't shrink the resolution enough so we
            //      continue
            //   3. Our factors shrank the resolution, but then blew out the
            //      prescaler so we continue (very unlikely, but possible)
            if resolution <= u16::MAX as u32 && prescale_factor <= u16::MAX as u32 {
                break;
            }

            // Since our current target failed to factor, we try the next
            // closest value to our target.
            if searchUp {
                resolution = target + searchOffset;
            } else {
                resolution = target - searchOffset;
                searchOffset += 1
            }
            searchUp = !searchUp;
        }
        ExactArrAndPsc {
            arr: (resolution as u16) - 1,
            psc: (prescale_factor as u16) - 1,
        }
    }
}

impl ClocksToArrAndPsc for PrioritizeFreq<u32> {
    type Arr = u32;

    fn timer_freq_to_arr_and_psc(self, timer_freq: u32) -> ExactArrAndPsc<Self::Arr> {
        // A high-resolution timer will always have enough bits to divide out even to the lowest
        // frequency for the Hertz type (1hz).
        ExactArrAndPsc {
            arr: (timer_freq / self.frequency.0) - 1,
            psc: 1,
        }
    }
}

macro_rules! pwm_timer_private {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $apbxrstr:ident, $pclkz:ident, $timxrst:ident, $timxen:ident, $enable_break_timer:expr, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        /// Create one or more output channels from a TIM Peripheral
        /// This function requires the maximum resolution of the duty cycle,
        /// the period of the PWM signal and the frozen clock configuration.
        ///
        /// The resolution should be chosen to offer sufficient steps against
        /// your target peripheral.  For example, a servo that can turn from
        /// 0 degrees (2% duty cycle) to 180 degrees (4% duty cycle) might choose
        /// a resolution of 9000.  This allows the servo to be set in increments
        /// of exactly one degree.
        #[allow(unused_parens)]
        pub fn $timx<cc: ClocksToArrAndPsc<Arr = $res>>(tim: $TIMx, clock_config: cc, clocks: &Clocks) -> ($(PwmChannel<$TIMx_CHy, NoPins>),+) {
            // Power the timer and reset it to ensure a clean state
            // We use unsafe here to abstract away this implementation detail
            // Justification: It is safe because only scopes with mutable references
            // to TIMx should ever modify this bit.
            unsafe {
                (*RCC::ptr()).$apbxenr.modify(|_, w| w.$timxen().set_bit());
                (*RCC::ptr()).$apbxrstr.modify(|_, w| w.$timxrst().set_bit());
                (*RCC::ptr()).$apbxrstr.modify(|_, w| w.$timxrst().clear_bit());
            }

            // enable auto reload preloader
            tim.cr1.modify(|_, w| w.arpe().set_bit());

            // TODO: This is repeated in the timer/pwm module.
            // It might make sense to move into the clocks as a crate-only property.
            // TODO: ppre1 is used in timer.rs (never ppre2), should this be dynamic?
            let clock_freq = clocks.$pclkz().0 * if clocks.ppre1() == 1 { 1 } else { 2 };
            let ExactArrAndPsc { arr, psc } : ExactArrAndPsc<$res> = clock_config.timer_freq_to_arr_and_psc(clock_freq);

            // Set the "resolution" of the duty cycle (ticks before restarting at 0)
            // Oddly this is unsafe for some timers and not others
            //
            // NOTE(write): not all timers are documented in stm32f3, thus marked unsafe.
            // This write uses all bits of this register so there are no unknown side effects.
            #[allow(unused_unsafe)]
            tim.arr.write(|w| unsafe {
                w.arr().bits(arr)
            });

            // Set the pre-scaler
            // NOTE(write): uses all bits of this register.
            tim.psc.write(|w| w.psc().bits(psc));

            // Make the settings reload immediately
            // NOTE(write): write to a state-less register.
            tim.egr.write(|w| w.ug().set_bit());

            // Enable outputs (STM32 Break Timer Specific)
            $enable_break_timer(&tim);

            // Enable the Timer
            tim.cr1.modify(|_, w| w.cen().set_bit());

            // TODO: Passing in the constructor is a bit silly,
            // is there an alternative approach to get this to repeat,
            // even though its not dynamic?
            ($($x { timx_chy: PhantomData, pin_status: PhantomData }),+)
        }
    }
}

macro_rules! pwm_timer_basic {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $apb1rstr:ident, $pclkz:ident, $timxrst:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $apb1rstr,
            $pclkz,
            $timxrst,
            $timxen,
            |_| (),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}

macro_rules! pwm_timer_with_break {
    ($timx:ident, $TIMx:ty, $res:ty, $apbxenr:ident, $apbxrstr:ident, $pclkz:ident, $timxrst:ident, $timxen:ident, [$($TIMx_CHy:ident),+], [$($x:ident),+]) => {
        pwm_timer_private!(
            $timx,
            $TIMx,
            $res,
            $apbxenr,
            $apbxrstr,
            $pclkz,
            $timxrst,
            $timxen,
            |tim: &$TIMx| tim.bdtr.modify(|_, w| w.moe().set_bit()),
            [$($TIMx_CHy),+],
            [$($x),+]
        );
    }
}

macro_rules! pwm_channel_pin {
    ($resulting_state:ident, $TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty, $ccmrz_output:ident, $ocym:ident, $ocype:ident) => {
        impl PwmChannel<$TIMx_CHy, NoPins> {
            /// Output to a specific pin from a channel that does not yet have
            /// any pins.  This channel cannot be enabled until this method
            /// is called.
            ///
            /// The pin is consumed and cannot be returned.
            pub fn $output_to_pzx(self, _p: $Pin) -> PwmChannel<$TIMx_CHy, $resulting_state> {
                unsafe {
                    (*$TIMx::ptr()).$ccmrz_output().modify(|_, w| {
                        w
                            // Select PWM Mode 1 for CHy
                            .$ocym()
                            .bits(0b0110)
                            // set pre-load enable so that updates to the duty cycle
                            // propagate but _not_ in the middle of a cycle.
                            .$ocype()
                            .set_bit()
                    });
                }
                PwmChannel {
                    timx_chy: PhantomData,
                    pin_status: PhantomData,
                }
            }
        }

        impl PwmChannel<$TIMx_CHy, $resulting_state> {
            /// Output to a specific pin from a channel is already configured
            /// with output pins.  There is no limit to the number of pins that
            /// can be used (as long as they are compatible).
            ///
            /// The pin is consumed and cannot be returned.
            pub fn $output_to_pzx(self, _p: $Pin) -> PwmChannel<$TIMx_CHy, $resulting_state> {
                self
            }
        }
    };
}

macro_rules! pwm_channel1_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr1_output,
            oc1m,
            oc1pe
        );
    };
}

macro_rules! pwm_channel1n_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithNPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr1_output,
            oc1m,
            oc1pe
        );
    };
}

macro_rules! pwm_channel2_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr1_output,
            oc2m,
            oc2pe
        );
    };
}

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! pwm_channel2n_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithNPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr1_output,
            oc2m,
            oc2pe
        );
    };
}

macro_rules! pwm_channel3_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr2_output,
            oc3m,
            oc3pe
        );
    };
}

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! pwm_channel3n_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithNPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr2_output,
            oc3m,
            oc3pe
        );
    };
}

macro_rules! pwm_channel4_pin {
    ($TIMx:ident, $TIMx_CHy:ident, $output_to_pzx:ident, $Pin:ty) => {
        pwm_channel_pin!(
            WithPins,
            $TIMx,
            $TIMx_CHy,
            $output_to_pzx,
            $Pin,
            ccmr2_output,
            oc4m,
            oc4pe
        );
    };
}

macro_rules! pwm_pin_for_pwm_channel_private {
    ($state:ident, $TIMx:ident, $TIMx_CHy:ty, $res:ty, $ccx_enable:ident, $ccrx:ident, $ccrq:ident) => {
        impl PwmPin for PwmChannel<$TIMx_CHy, $state> {
            type Duty = $res;

            fn disable(&mut self) {
                unsafe {
                    (*$TIMx::ptr())
                        .ccer
                        .modify(|_, w| w.$ccx_enable().clear_bit());
                }
            }

            fn enable(&mut self) {
                unsafe {
                    (*$TIMx::ptr())
                        .ccer
                        .modify(|_, w| w.$ccx_enable().set_bit());
                }
            }

            fn get_max_duty(&self) -> Self::Duty {
                unsafe { (*$TIMx::ptr()).arr.read().arr().bits() }
            }

            fn get_duty(&self) -> Self::Duty {
                unsafe { (*$TIMx::ptr()).$ccrx.read().$ccrq().bits() }
            }

            fn set_duty(&mut self, duty: Self::Duty) -> () {
                unsafe {
                    (*$TIMx::ptr()).$ccrx.modify(|_, w| w.$ccrq().bits(duty));
                }
            }
        }
    };
}

macro_rules! pwm_pin_for_pwm_channel {
    ($TIMx:ident, $TIMx_CHy:ty, $res:ty, $ccxe:ident, $ccrx:ident, $ccrq:ident) => {
        pwm_pin_for_pwm_channel_private!(WithPins, $TIMx, $TIMx_CHy, $res, $ccxe, $ccrx, $ccrq);
    };
}

macro_rules! pwm_pin_for_pwm_n_channel {
    ($TIMx:ident, $TIMx_CHy:ty, $res:ty, $ccxe:ident, $ccxne:ident, $ccrx:ident, $ccrq:ident) => {
        pwm_pin_for_pwm_channel_private!(WithPins, $TIMx, $TIMx_CHy, $res, $ccxe, $ccrx, $ccrq);

        pwm_pin_for_pwm_channel_private!(WithNPins, $TIMx, $TIMx_CHy, $res, $ccxne, $ccrx, $ccrq);
    };
}

// TIM1

#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim1_common {
    () => {
        use crate::pac::TIM1;

        /// Output Compare Channel 1 of Timer 1 (type state)
        pub struct TIM1_CH1 {}
        /// Output Compare Channel 2 of Timer 1 (type state)
        pub struct TIM1_CH2 {}
        /// Output Compare Channel 3 of Timer 1 (type state)
        pub struct TIM1_CH3 {}
        /// Output Compare Channel 4 of Timer 1 (type state)
        pub struct TIM1_CH4 {}

        pwm_timer_with_break!(
            tim1,
            TIM1,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim1rst,
            tim1en,
            [TIM1_CH1, TIM1_CH2, TIM1_CH3, TIM1_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_n_channel!(TIM1, TIM1_CH1, u16, cc1e, cc1ne, ccr1, ccr);
        pwm_pin_for_pwm_n_channel!(TIM1, TIM1_CH2, u16, cc2e, cc2ne, ccr2, ccr);
        pwm_pin_for_pwm_n_channel!(TIM1, TIM1_CH3, u16, cc3e, cc3ne, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM1, TIM1_CH4, u16, cc4e, ccr4, ccr);

        //Pins
        pwm_channel1_pin!(TIM1, TIM1_CH1, output_to_pa8, gpioa::PA8<gpio::AF6>);

        pwm_channel1n_pin!(TIM1, TIM1_CH1, output_to_pa7, gpioa::PA7<gpio::AF6>);
        pwm_channel1n_pin!(TIM1, TIM1_CH1, output_to_pa11, gpioa::PA11<gpio::AF6>);
        pwm_channel1n_pin!(TIM1, TIM1_CH1, output_to_pb13, gpiob::PB13<gpio::AF6>);
        pwm_channel1n_pin!(TIM1, TIM1_CH1, output_to_pc13, gpioc::PC13<gpio::AF4>);

        pwm_channel2_pin!(TIM1, TIM1_CH2, output_to_pa9, gpioa::PA9<gpio::AF6>);

        pwm_channel2n_pin!(TIM1, TIM1_CH2, output_to_pa12, gpioa::PA12<gpio::AF6>);
        pwm_channel2n_pin!(TIM1, TIM1_CH2, output_to_pb0, gpiob::PB0<gpio::AF6>);
        pwm_channel2n_pin!(TIM1, TIM1_CH2, output_to_pb14, gpiob::PB14<gpio::AF6>);

        pwm_channel3_pin!(TIM1, TIM1_CH3, output_to_pa10, gpioa::PA10<gpio::AF6>);

        pwm_channel3n_pin!(TIM1, TIM1_CH3, output_to_pb1, gpiob::PB1<gpio::AF6>);
        pwm_channel3n_pin!(TIM1, TIM1_CH3, output_to_pb15, gpiob::PB15<gpio::AF4>);
        pwm_channel3n_pin!(TIM1, TIM1_CH3, output_to_pf0, gpiof::PF0<gpio::AF6>);

        pwm_channel4_pin!(TIM1, TIM1_CH4, output_to_pa11, gpioa::PA11<gpio::AF11>);
    };
}

#[cfg(any(feature = "stm32f334", feature = "stm32f398"))]
macro_rules! tim1_ext1 {
    () => {
        pwm_channel1_pin!(TIM1, TIM1_CH1, output_to_pc0, gpioc::PC0<gpio::AF2>);

        pwm_channel2_pin!(TIM1, TIM1_CH2, output_to_pc1, gpioc::PC1<gpio::AF2>);

        pwm_channel3_pin!(TIM1, TIM1_CH3, output_to_pc2, gpioc::PC2<gpio::AF2>);

        pwm_channel4_pin!(TIM1, TIM1_CH4, output_to_pc3, gpioc::PC3<gpio::AF2>);
    };
}

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim1_ext2 {
    () => {
        pwm_channel1_pin!(TIM1, TIM1_CH1, output_to_pe9, gpioe::PE9<gpio::AF2>);

        pwm_channel1n_pin!(TIM1, TIM1_CH1, output_to_pe8, gpioe::PE8<gpio::AF2>);

        pwm_channel2_pin!(TIM1, TIM1_CH2, output_to_pe11, gpioe::PE11<gpio::AF2>);

        pwm_channel2n_pin!(TIM1, TIM1_CH2, output_to_pe10, gpioe::PE10<gpio::AF2>);

        pwm_channel3_pin!(TIM1, TIM1_CH3, output_to_pe13, gpioe::PE13<gpio::AF2>);

        pwm_channel3n_pin!(TIM1, TIM1_CH3, output_to_pe12, gpioe::PE12<gpio::AF2>);

        pwm_channel4_pin!(TIM1, TIM1_CH4, output_to_pe14, gpioe::PE14<gpio::AF2>);
    };
}

// TODO: stm32f301 has TIM1 with ext1
#[cfg(any(
    feature = "stm32f318",
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim1_common!();

#[cfg(any(feature = "stm32f334", feature = "stm32f398"))]
tim1_ext1!();

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim1_ext2!();

// TIM2

pwm_timer_basic!(
    tim2,
    TIM2,
    u32,
    apb1enr,
    apb1rstr,
    pclk1,
    tim2rst,
    tim2en,
    [TIM2_CH1, TIM2_CH2, TIM2_CH3, TIM2_CH4],
    [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
);

// Channels
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH1, u32, cc1e, ccr1, ccr);
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH2, u32, cc2e, ccr2, ccr);
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH3, u32, cc3e, ccr3, ccr);
pwm_pin_for_pwm_channel!(TIM2, TIM2_CH4, u32, cc4e, ccr4, ccr);

// Pins
pwm_channel1_pin!(TIM2, TIM2_CH1, output_to_pa0, gpioa::PA0<gpio::AF1>);
pwm_channel1_pin!(TIM2, TIM2_CH1, output_to_pa5, gpioa::PA5<gpio::AF1>);
pwm_channel1_pin!(TIM2, TIM2_CH1, output_to_pa15, gpioa::PA15<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel1_pin!(TIM2, TIM2_CH1, output_to_pd3, gpiod::PD3<gpio::AF2>);

pwm_channel2_pin!(TIM2, TIM2_CH2, output_to_pa1, gpioa::PA1<gpio::AF1>);
pwm_channel2_pin!(TIM2, TIM2_CH2, output_to_pb3, gpiob::PB3<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel2_pin!(TIM2, TIM2_CH2, output_to_pd4, gpiod::PD4<gpio::AF2>);

pwm_channel3_pin!(TIM2, TIM2_CH3, output_to_pa2, gpioa::PA2<gpio::AF1>);
pwm_channel3_pin!(TIM2, TIM2_CH3, output_to_pa9, gpioa::PA9<gpio::AF10>);
pwm_channel3_pin!(TIM2, TIM2_CH3, output_to_pb10, gpiob::PB10<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel3_pin!(TIM2, TIM2_CH3, output_to_pd7, gpiod::PD7<gpio::AF2>);

pwm_channel4_pin!(TIM2, TIM2_CH4, output_to_pa3, gpioa::PA3<gpio::AF1>);
pwm_channel4_pin!(TIM2, TIM2_CH4, output_to_pa10, gpioa::PA10<gpio::AF1>);
#[cfg(not(any(feature = "stm32f373", feature = "stm32f378")))]
pwm_channel4_pin!(TIM2, TIM2_CH4, output_to_pb11, gpiob::PB11<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel4_pin!(TIM2, TIM2_CH4, output_to_pd6, gpiod::PD6<gpio::AF2>);

// TIM3

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim3_common {
    () => {
        use crate::pac::TIM3;

        /// Output Compare Channel 1 of Timer 3 (type state)
        pub struct TIM3_CH1 {}
        /// Output Compare Channel 2 of Timer 3 (type state)
        pub struct TIM3_CH2 {}
        /// Output Compare Channel 3 of Timer 3 (type state)
        pub struct TIM3_CH3 {}
        /// Output Compare Channel 4 of Timer 3 (type state)
        pub struct TIM3_CH4 {}

        pwm_timer_basic!(
            tim3,
            TIM3,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim3rst,
            tim3en,
            [TIM3_CH1, TIM3_CH2, TIM3_CH3, TIM3_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH2, u16, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH3, u16, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM3, TIM3_CH4, u16, cc4e, ccr4, ccr);

        // Pins
        pwm_channel1_pin!(TIM3, TIM3_CH1, output_to_pa6, gpioa::PA6<gpio::AF2>);
        pwm_channel1_pin!(TIM3, TIM3_CH1, output_to_pb4, gpiob::PB4<gpio::AF2>);

        pwm_channel2_pin!(TIM3, TIM3_CH2, output_to_pa4, gpioa::PA4<gpio::AF2>);
        pwm_channel2_pin!(TIM3, TIM3_CH2, output_to_pa7, gpioa::PA7<gpio::AF2>);
        pwm_channel2_pin!(TIM3, TIM3_CH2, output_to_pb5, gpiob::PB5<gpio::AF2>);

        pwm_channel3_pin!(TIM3, TIM3_CH3, output_to_pb0, gpiob::PB0<gpio::AF2>);

        pwm_channel4_pin!(TIM3, TIM3_CH4, output_to_pb1, gpiob::PB1<gpio::AF2>);
        pwm_channel4_pin!(TIM3, TIM3_CH4, output_to_pb7, gpiob::PB7<gpio::AF10>);
    };
}

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim3_ext1 {
    () => {
        pwm_channel1_pin!(TIM3, TIM3_CH1, output_to_pc6, gpioc::PC6<gpio::AF2>);

        pwm_channel2_pin!(TIM3, TIM3_CH2, output_to_pc7, gpioc::PC7<gpio::AF2>);

        pwm_channel3_pin!(TIM3, TIM3_CH3, output_to_pc8, gpioc::PC8<gpio::AF2>);

        pwm_channel4_pin!(TIM3, TIM3_CH4, output_to_pc9, gpioc::PC9<gpio::AF2>);
    };
}

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim3_ext2 {
    () => {
        pwm_channel1_pin!(TIM3, TIM3_CH1, output_to_pe2, gpioe::PE6<gpio::AF2>);

        pwm_channel2_pin!(TIM3, TIM3_CH2, output_to_pe3, gpioe::PE7<gpio::AF2>);

        pwm_channel3_pin!(TIM3, TIM3_CH3, output_to_pe4, gpioe::PE8<gpio::AF2>);

        pwm_channel4_pin!(TIM3, TIM3_CH4, output_to_pe5, gpioe::PE9<gpio::AF2>);
    };
}

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f328",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim3_common!();

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f334",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim3_ext1!();

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim3_ext2!();

#[cfg(feature = "stm32f373")]
pwm_channel2_pin!(TIM3, TIM3_CH2, output_to_pb0, gpiob::PB0<gpio::AF10>);

#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
pwm_channel3_pin!(TIM3, TIM3_CH3, output_to_pb6, gpiob::PB6<gpio::AF10>);

// TIM4

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim4_common {
    () => {
        use crate::pac::TIM4;

        /// Output Compare Channel 1 of Timer 4 (type state)
        pub struct TIM4_CH1 {}
        /// Output Compare Channel 2 of Timer 4 (type state)
        pub struct TIM4_CH2 {}
        /// Output Compare Channel 3 of Timer 4 (type state)
        pub struct TIM4_CH3 {}
        /// Output Compare Channel 4 of Timer 4 (type state)
        pub struct TIM4_CH4 {}

        pwm_timer_basic!(
            tim4,
            TIM4,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim4rst,
            tim4en,
            [TIM4_CH1, TIM4_CH2, TIM4_CH3, TIM4_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH2, u16, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH3, u16, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM4, TIM4_CH4, u16, cc4e, ccr4, ccr);

        // Pins
        pwm_channel1_pin!(TIM4, TIM4_CH1, output_to_pa11, gpioa::PA11<gpio::AF10>);
        pwm_channel1_pin!(TIM4, TIM4_CH1, output_to_pb6, gpiob::PB6<gpio::AF2>);

        pwm_channel2_pin!(TIM4, TIM4_CH2, output_to_pa12, gpioa::PA12<gpio::AF10>);
        pwm_channel2_pin!(TIM4, TIM4_CH2, output_to_pb7, gpiob::PB7<gpio::AF2>);

        pwm_channel3_pin!(TIM4, TIM4_CH3, output_to_pa13, gpioa::PA13<gpio::AF10>);
        pwm_channel3_pin!(TIM4, TIM4_CH3, output_to_pb8, gpiob::PB8<gpio::AF2>);

        pwm_channel4_pin!(TIM4, TIM4_CH4, output_to_pb9, gpiob::PB9<gpio::AF2>);
    };
}

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f358",
    feature = "stm32f398"
))]
macro_rules! tim4_ext {
    () => {
        pwm_channel1_pin!(TIM4, TIM4_CH1, output_to_pd12, gpiod::PD12<gpio::AF2>);

        pwm_channel2_pin!(TIM4, TIM4_CH2, output_to_pd13, gpiod::PD13<gpio::AF2>);

        pwm_channel3_pin!(TIM4, TIM4_CH3, output_to_pd14, gpiod::PD14<gpio::AF2>);

        pwm_channel4_pin!(TIM4, TIM4_CH4, output_to_pd15, gpiod::PD15<gpio::AF2>);
        pwm_channel4_pin!(TIM4, TIM4_CH4, output_to_pf6, gpiof::PF6<gpio::AF2>);
    };
}

#[cfg(any(
    feature = "stm32f302",
    feature = "stm32f303",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim4_common!();

#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f373",
    feature = "stm32f378",
    feature = "stm32f358",
    feature = "stm32f398"
))]
tim4_ext!();

// TIM5

#[cfg(feature = "stm32f373")]
macro_rules! tim5 {
    () => {
        use crate::pac::TIM5;

        /// Output Compare Channel 1 of Timer 5 (type state)
        pub struct TIM5_CH1 {}
        /// Output Compare Channel 2 of Timer 5 (type state)
        pub struct TIM5_CH2 {}
        /// Output Compare Channel 3 of Timer 5 (type state)
        pub struct TIM5_CH3 {}
        /// Output Compare Channel 4 of Timer 5 (type state)
        pub struct TIM5_CH4 {}

        pwm_timer_basic!(
            tim5,
            TIM5,
            u32,
            apb1enr,
            apb1rstr,
            pclk1,
            tim5rst,
            tim5en,
            [TIM5_CH1, TIM5_CH2, TIM5_CH3, TIM5_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH1, u32, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH2, u32, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH3, u32, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM5, TIM5_CH4, u32, cc4e, ccr4, ccr);

        // Pins
        pwm_channel1_pin!(TIM5, TIM5_CH1, output_to_pa0, gpioa::PA0<gpio::AF2>);
        pwm_channel1_pin!(TIM5, TIM5_CH1, output_to_pa8, gpioa::PA8<gpio::AF2>);
        pwm_channel1_pin!(TIM5, TIM5_CH1, output_to_pc0, gpioc::PC0<gpio::AF2>);

        pwm_channel2_pin!(TIM5, TIM5_CH2, output_to_pa1, gpioa::PA1<gpio::AF2>);
        pwm_channel2_pin!(TIM5, TIM5_CH2, output_to_pa11, gpioa::PA11<gpio::AF2>);
        pwm_channel2_pin!(TIM5, TIM5_CH2, output_to_pc1, gpioc::PC1<gpio::AF2>);

        pwm_channel3_pin!(TIM5, TIM5_CH3, output_to_pa2, gpioa::PA2<gpio::AF2>);
        pwm_channel3_pin!(TIM5, TIM5_CH3, output_to_pa12, gpioa::PA12<gpio::AF2>);
        pwm_channel3_pin!(TIM5, TIM5_CH3, output_to_pc2, gpioc::PC2<gpio::AF2>);

        pwm_channel4_pin!(TIM5, TIM5_CH4, output_to_pa3, gpioa::PA3<gpio::AF2>);
        pwm_channel4_pin!(TIM5, TIM5_CH4, output_to_pa13, gpioa::PA13<gpio::AF2>);
        pwm_channel4_pin!(TIM5, TIM5_CH4, output_to_pc3, gpioc::PC3<gpio::AF2>);
    };
}

// TODO: This timer is also present in stm32f378
#[cfg(any(feature = "stm32f373"))]
tim5!();

// TIM8

#[cfg(any(feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
macro_rules! tim8 {
    () => {
        use crate::pac::TIM8;

        /// Output Compare Channel 1 of Timer 8 (type state)
        pub struct TIM8_CH1 {}
        /// Output Compare Channel 2 of Timer 8 (type state)
        pub struct TIM8_CH2 {}
        /// Output Compare Channel 3 of Timer 8 (type state)
        pub struct TIM8_CH3 {}
        /// Output Compare Channel 4 of Timer 8 (type state)
        pub struct TIM8_CH4 {}

        pwm_timer_with_break!(
            tim8,
            TIM8,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim8rst,
            tim8en,
            [TIM8_CH1, TIM8_CH2, TIM8_CH3, TIM8_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH1, u16, cc1e, cc1ne, ccr1, ccr);
        pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH2, u16, cc2e, cc2ne, ccr2, ccr);
        pwm_pin_for_pwm_n_channel!(TIM8, TIM8_CH3, u16, cc3e, cc3ne, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM8, TIM8_CH4, u16, cc4e, ccr4, ccr);

        //Pins
        pwm_channel1_pin!(TIM8, TIM8_CH1, output_to_pa15, gpioa::PA15<gpio::AF2>);
        pwm_channel1_pin!(TIM8, TIM8_CH1, output_to_pb6, gpiob::PB6<gpio::AF2>);
        pwm_channel1_pin!(TIM8, TIM8_CH1, output_to_pc6, gpioc::PC6<gpio::AF4>);

        pwm_channel1n_pin!(TIM8, TIM8_CH1, output_to_pa7, gpioa::PA7<gpio::AF4>);
        pwm_channel1n_pin!(TIM8, TIM8_CH1, output_to_pb3, gpiob::PB3<gpio::AF4>);
        pwm_channel1n_pin!(TIM8, TIM8_CH1, output_to_pc10, gpioc::PC10<gpio::AF4>);

        pwm_channel2_pin!(TIM8, TIM8_CH2, output_to_pa14, gpioa::PA14<gpio::AF5>);
        pwm_channel2_pin!(TIM8, TIM8_CH2, output_to_pb8, gpiob::PB8<gpio::AF10>);
        pwm_channel2_pin!(TIM8, TIM8_CH2, output_to_pc7, gpioc::PC7<gpio::AF4>);

        pwm_channel2n_pin!(TIM8, TIM8_CH2, output_to_pb0, gpiob::PB0<gpio::AF4>);
        pwm_channel2n_pin!(TIM8, TIM8_CH2, output_to_pb4, gpiob::PB4<gpio::AF4>);
        pwm_channel2n_pin!(TIM8, TIM8_CH2, output_to_pc11, gpioc::PC11<gpio::AF4>);

        pwm_channel3_pin!(TIM8, TIM8_CH3, output_to_pb9, gpiob::PB9<gpio::AF10>);
        pwm_channel3_pin!(TIM8, TIM8_CH3, output_to_pc8, gpioc::PC8<gpio::AF4>);

        pwm_channel3n_pin!(TIM8, TIM8_CH3, output_to_pb1, gpiob::PB1<gpio::AF4>);
        pwm_channel3n_pin!(TIM8, TIM8_CH3, output_to_pb5, gpiob::PB5<gpio::AF3>);
        pwm_channel3n_pin!(TIM8, TIM8_CH3, output_to_pc12, gpioc::PC12<gpio::AF4>);

        pwm_channel4_pin!(TIM8, TIM8_CH4, output_to_pc9, gpioc::PC9<gpio::AF4>);
    };
}

#[cfg(any(feature = "stm32f303", feature = "stm32f358", feature = "stm32f398"))]
tim8!();

#[cfg(any(
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel4_pin!(TIM8, TIM8_CH4, output_to_pd1, gpiod::PD1<gpio::AF4>);

// TIM12

#[cfg(feature = "stm32f373")]
macro_rules! tim12 {
    () => {
        use crate::pac::TIM12;

        /// Output Compare Channel 1 of Timer 12 (type state)
        pub struct TIM12_CH1 {}
        /// Output Compare Channel 2 of Timer 12 (type state)
        pub struct TIM12_CH2 {}
        /// Output Compare Channel 3 of Timer 12 (type state)
        pub struct TIM12_CH3 {}
        /// Output Compare Channel 4 of Timer 12 (type state)
        pub struct TIM12_CH4 {}

        pwm_timer_basic!(
            tim12,
            TIM12,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim12rst,
            tim12en,
            [TIM12_CH1, TIM12_CH2],
            [PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM12, TIM12_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM12, TIM12_CH2, u16, cc2e, ccr2, ccr);

        // Pins
        pwm_channel1_pin!(TIM12, TIM12_CH1, output_to_pa4, gpioa::PA4<gpio::AF10>);
        pwm_channel1_pin!(TIM12, TIM12_CH1, output_to_pa14, gpioa::PA14<gpio::AF10>);
        pwm_channel1_pin!(TIM12, TIM12_CH1, output_to_pb14, gpiob::PB14<gpio::AF10>);

        pwm_channel2_pin!(TIM12, TIM12_CH2, output_to_pa5, gpioa::PA5<gpio::AF10>);
        pwm_channel2_pin!(TIM12, TIM12_CH2, output_to_pa15, gpioa::PA15<gpio::AF10>);
        pwm_channel2_pin!(TIM12, TIM12_CH2, output_to_pb15, gpiob::PB15<gpio::AF10>);
    };
}

// TODO: This timer is also present in stm32f378
#[cfg(feature = "stm32f373")]
tim12!();

// TIM13

#[cfg(feature = "stm32f373")]
macro_rules! tim13 {
    () => {
        use crate::pac::TIM13;

        /// Output Compare Channel 1 of Timer 13 (type state)
        pub struct TIM13_CH1 {}
        /// Output Compare Channel 2 of Timer 13 (type state)
        pub struct TIM13_CH2 {}
        /// Output Compare Channel 3 of Timer 13 (type state)
        pub struct TIM13_CH3 {}
        /// Output Compare Channel 4 of Timer 13 (type state)
        pub struct TIM13_CH4 {}

        pwm_timer_basic!(
            tim13,
            TIM13,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim13rst,
            tim13en,
            [TIM13_CH1],
            [PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM13, TIM13_CH1, u16, cc1e, ccr1, ccr);

        // Pins
        pwm_channel1_pin!(TIM13, TIM13_CH1, output_to_pa6, gpioa::PA6<gpio::AF9>);
        pwm_channel1_pin!(TIM13, TIM13_CH1, output_to_pa9, gpioa::PA9<gpio::AF2>);
        pwm_channel1_pin!(TIM13, TIM13_CH1, output_to_pb3, gpiob::PB3<gpio::AF9>);
        pwm_channel1_pin!(TIM13, TIM13_CH1, output_to_pc4, gpioc::PC4<gpio::AF2>);
    };
}

#[cfg(feature = "stm32f373")]
tim13!();

// TIM14

#[cfg(feature = "stm32f373")]
macro_rules! tim14 {
    () => {
        use crate::pac::TIM14;

        /// Output Compare Channel 1 of Timer 14 (type state)
        pub struct TIM14_CH1 {}
        /// Output Compare Channel 2 of Timer 14 (type state)
        pub struct TIM14_CH2 {}
        /// Output Compare Channel 3 of Timer 14 (type state)
        pub struct TIM14_CH3 {}
        /// Output Compare Channel 4 of Timer 14 (type state)
        pub struct TIM14_CH4 {}

        pwm_timer_basic!(
            tim14,
            TIM14,
            u16,
            apb1enr,
            apb1rstr,
            pclk1,
            tim14rst,
            tim14en,
            [TIM14_CH1],
            [PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM14, TIM14_CH1, u16, cc1e, ccr1, ccr);

        // Pins
        pwm_channel1_pin!(TIM14, TIM14_CH1, output_to_pa5, gpioa::PA5<gpio::AF9>);
        pwm_channel1_pin!(TIM14, TIM14_CH1, output_to_pa7, gpioa::PA7<gpio::AF9>);
        pwm_channel1_pin!(TIM14, TIM14_CH1, output_to_pa10, gpioa::PA10<gpio::AF9>);
        pwm_channel1_pin!(TIM14, TIM14_CH1, output_to_pf9, gpiof::PF9<gpio::AF2>);
    };
}

// TODO: This timer is also present in stm32f378
#[cfg(feature = "stm32f373")]
tim14!();

// TIM15

pwm_timer_with_break!(
    tim15,
    TIM15,
    u16,
    apb2enr,
    apb2rstr,
    pclk2,
    tim15rst,
    tim15en,
    [TIM15_CH1, TIM15_CH2],
    [PwmChannel, PwmChannel]
);

// Channels
pwm_pin_for_pwm_n_channel!(TIM15, TIM15_CH1, u16, cc1e, cc1ne, ccr1, ccr1);
pwm_pin_for_pwm_channel!(TIM15, TIM15_CH2, u16, cc2e, ccr2, ccr2);

// Pins
pwm_channel1_pin!(TIM15, TIM15_CH1, output_to_pa2, gpioa::PA2<gpio::AF9>);
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
pwm_channel1_pin!(TIM15, TIM15_CH1, output_to_pb6, gpiob::PB6<gpio::AF9>);
pwm_channel1_pin!(TIM15, TIM15_CH1, output_to_pb14, gpiob::PB14<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel1_pin!(TIM15, TIM15_CH1, output_to_pf9, gpiof::PF9<gpio::AF3>);

pwm_channel1n_pin!(TIM15, TIM15_CH1, output_to_pa1, gpioa::PA1<gpio::AF9>);
pwm_channel1n_pin!(TIM15, TIM15_CH1, output_to_pb15, gpiob::PB15<gpio::AF2>);
pwm_channel2_pin!(TIM15, TIM15_CH2, output_to_pa3, gpioa::PA3<gpio::AF9>);
#[cfg(any(feature = "stm32f373", feature = "stm32f378"))]
pwm_channel2_pin!(TIM15, TIM15_CH2, output_to_pb7, gpiob::PB7<gpio::AF9>);
pwm_channel2_pin!(TIM15, TIM15_CH2, output_to_pb15, gpiob::PB15<gpio::AF2>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel2_pin!(TIM15, TIM15_CH2, output_to_pf10, gpiof::PF10<gpio::AF3>);

// TIM16

pwm_timer_with_break!(
    tim16,
    TIM16,
    u16,
    apb2enr,
    apb2rstr,
    pclk2,
    tim16rst,
    tim16en,
    [TIM16_CH1],
    [PwmChannel]
);

// Channels
pwm_pin_for_pwm_n_channel!(TIM16, TIM16_CH1, u16, cc1e, cc1ne, ccr1, ccr1);

// Pins
pwm_channel1_pin!(TIM16, TIM16_CH1, output_to_pa9, gpioa::PA6<gpio::AF1>);
pwm_channel1_pin!(TIM16, TIM16_CH1, output_to_pa12, gpioa::PA12<gpio::AF1>);
pwm_channel1_pin!(TIM16, TIM16_CH1, output_to_pb4, gpiob::PB4<gpio::AF1>);
pwm_channel1_pin!(TIM16, TIM16_CH1, output_to_pb8, gpiob::PB8<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel1_pin!(TIM16, TIM16_CH1, output_to_pe0, gpioe::PE0<gpio::AF4>);

pwm_channel1n_pin!(TIM16, TIM16_CH1, output_to_pa13, gpioa::PA13<gpio::AF1>);
pwm_channel1n_pin!(TIM16, TIM16_CH1, output_to_pb6, gpiob::PB6<gpio::AF1>);

// TIM17

pwm_timer_with_break!(
    tim17,
    TIM17,
    u16,
    apb2enr,
    apb2rstr,
    pclk2,
    tim17rst,
    tim17en,
    [TIM17_CH1],
    [PwmChannel]
);

// Channels
pwm_pin_for_pwm_n_channel!(TIM17, TIM17_CH1, u16, cc1e, cc1ne, ccr1, ccr1);

// Pins
pwm_channel1_pin!(TIM17, TIM17_CH1, output_to_pa7, gpioa::PA7<gpio::AF1>);
pwm_channel1_pin!(TIM17, TIM17_CH1, output_to_pb5, gpiob::PB5<gpio::AF10>);
pwm_channel1_pin!(TIM17, TIM17_CH1, output_to_pb9, gpiob::PB9<gpio::AF1>);
#[cfg(any(
    feature = "stm32f302xb",
    feature = "stm32f302xc",
    feature = "stm32f302xd",
    feature = "stm32f302xe",
    feature = "stm32f303xb",
    feature = "stm32f303xc",
    feature = "stm32f303xd",
    feature = "stm32f303xe",
    feature = "stm32f358",
    feature = "stm32f398"
))]
pwm_channel1_pin!(TIM17, TIM17_CH1, output_to_pe1, gpioe::PE1<gpio::AF4>);

pwm_channel1n_pin!(TIM17, TIM17_CH1, output_to_pa13, gpioa::PA13<gpio::AF1>);

// TIM19

#[cfg(feature = "stm32f373")]
macro_rules! tim19 {
    () => {
        use crate::pac::TIM19;

        /// Output Compare Channel 1 of Timer 19 (type state)
        pub struct TIM19_CH1 {}
        /// Output Compare Channel 2 of Timer 19 (type state)
        pub struct TIM19_CH2 {}
        /// Output Compare Channel 3 of Timer 19 (type state)
        pub struct TIM19_CH3 {}
        /// Output Compare Channel 4 of Timer 19 (type state)
        pub struct TIM19_CH4 {}

        pwm_timer_basic!(
            tim19,
            TIM19,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim19rst,
            tim19en,
            [TIM19_CH1, TIM19_CH2, TIM19_CH3, TIM19_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH1, u16, cc1e, ccr1, ccr);
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH2, u16, cc2e, ccr2, ccr);
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH3, u16, cc3e, ccr3, ccr);
        pwm_pin_for_pwm_channel!(TIM19, TIM19_CH4, u16, cc4e, ccr4, ccr);

        // Pins
        pwm_channel1_pin!(TIM19, TIM19_CH1, output_to_pa0, gpioa::PA0<gpio::AF11>);
        pwm_channel1_pin!(TIM19, TIM19_CH1, output_to_pb6, gpiob::PB6<gpio::AF11>);
        pwm_channel1_pin!(TIM19, TIM19_CH1, output_to_pc10, gpioc::PC10<gpio::AF2>);

        pwm_channel2_pin!(TIM19, TIM19_CH2, output_to_pa1, gpioa::PA1<gpio::AF11>);
        pwm_channel2_pin!(TIM19, TIM19_CH2, output_to_pb7, gpiob::PB7<gpio::AF11>);
        pwm_channel2_pin!(TIM19, TIM19_CH2, output_to_pc11, gpioc::PC11<gpio::AF2>);

        pwm_channel3_pin!(TIM19, TIM19_CH3, output_to_pa2, gpioa::PA2<gpio::AF11>);
        pwm_channel3_pin!(TIM19, TIM19_CH3, output_to_pb8, gpiob::PB8<gpio::AF11>);
        pwm_channel3_pin!(TIM19, TIM19_CH3, output_to_pc12, gpioc::PC12<gpio::AF2>);

        pwm_channel4_pin!(TIM19, TIM19_CH4, output_to_pa3, gpioa::PA3<gpio::AF11>);
        pwm_channel4_pin!(TIM19, TIM19_CH4, output_to_pb9, gpiob::PB9<gpio::AF11>);
        pwm_channel4_pin!(TIM19, TIM19_CH4, output_to_pd0, gpiod::PD0<gpio::AF2>);
    };
}

// TODO: This timer is also present in stm32f378
#[cfg(feature = "stm32f373")]
tim19!();

// TIM20
//
#[cfg(feature = "stm32f398")]
macro_rules! tim20 {
    () => {
        use crate::pac::TIM20;

        /// Output Compare Channel 1 of Timer 20 (type state)
        pub struct TIM20_CH1 {}
        /// Output Compare Channel 2 of Timer 20 (type state)
        pub struct TIM20_CH2 {}
        /// Output Compare Channel 3 of Timer 20 (type state)
        pub struct TIM20_CH3 {}
        /// Output Compare Channel 4 of Timer 20 (type state)
        pub struct TIM20_CH4 {}

        pwm_timer_basic!(
            tim20,
            TIM20,
            u16,
            apb2enr,
            apb2rstr,
            pclk2,
            tim20rst,
            tim20en,
            [TIM20_CH1, TIM20_CH2, TIM20_CH3, TIM20_CH4],
            [PwmChannel, PwmChannel, PwmChannel, PwmChannel]
        );

        // Channels
        // TODO: stm32f3 doesn't suppport registers for all 4 channels
        pwm_pin_for_pwm_n_channel!(TIM20, TIM20_CH1, u16, cc1e, cc1ne, ccr1, ccr1);

        //Pins
        pwm_channel1_pin!(TIM20, TIM20_CH1, output_to_pe2, gpioe::PE2<gpio::AF6>);

        pwm_channel1n_pin!(TIM20, TIM20_CH1, output_to_pe4, gpioe::PE4<gpio::AF6>);
    };
}

#[cfg(feature = "stm32f398")]
tim20!();
