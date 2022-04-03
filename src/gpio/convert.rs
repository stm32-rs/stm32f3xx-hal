use super::*;

impl<const P: char, const N: u8, MODE: PinMode> Pin<P, N, MODE> {
    /// Configures the pin to operate alternate mode
    pub fn into_alternate<const A: u8>(
        self,
        _moder: &mut MODER<P>,
        _otyper: &mut OTYPER<P>,
        _afr: &mut <Self as HL>::Afr,
    ) -> Pin<P, N, Alternate<A, PushPull>>
    where
        Self: super::marker::IntoAf<A>,
    {
        self.into_mode()
    }

    /// Configures the pin to operate in alternate open drain mode
    pub fn into_alternate_open_drain<const A: u8>(
        self,
        _moder: &mut MODER<P>,
        _otyper: &mut OTYPER<P>,
        _afr: &mut <Self as HL>::Afr,
    ) -> Pin<P, N, Alternate<A, OpenDrain>>
    where
        Self: super::marker::IntoAf<A>,
    {
        self.into_mode()
    }

    /// Configures the pin to operate as a input pin
    pub fn into_input(self) -> Pin<P, N, Input> {
        self.into_mode()
    }

    /// Configures the pin to operate as a floating input pin
    pub fn into_floating_input(
        self,
        _moder: &mut MODER<P>,
        pupdr: &mut PUPDR<P>,
    ) -> Pin<P, N, Input> {
        self.into_mode().internal_resistor(pupdr, Pull::None)
    }

    /// Configures the pin to operate as a pulled down input pin
    pub fn into_pull_down_input(
        self,
        _moder: &mut MODER<P>,
        pupdr: &mut PUPDR<P>,
    ) -> Pin<P, N, Input> {
        self.into_mode().internal_resistor(pupdr, Pull::Down)
    }

    /// Configures the pin to operate as a pulled up input pin
    pub fn into_pull_up_input(
        self,
        _moder: &mut MODER<P>,
        pupdr: &mut PUPDR<P>,
    ) -> Pin<P, N, Input> {
        self.into_mode().internal_resistor(pupdr, Pull::Up)
    }

    /// Configures the pin to operate as an open drain output pin
    /// Initial state will be low.
    pub fn into_open_drain_output(
        self,
        _moder: &mut MODER<P>,
        _otyper: &mut OTYPER<P>,
    ) -> Pin<P, N, Output<OpenDrain>> {
        self.into_mode()
    }

    /// Configures the pin to operate as an open-drain output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    pub fn into_open_drain_output_in_state(
        mut self,
        _moder: &mut MODER<P>,
        _otyper: &mut OTYPER<P>,
        initial_state: PinState,
    ) -> Pin<P, N, Output<OpenDrain>> {
        self._set_state(initial_state);
        self.into_mode()
    }

    /// Configures the pin to operate as an push pull output pin
    /// Initial state will be low.
    pub fn into_push_pull_output(
        mut self,
        _moder: &mut MODER<P>,
        _otyper: &mut OTYPER<P>,
    ) -> Pin<P, N, Output<PushPull>> {
        self._set_low();
        self.into_mode()
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    pub fn into_push_pull_output_in_state(
        mut self,
        _moder: &mut MODER<P>,
        _otyper: &mut OTYPER<P>,
        initial_state: PinState,
    ) -> Pin<P, N, Output<PushPull>> {
        self._set_state(initial_state);
        self.into_mode()
    }

    /// Configures the pin to operate as an analog input pin
    pub fn into_analog(self, _moder: &mut MODER<P>, _pupdr: &mut PUPDR<P>) -> Pin<P, N, Analog> {
        self.into_mode()
    }

    /// Configures the pin as a pin that can change between input
    /// and output without changing the type. It starts out
    /// as a floating input
    pub fn into_dynamic(self, moder: &mut MODER<P>, pupdr: &mut PUPDR<P>) -> DynamicPin<P, N> {
        self.into_floating_input(moder, pupdr);
        DynamicPin::new(Dynamic::InputFloating)
    }

    /// Puts `self` into mode `M`.
    ///
    /// This violates the type state constraints from `MODE`, so callers must
    /// ensure they use this properly.
    #[inline(always)]
    pub(super) fn mode<M: PinMode>(&mut self) {
        let offset = 2 * N;
        unsafe {
            if MODE::OTYPER != M::OTYPER {
                if let Some(otyper) = M::OTYPER {
                    (*Gpio::<P>::ptr())
                        .otyper
                        .modify(|r, w| w.bits(r.bits() & !(0b1 << N) | (otyper << N)));
                }
            }

            if MODE::AFR != M::AFR {
                if let Some(afr) = M::AFR {
                    if N < 8 {
                        let offset2 = 4 * { N };
                        (*Gpio::<P>::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(0b1111 << offset2)) | (afr << offset2))
                        });
                    } else {
                        let offset2 = 4 * { N - 8 };
                        (*Gpio::<P>::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(0b1111 << offset2)) | (afr << offset2))
                        });
                    }
                }
            }

            if MODE::MODER != M::MODER {
                (*Gpio::<P>::ptr())
                    .moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (M::MODER << offset)));
            }
        }
    }

    #[inline(always)]
    /// Converts pin into specified mode
    pub(super) fn into_mode<M: PinMode>(mut self) -> Pin<P, N, M> {
        self.mode::<M>();
        Pin::new()
    }
}

/// Marker trait for valid pin modes (type state).
///
/// It can not be implemented by outside types.
pub trait PinMode: crate::Sealed {
    // These constants are used to implement the pin configuration code.
    // They are not part of public API.

    #[doc(hidden)]
    const MODER: u32 = u32::MAX;
    #[doc(hidden)]
    const OTYPER: Option<u32> = None;
    #[doc(hidden)]
    const AFR: Option<u32> = None;
}

impl crate::Sealed for Input {}
impl PinMode for Input {
    const MODER: u32 = 0b00;
}

impl crate::Sealed for Analog {}
impl PinMode for Analog {
    const MODER: u32 = 0b11;
}

impl<Otype> crate::Sealed for Output<Otype> {}
impl PinMode for Output<OpenDrain> {
    const MODER: u32 = 0b01;
    const OTYPER: Option<u32> = Some(0b1);
}

impl PinMode for Output<PushPull> {
    const MODER: u32 = 0b01;
    const OTYPER: Option<u32> = Some(0b0);
}

impl<const A: u8, Otype> crate::Sealed for Alternate<A, Otype> {}
impl<const A: u8> PinMode for Alternate<A, OpenDrain> {
    const MODER: u32 = 0b10;
    const OTYPER: Option<u32> = Some(0b1);
    const AFR: Option<u32> = Some(A as _);
}

impl<const A: u8> PinMode for Alternate<A, PushPull> {
    const MODER: u32 = 0b10;
    const OTYPER: Option<u32> = Some(0b0);
    const AFR: Option<u32> = Some(A as _);
}
