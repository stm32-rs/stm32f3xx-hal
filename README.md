# stm32f3xx-hal

[![Build Status](https://travis-ci.com/stm32-rs/stm32f3xx-hal.svg)](https://travis-ci.com/stm32-rs/stm32f3xx-hal)
[![Crate](https://img.shields.io/crates/v/stm32f3xx-hal.svg)](https://crates.io/crates/stm32f3xx-hal)
[![Docs](https://docs.rs/stm32f3xx-hal/badge.svg)](https://docs.rs/stm32f3xx-hal)

`stm32f3xx-hal` contains a multi device hardware abstraction on top of the
peripheral access API for the STMicro STM32F3 series microcontrollers. The
selection of the MCU is done by feature gates, typically specified by board
support crates. Currently supported configurations are:

*   stm32f301 ✔️ YES!
*   stm32f318 ✔️ YES!
*   stm32f302 ✔️ YES!
*   stm32f303 ✔️ YES!
*   stm32f373 ✔️ YES!
*   stm32f378 ✔️ YES!
*   stm32f334 ✔️ YES!
*   stm32f328 ✔️ YES!
*   stm32f358 ✔️ YES!
*   stm32f398 ✔️ YES!

The idea behind this crate is to gloss over the slight differences in the
various peripherals available on those MCUs so a HAL can be written for all
chips in that same family without having to cut and paste crates for every
single model.

Collaboration on this crate is highly welcome as are pull requests!

This crate relies on Adam Greigs fantastic [`stm32f3`][] crate to provide
appropriate register definitions and implements a partial set of the
[`embedded-hal`][] traits.

Almost all of the implementation was shamelessly adapted from the
[`stm32f30x-hal`][] crate by Jorge Aparicio.

[`stm32f3`]: https://crates.io/crates/stm32f3
[`stm32f30x-hal`]: https://github.com/japaric/stm32f30x-hal
[`embedded-hal`]: https://github.com/japaric/embedded-hal

## License

[0-clause BSD license](LICENSE-0BSD.txt).

## Selecting the right feature gate

For some of the stm32f3xx chips there are sub-variants that behave differently
and need to be specified.

If your code built perfectly before and throws a compile error now, don't worry.
While there are a lot of cool new features coming your way, we want to make sure
your old code still behaves the same and there are no wrong assumptions
creating unexpected behaviour.

As this crate is still under fundamental development, expect more sub-variants
to pop up in the future.

### Steps to select the right feature
1. Get the full name of the chip you are using from your datasheet, user manual or other source.

    *Example: We want to use the STM32F3Discovery kit.*
    *The [Usermanual][] tells us it's using a STM32F303VC chip.*

2. Find your chip as a feature that derives `"device-selected"` in the [Cargo.toml] file.

    *Example: Looking for the right feature for our STM32F303VC chip we first find
    `stm32f301 = […]`. This is the wrong chip, as we're not looking for `f301` but for `f303`.*

    *Next in line is `stm32f303 = ["stm32f3/stm32f303", "needs-subvariant"]`. We're
    getting close! But there is no `"device-selected"` in the derived features list.*

    *Looking further we find `stm32f303xc`. This matches STM32F303VC (note that VC → xc) and derives `"device-selected`", so we're good to go.*

3. Add the chip name as a feature to your cargo call.

    *Example: Using the STM32F303VC chip we run `cargo check --features stm32f303xc`.*

[Usermanual]: https://www.st.com/content/ccc/resource/technical/document/user_manual/8a/56/97/63/8d/56/41/73/DM00063382.pdf/files/DM00063382.pdf/jcr:content/translations/en.DM00063382.pdf
