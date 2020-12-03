# `stm32f3xx-hal`

[![Build Status](https://github.com/stm32-rs/stm32f3xx-hal/workflows/CI/badge.svg)](https://github.com/stm32-rs/stm32f3xx-hal/actions)
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

## [Changelog](CHANGELOG.md)

## Selecting the right chip

This crate requires you to specify your target chip as a feature.

*Example: The STM32F3Discovery board has a STM32F303VCT6 chip.
So you want to expand your call to `cargo` with `--features stm32f303xc`.*

### Possible chips

[comment]: # (Any changes here should be mirrored in src/lib.rs)

Note: `x` denotes any character in [a-z]
*   stm32f301xb
*   stm32f301xc
*   stm32f301xd
*   stm32f301xe
*   stm32f318
*   stm32f302xb
*   stm32f302xc
*   stm32f302xd
*   stm32f302xe
*   stm32f302x6
*   stm32f302x8
*   stm32f303xb
*   stm32f303xc
*   stm32f303xd
*   stm32f303xe
*   stm32f303x6
*   stm32f303x8
*   stm32f373
*   stm32f378
*   stm32f334
*   stm32f328
*   stm32f358
*   stm32f398

### Background

For some of the stm32f3xx chips there are sub-variants that differ in
functionality, peripheral use and hence 'under the hood' implementation.  To
allow the full use of all peripherals on certain subvariants without
allowing for code that just doesn't run on other sub-vairants, they are
distinct features that need to be specified.

As this crate is still under fundamental development, expect more
sub-variants replacing the plain variants in the future as we are
implementing more stuff.  It is not desired to allow the plain variants to
be used as this leads to confusion.
*Example: the stm32f303xc has a gpio_e bank while the stm32f303x6 does
not. Hence we don't want to expose the gpoio_e bank on all stm32f303 (i.e.
when specifying the feature stm32f303) albeit a stm32f303xc user would
expect it to do so.*

### Detailed steps to select the right chip

1. Get the full name of the chip you are using from your datasheet, user manual or other source.

    *Example: We want to use the STM32F3Discovery kit.*
    *The [Usermanual][] tells us it's using a STM32F303VC chip.*

2. Find your chip as a feature in the list above.

    *Example: Looking for the right feature for our STM32F303VC chip we first find
    `stm32f301xb`. This is the wrong chip, as we're not looking for `f301` but for `f303`.*

    *Looking further we find `stm32f303xc`. This matches STM32F303VC (note that VC → xc).*

3. Add the chip name as a feature to your cargo call.

    *Example: Using the STM32F303VC chip we run `cargo check --features stm32f303xc`.*

[Usermanual]: https://www.st.com/content/ccc/resource/technical/document/user_manual/8a/56/97/63/8d/56/41/73/DM00063382.pdf/files/DM00063382.pdf/jcr:content/translations/en.DM00063382.pdf

## License

[0-clause BSD license](LICENSE-0BSD.txt).
