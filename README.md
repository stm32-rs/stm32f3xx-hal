# `stm32f3xx-hal`

[![Build Status](https://github.com/stm32-rs/stm32f3xx-hal/workflows/CI/badge.svg)](https://github.com/stm32-rs/stm32f3xx-hal/actions)
[![Crate](https://img.shields.io/crates/v/stm32f3xx-hal.svg)](https://crates.io/crates/stm32f3xx-hal)
[![Docs](https://docs.rs/stm32f3xx-hal/badge.svg)](https://docs.rs/stm32f3xx-hal)
[![Crates.io](https://img.shields.io/crates/d/stm32f3xx-hal.svg)](https://crates.io/crates/stm32f3xx-hal)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.59+-blue.svg)

`stm32f3xx-hal` contains a multi device hardware abstraction on top of the
peripheral access API for the STMicro STM32F3 series microcontrollers. The
selection of the MCU is done by feature gates, typically specified by board
support crates. An excerpt of supported chip variants:

* stm32f301
* stm32f318
* stm32f302
* stm32f303
* stm32f373
* stm32f378
* stm32f334
* stm32f328
* stm32f358
* stm32f398

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

## Getting Started

### Adding stm32f3xx-hal and other dependencies

Cargo.toml:

```toml
[package]
# ...
resolver = "2"

[dependencies]
cortex-m = "0.7.2"
cortex-m-rt = { version = "0.6.13", features = ["device"] }
# Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
panic-halt = "0.2.0"
# Replace stm32f303xc with your target chip, see next section for more info
stm32f3xx-hal = { version = "0.9.2", features = ["ld", "rt", "stm32f303xc"] }
```

We also need to tell Rust about target architecture and how to link our
executable by creating `.cargo/config`.

.cargo/config:

```toml
[target.thumbv7em-none-eabihf]
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"
```

### Selecting the right chip

This crate requires you to specify your target chip as a feature.

*Example: The STM32F3Discovery board has a STM32F303VCT6 chip according to the
[user manual][]. So you need to specify `stm32f303xc` in your `Cargo.toml`
(note that VC → xc).*

All possible chip variants are selectable via cargo features.
You can find a list [here, in the docs][chip-features].

#### Note

1. This features are mutually exclusive. Only one feature / chip variant can be
chosen.
2. You **have** to choose exactly **one** feature to build this crate at all.

[chip-features]: https://docs.rs/stm32f3xx-hal/latest/stm32f3xx_hal/#target-chip-selection

#### Background

For some of the stm32f3xx chips there are sub-variants that differ in
functionality, peripheral use and hence 'under the hood' implementation.
To allow the full use of all peripherals on certain sub-variants without
allowing for code that just doesn't run on other sub-variants, they are
distinct features that need to be specified.

[user manual]: https://www.st.com/content/ccc/resource/technical/document/user_manual/8a/56/97/63/8d/56/41/73/DM00063382.pdf/files/DM00063382.pdf/jcr:content/translations/en.DM00063382.pdf

### Basic Usage

```rust
#![no_std]
#![no_main]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

#[entry]
fn main() -> ! {
      let dp = pac::Peripherals::take().unwrap();

      let mut rcc = dp.RCC.constrain();
      let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

      let mut led = gpioe
            .pe13
            .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

      loop {
            led.toggle().unwrap();
            asm::delay(8_000_000);
      }
}
```

See the [examples folder](examples) for more example programs.

## [Changelog](CHANGELOG.md)

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile on stable Rust 1.59.0 and up. It *might*
compile with older versions but that may change in any new patch release.

<!-- This should not prevent anyone to use newer features. -->
<!-- As soon as the MSVR does not compile anymore, just bump it. -->

<!-- Don't forget to also adjust the MSVR version in `.github/workflows/ci.yml` -->

## [Contributing](CONTRIBUTING.md)

## License

Licensed under either of

* Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
