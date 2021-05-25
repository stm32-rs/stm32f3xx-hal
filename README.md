# `stm32f3xx-hal`

[![Build Status](https://github.com/stm32-rs/stm32f3xx-hal/workflows/CI/badge.svg)](https://github.com/stm32-rs/stm32f3xx-hal/actions)
[![Crate](https://img.shields.io/crates/v/stm32f3xx-hal.svg)](https://crates.io/crates/stm32f3xx-hal)
[![Docs](https://docs.rs/stm32f3xx-hal/badge.svg)](https://docs.rs/stm32f3xx-hal)

`stm32f3xx-hal` contains a multi device hardware abstraction on top of the
peripheral access API for the STMicro STM32F3 series microcontrollers. The
selection of the MCU is done by feature gates, typically specified by board
support crates. Currently supported configurations are:

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
[dependencies]
cortex-m = "0.7.2"
cortex-m-rt = { version = "0.6.13", features = ["device"] }
# Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
panic-halt = "0.2.0"
# Replace stm32f303xc with your target chip, see next section for more info
stm32f3xx-hal = { version = "0.7.0", features = ["ld", "rt", "stm32f303xc"] }
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

#### Possible chips

[comment]: # (Any changes here should be mirrored in src/lib.rs)

Please select one of the following (`x` denotes any character in [a-z]):

* stm32f301x6
* stm32f301x8
* stm32f318x8
* stm32f302x6
* stm32f302x8
* stm32f302xb
* stm32f302xc
* stm32f302xd
* stm32f302xe
* stm32f303x6
* stm32f303x8
* stm32f303xb
* stm32f303xc
* stm32f303xd
* stm32f303xe
* stm32f328x8
* stm32f358xc
* stm32f398xe
* stm32f373x8
* stm32f373xb
* stm32f373xc
* stm32f378xc
* stm32f334x4
* stm32f334x6
* stm32f334x8

#### Background

For some of the stm32f3xx chips there are sub-variants that differ in
functionality, peripheral use and hence 'under the hood' implementation.  To
allow the full use of all peripherals on certain subvariants without
allowing for code that just doesn't run on other sub-vairants, they are
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

This crate is guaranteed to compile on stable Rust 1.51.0 and up. It *might*
compile with older versions but that may change in any new patch release.

<!-- This should not prevent anyone to use newer features. -->
<!-- As soon as the MSVR does not compile anymore, just bump it. -->

<!-- Don't forget to also adjust the MSVR version in `.github/workflows/ci.yml` -->

## License

[0-clause BSD license](LICENSE-0BSD.txt).

## Contributing

### Running Tests

Tests are run via the integration test pattern and are executed on a target
chip, rather than on a host system. First, install
[probe-run](https://crates.io/crates/probe-run) via `cargo install probe-run`.
Next, you'll need to modify `.cargo/config` to link defmt and use `probe-run`
configured for your chip. See details within the comments in that file.

Now, you can execute a test by setting your device, defmt, and any test specific
features:

```bash
cargo test --test rcc --features=stm32f303xc,defmt,rt
```

The result _always_ shows a backtrace, even in the case of success.
Exit code of 0 means that the run was successful.
