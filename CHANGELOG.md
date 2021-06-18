# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## Unreleased

### Added

- Make `Clocks` `ppre1()` and `ppre2()` methods public, to get the current
  Prescaler value. ([#210])
- Support for more CAN bit rates and modes. ([#186])
- Implement `into_xxx` methods for partially erased pins ([#189])
- Enable better GPIO internal resistor configuration ([#189])
- Support for GPIO output slew rate configuration ([#189])
- Support for GPIO interrupts ([#189])
- `ld` feature, which enables the memory.x generation ([#216])
- Implement `DelayMs` for `Milliseconds` and `DelayUs` for `Microseconds` ([#234])
- ADC can now be `free()`'d ([#212])
- Serial does now implement `embedded_hal::serial::{Read, Write}`.
  No `split()` necessary. ([#232])
- Serial can now listen for the "Transmission Complete" `Tc` interrupt event ([#232])
- Serial can now listen for the `Idle` interrupt event ([#238])

### Changed

- The structure of `gpio.rs` is greatly changed. Generic `Pin` struct is used
  for every GPIO pin now ([#189])

### Fixed

- Delay based on systick no longer panics ([#203]) for to high values
  and support longer delays ([#208])
- Long delay during ADC initialization ([#217])

### Breaking Changes

- The MSRV was bumped to 1.51 ([#227])
- Replace custom time based units with types defined in the [embedded-time][]
  crate ([#192])
- The `rcc` public API now expects time based units in `Megahertz`.
  If the supplied frequency cannot be converted to `Hertz` the code
  will `panic`. This will occur if the supplied `Megahertz` frequency
  cannot fit into `u32::MAX` when converting to `Hertz` ([#192])

```rust
// The supplied frequencies must be in `MHz`.
let clocks = rcc
    .cfgr
    .use_hse(8.MHz())
    .hclk(48.MHz())
    .sysclk(48.MHz())
    .pclk1(12.MHz())
    .pclk2(12.MHz())
```

- You always required to select a sub-target for target chips ([#216])
- Bump dependencies: ([#229])
  - `cortex-m` to 0.7.2
  - `cortex-m-rt` to 0.6.4
  - `defmt` to 0.2.2
  - `embedded-hal` to 0.2.5
  - `nb` to 1.0.0
  - `stm32f3` to 0.13.2
  - `stm32-usbd` to 0.6.0
- `into_afx` methods are splitted into `into_afx_push_pull` and
  `into_afx_open_drain` ([#189])
- GPIO output mode (`PushPull` or `OpenDrain`) is encoded into pin typestate
  in alternate function mode ([#189])
- GPIO internal resistor configuration is no longer encoded into pin typestate
  in input mode ([#189])
- Remove `stm32` module. Use `use stm32f3xx_hal::pac` instead.
  This module was a deprecated in [v0.5.0][] and is now subject for
  removal. ([#220])
- `Serial::uart1` ... functions are renamed to `Serial::new`. ([#212])

## [v0.6.1] - 2020-12-10

### Changed

- Removed `doc-comment` dependency ([#184])

## [v0.6.0] - 2020-12-10

### Added

- Support for 16-bit words with SPI ([#107])
- SPI support for reclock after initialization ([#98])
- Support for `stm32f302x6` and `stm32f302x8` devices ([#132])
- Support for the onboard real-time clock (RTC) ([#136])
- Enable DMA for USART on `stm32f302` devices ([#139])
- Basic CAN bus support ([#100])
- Impls for all SPI pins for all `stm32f302` sub-targets, `stm32f303`
  subtargets, `stm32f3x8` targets, `stm32f334`, and `stm32f373`
  ([#99])
- SPI4 peripheral for supported devices. ([#99])
- Support for I2C transfer of more than 255 bytes, and 0 byte write ([#154])
- Support for HSE bypass and CSS ([#156])
- Impls for missing I2C pin definitions ([#164])
- Support I2C3 ([#164])
- Support for [`defmt`][defmt] ([#172])
  - Now [defmt][] features are available.
  - Currently these are only used for panicking calls, like
    `assert!` `panic!` or `unwrap()`. These are enabled using the [defmt][]
    [filter][].
  - For now [defmt][] is mostly intended for internal development and testing
    to further reduce panicking calls in this crate.
    The support of this feature is subject to change as the development
    of [defmt][] is advancing.

### Changed

- Introduced auto-generated GPIO mappings based on the STM32CubeMX database
  ([#129])

### Fixed

- Fixed [#151] not being able to generate 72 MHz HCLK for stm32f303xc devices
  ([#152])
- Wrong I2C clock source ([#164])

### Breaking Changes

- Removed impl for `SckPin<SPI2>` for `PB13<AF5>` from `stm32f328` and
  `stm32f378` targets. ([#99])
- Removed SPI1 support for `stm32f302x6` and `stm32f302x8` sub-targets
  and `stm32f318` target. ([#99])
- This release requires 1.48, as intra-doc-links are now used internally.
  Until now, no MSRV was tracked. This has changed now. This however does
  not mean, that we guarantee any MSRV policy. It is rather for documentation
  purposes and if a new useful feature arises, we will increase the MSRV.
  ([#170])
- Removed I2C2 support for `stm32f303x6`, `stm32f303x8` and `stm32f328` targets.
  ([#164])
- `I2c::i2c1` and `I2c::i2c2` functions are renamed to `I2c::new`.
  ([#164])

## [v0.5.0] - 2020-07-21

### Added

- Implement `InputPin` for `Output<OpenDrain>` pins ([#114])
- Support for safe one-shot DMA transfers ([#86])
- DMA support for serial reception and transmission ([#86])
- ADC support for `stm32f303` devices ([#47])

### Fixed

- `PLL` was calculated wrong for devices, which do not divide `HSI` ([#67])

### Changed

- The system clock calculation is more fine grained now. ([#67])
  Now the system clock can be some value, like 14 MHz, which can not a
  be represented as a multiple of the oscillator clock:

```rust
let clocks = rcc
    .cfgr
    .use_hse(8.mhz())
    .sysclk(14.mhz())

// or
let clocks = rcc
    .cfgr
    .use_hse(32.mhz())
    .sysclk(72.mhz())
```

  This is possible through utilizing the divider, which can divide the
  external oscillator clock on most devices. Some devices have even the
  possibility to divide the internal oscillator clock.

### Breaking changes

- The feature gate requires you to select a subvariant if possible. ([#75])
- Split up `stm32f302` into sub-targets `stm32f302xb`,`stm32f302xc`,`stm32f302xd`,`stm32f302xe`
- Bump `stm32f3` dependency to `0.11.0` ([#97])
- The `stm32f3` reexport is now renamed from `stm32` to `pac` ([#101])
- The correct `stm32f3` modules are now used for the `stm32f318` and `stm32f738`
  targets. As a result, some previously (wrongly) supported peripherals have
  been removed from these targets. ([#116])

## [v0.4.3] - 2020-04-11

### Added

- Independent Watchdog ([#58])

### Fixed

- Wrong default modes for debug GPIO pins ([#82])
- Wrong calculation of HCLK prescaler, if using a prescaler value equal or
  higher than 64 ([#42])
- UART reception error flags not cleared ([#91])

## [v0.4.2] - 2020-03-21

### Fixed

- Wrong frequency reported by `MonoTimer` ([#56])
- Use automatic mode with I2C autoend on `Read` ([#72])

### Changed

- Bump `stm32f3` dependency to `0.10.0` ([#70])

## [v0.4.1] - 2020-03-07

### Added

- Use Infallible error type for UART ([#50])
- Implement blocking Write for UART ([#50])
- Implement blocking Read for I2C ([#52])

### Fixed

- Regression in v0.4.0 that set SPI to LSB-first ordering ([#60])

## [v0.4.0] - 2019-12-27

### Added

- USB Driver for all devices except `stm32f301` and `stm32f334` as they have no
  USB peripheral. ([#24])
- `StatefulOutputPin` and `ToggleableOutputPin` ([#25])
- Support devices with 2-bit PLLSRC fields ([#31])
  - This allows using 72 MHz `sysclk` on the `stm32f303`
- Analog gpio trait ([#33])
- Add PWM Channels ([#34])
- SPI embedded hal modes are now public
  ([#35])

### Breaking changes

- Alternate gpio functions are now **only** made available for devices, which
  have them. ([#21])
- `stm32f303` is now split into `stm32f303xd` and `stm32f303xe` as they provide
  different alternate gpio functions. `stm32f303` is still available.
- Bump `stm32f3` dependency to `0.9.0`
  ([#39])

### Fixed

- Fixed wrong initialization of the SPI ([#35])

## [v0.3.0] - 2019-08-26

### Added

- HSE and USB clock are now supported ([#18])

### Changed

- Bump `stm32f3` version to `0.8.0` ([#19])

## [v0.2.3] - 2019-07-07

### Fixed

- Fix timer initialization ([#17])

## [v0.2.2] - 2019-07-06

## Fixed

- Missing `stm32f303` timers ([#16])

## [v0.2.1] - 2019-07-06

### Added

- Fully erased pin ([#14])

## [v0.2.0] - 2019-07-02

### Changed

- Bump `stm32f3` version to `0.7.1` ([#12])
- Bump `embedded-hal` version to `0.2.3` ([#11])

### Fixed

- Various peripheral mappings for some devices ([#12])

### Breaking changes

- Switch to the `embedded-hal` v2 digital pin trait.

## [v0.1.5] - 2019-06-11

### Added

- Support for GPIO AF14 ([#6])

## [v0.1.4] - 2019-05-04

### Fixed

- Fixed I2C address ([#4])

## [v0.1.3] - 2019-04-12

### Added

- Implement GPIO `InputPin` traits ([#2])

## [v0.1.2] - 2019-04-06

### Added

- Support `stm32f328`, `stm32f358` and `stm32f398` devices
- Support `stm32f334` device
- Support `stm32f378` device
- Support `stm32f373` device

## [v0.1.1] - 2019-03-31

- Support `stm32f301` and `stm32f318` devices
- Support `stm32f302` device

## [v0.1.0] - 2019-03-31

- Support `stm32f303` device

[embedded-time]: https://github.com/FluenTech/embedded-time/
[defmt]: https://github.com/knurling-rs/defmt
[filter]: https://defmt.ferrous-systems.com/filtering.html

[#234]: https://github.com/stm32-rs/stm32f3xx-hal/pull/234
[#232]: https://github.com/stm32-rs/stm32f3xx-hal/pull/232
[#229]: https://github.com/stm32-rs/stm32f3xx-hal/pull/229
[#227]: https://github.com/stm32-rs/stm32f3xx-hal/pull/227
[#220]: https://github.com/stm32-rs/stm32f3xx-hal/pull/220
[#217]: https://github.com/stm32-rs/stm32f3xx-hal/pull/217
[#216]: https://github.com/stm32-rs/stm32f3xx-hal/pull/216
[#212]: https://github.com/stm32-rs/stm32f3xx-hal/pull/212
[#210]: https://github.com/stm32-rs/stm32f3xx-hal/pull/210
[#208]: https://github.com/stm32-rs/stm32f3xx-hal/pull/208
[#203]: https://github.com/stm32-rs/stm32f3xx-hal/issues/203
[#192]: https://github.com/stm32-rs/stm32f3xx-hal/pull/192
[#189]: https://github.com/stm32-rs/stm32f3xx-hal/pull/189
[#186]: https://github.com/stm32-rs/stm32f3xx-hal/pull/186
[#184]: https://github.com/stm32-rs/stm32f3xx-hal/pull/184
[#172]: https://github.com/stm32-rs/stm32f3xx-hal/pull/172
[#170]: https://github.com/stm32-rs/stm32f3xx-hal/pull/170
[#164]: https://github.com/stm32-rs/stm32f3xx-hal/pull/164
[#156]: https://github.com/stm32-rs/stm32f3xx-hal/pull/156
[#154]: https://github.com/stm32-rs/stm32f3xx-hal/pull/154
[#152]: https://github.com/stm32-rs/stm32f3xx-hal/pull/152
[#151]: https://github.com/stm32-rs/stm32f3xx-hal/issues/151
[#139]: https://github.com/stm32-rs/stm32f3xx-hal/pull/139
[#136]: https://github.com/stm32-rs/stm32f3xx-hal/pull/136
[#132]: https://github.com/stm32-rs/stm32f3xx-hal/pull/132
[#129]: https://github.com/stm32-rs/stm32f3xx-hal/pull/129
[#116]: https://github.com/stm32-rs/stm32f3xx-hal/pull/116
[#114]: https://github.com/stm32-rs/stm32f3xx-hal/pull/114
[#107]: https://github.com/stm32-rs/stm32f3xx-hal/pull/107
[#101]: https://github.com/stm32-rs/stm32f3xx-hal/pull/101
[#100]: https://github.com/stm32-rs/stm32f3xx-hal/pull/100
[#99]: https://github.com/stm32-rs/stm32f3xx-hal/pull/99
[#98]: https://github.com/stm32-rs/stm32f3xx-hal/pull/98
[#97]: https://github.com/stm32-rs/stm32f3xx-hal/pull/97
[#91]: https://github.com/stm32-rs/stm32f3xx-hal/pull/91
[#86]: https://github.com/stm32-rs/stm32f3xx-hal/pull/86
[#82]: https://github.com/stm32-rs/stm32f3xx-hal/pull/82
[#75]: https://github.com/stm32-rs/stm32f3xx-hal/pull/75
[#72]: https://github.com/stm32-rs/stm32f3xx-hal/pull/72
[#70]: https://github.com/stm32-rs/stm32f3xx-hal/pull/70
[#67]: https://github.com/stm32-rs/stm32f3xx-hal/pull/67
[#60]: https://github.com/stm32-rs/stm32f3xx-hal/pull/60
[#58]: https://github.com/stm32-rs/stm32f3xx-hal/pull/58
[#56]: https://github.com/stm32-rs/stm32f3xx-hal/pull/56
[#52]: https://github.com/stm32-rs/stm32f3xx-hal/pull/52
[#50]: https://github.com/stm32-rs/stm32f3xx-hal/pull/50
[#47]: https://github.com/stm32-rs/stm32f3xx-hal/pull/47
[#42]: https://github.com/stm32-rs/stm32f3xx-hal/pull/42
[#39]: https://github.com/stm32-rs/stm32f3xx-hal/pull/39
[#35]: https://github.com/stm32-rs/stm32f3xx-hal/pull/18
[#34]: https://github.com/stm32-rs/stm32f3xx-hal/pull/34
[#33]: https://github.com/stm32-rs/stm32f3xx-hal/pull/33
[#31]: https://github.com/stm32-rs/stm32f3xx-hal/pull/33
[#25]: https://github.com/stm32-rs/stm32f3xx-hal/pull/25
[#24]: https://github.com/stm32-rs/stm32f3xx-hal/pull/24
[#21]: https://github.com/stm32-rs/stm32f3xx-hal/pull/21
[#19]: https://github.com/stm32-rs/stm32f3xx-hal/pull/19
[#18]: https://github.com/stm32-rs/stm32f3xx-hal/pull/18
[#17]: https://github.com/stm32-rs/stm32f3xx-hal/pull/17
[#16]: https://github.com/stm32-rs/stm32f3xx-hal/pull/16
[#14]: https://github.com/stm32-rs/stm32f3xx-hal/pull/14
[#12]: https://github.com/stm32-rs/stm32f3xx-hal/pull/12
[#11]: https://github.com/stm32-rs/stm32f3xx-hal/pull/11
[#6]: https://github.com/stm32-rs/stm32f3xx-hal/pull/6
[#4]: https://github.com/stm32-rs/stm32f3xx-hal/pull/4
[#2]: https://github.com/stm32-rs/stm32f3xx-hal/pull/2

<!-- cargo-release: latest tag -->
[v0.6.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.6.1
[v0.6.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.5.0
[v0.4.3]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.4.3
[v0.4.2]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.4.2
[v0.4.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.4.1
[v0.4.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.3.0
[v0.2.3]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.2.3
[v0.2.2]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.2.2
[v0.2.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.2.0
[v0.1.5]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.1.5
[v0.1.4]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.1.4
[v0.1.3]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.1.3
[v0.1.2]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.1.2
[v0.1.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.1.1
