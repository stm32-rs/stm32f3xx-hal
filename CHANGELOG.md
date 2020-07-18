# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- Implement `InputPin` for `Output<OpenDrain>` pins ([#114](https://github.com/stm32-rs/stm32f3xx-hal/pull/114))
- Support for safe one-shot DMA transfers ([#86](https://github.com/stm32-rs/stm32f3xx-hal/pull/86))
- DMA support for serial reception and transmission ([#86](https://github.com/stm32-rs/stm32f3xx-hal/pull/86))

### Fixed

- `PLL` was calculated wrong for devices, which do not divide `HSI` ([#67](https://github.com/stm32-rs/stm32f3xx-hal/pull/67))

### Changed

- The system clock calculation is more fine grained now. ([#67](https://github.com/stm32-rs/stm32f3xx-hal/pull/67))
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

- The feature gate requires you to select a subvariant if possible. ([#75](https://github.com/stm32-rs/stm32f3xx-hal/pull/75))
- Split up `stm32f302` into sub-targets `stm32f302xb`,`stm32f302xc`,`stm32f302xd`,`stm32f302xe`
- Bump `stm32f3` dependency to `0.11.0` ([#97](https://github.com/stm32-rs/stm32f3xx-hal/pull/97))
- The `stm32f3` reexport is now renamed from `stm32` to `pac` ([#101](https://github.com/stm32-rs/stm32f3xx-hal/pull/101))

## [v0.4.3] - 2020-04-11

### Added

- Independent Watchdog ([#58](https://github.com/stm32-rs/stm32f3xx-hal/pull/58))

### Fixed

- Wrong default modes for debug GPIO pins ([#82](https://github.com/stm32-rs/stm32f3xx-hal/pull/82))
- Wrong calculation of HCLK prescaler, if using a prescaler value equal or
  higher than 64 ([#42](https://github.com/stm32-rs/stm32f3xx-hal/pull/42))
- UART reception error flags not cleared ([#91](https://github.com/stm32-rs/stm32f3xx-hal/pull/91))

## [v0.4.2] - 2020-03-21

### Fixed

- Wrong frequency reported by `MonoTimer` ([#56](https://github.com/stm32-rs/stm32f3xx-hal/pull/56))
- Use automatic mode with I2C autoend on `Read` ([#72](https://github.com/stm32-rs/stm32f3xx-hal/pull/72))

### Changed

- Bump `stm32f3` dependency to `0.10.0` ([#70](https://github.com/stm32-rs/stm32f3xx-hal/pull/70))

## [v0.4.1] - 2020-03-07

### Added

- Use Infallible error type for UART ([#50](https://github.com/stm32-rs/stm32f3xx-hal/pull/50))
- Implement blocking Write for UART ([#50](https://github.com/stm32-rs/stm32f3xx-hal/pull/50))
- Implement blocking Read for I2C ([#52](https://github.com/stm32-rs/stm32f3xx-hal/pull/52))

### Fixed

- Regression in v0.4.0 that set SPI to LSB-first ordering ([#60](https://github.com/stm32-rs/stm32f3xx-hal/pull/60))

## [v0.4.0] - 2019-12-27

### Added

- USB Driver for all devices except `stm32f301` and `stm32f334` as they have no
  USB peripheral. ([#24](https://github.com/stm32-rs/stm32f3xx-hal/pull/24))
- `StatefulOutputPin` and `ToggleableOutputPin` ([#25](https://github.com/stm32-rs/stm32f3xx-hal/pull/25))
- Support devices with 2-bit PLLSRC fields ([#31](https://github.com/stm32-rs/stm32f3xx-hal/pull/33))
  - This allows using 72 MHz `sysclk` on the `stm32f303`
- Analog gpio trait ([#33](https://github.com/stm32-rs/stm32f3xx-hal/pull/33))
- Add PWM Channels ([#34](https://github.com/stm32-rs/stm32f3xx-hal/pull/34))
- SPI embedded hal modes are now public ([#35](https://github.com/stm32-rs/stm32f3xx-hal/pull/18))

### Breaking changes

- Alternate gpio functions are now **only** made available for devices, which have them.
  ([#21](https://github.com/stm32-rs/stm32f3xx-hal/pull/21))
- `stm32f303` is now split into `stm32f303xd` and `stm32f303xe` as they provide
  different alternate gpio functions. `stm32f303` is still available.
- Bump `stm32f3` dependency to `0.9.0` ([#39](https://github.com/stm32-rs/stm32f3xx-hal/pull/39))

### Fixed

- Fixed wrong initialization of the SPI ([#35](https://github.com/stm32-rs/stm32f3xx-hal/pull/35))

## [v0.3.0] - 2019-08-26

### Added

- HSE and USB clock are now supported ([#18](https://github.com/stm32-rs/stm32f3xx-hal/pull/18))

### Changed

- Bump `stm32f3` version to `0.8.0` ([#19](https://github.com/stm32-rs/stm32f3xx-hal/pull/19))

## [v0.2.3] - 2019-07-07

### Fixed

- Fix timer initialization ([#17](https://github.com/stm32-rs/stm32f3xx-hal/pull/17))

## [v0.2.2] - 2019-07-06

## Fixed

- Missing `stm32f303` timers ([#16](https://github.com/stm32-rs/stm32f3xx-hal/pull/16))

## [v0.2.1] - 2019-07-06

### Added

- Fully erased pin ([#14](https://github.com/stm32-rs/stm32f3xx-hal/pull/14))

## [v0.2.0] - 2019-07-02

### Changed

- Bump `stm32f3` version to `0.7.1` ([#12](https://github.com/stm32-rs/stm32f3xx-hal/pull/12))
- Bump `embedded-hal` version to `0.2.3` ([#11](https://github.com/stm32-rs/stm32f3xx-hal/pull/11))

### Fixed

- Various peripheral mappings for some devices ([#12](https://github.com/stm32-rs/stm32f3xx-hal/pull/12))

### Breaking changes

- Switch to the `embedded-hal` v2 digital pin trait.

## [v0.1.5] - 2019-06-11

### Added

- Support for GPIO AF14 ([#6](https://github.com/stm32-rs/stm32f3xx-hal/pull/6))

## [v0.1.4] - 2019-05-04

### Fixed

- Fixed I2C address ([#4](https://github.com/stm32-rs/stm32f3xx-hal/pull/4))

## [v0.1.3] - 2019-04-12

### Added

- Implement GPIO `InputPin` traits ([#2](https://github.com/stm32-rs/stm32f3xx-hal/pull/2))

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

[Unreleased]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.4.3...HEAD
[v0.4.3]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.4.2...v0.4.3
[v0.4.2]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.4.1...v0.4.2
[v0.4.1]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.4.0...v0.4.1
[v0.4.0]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.2.3...v0.3.0
[v0.2.3]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.2.2...v0.2.3
[v0.2.2]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.2.1...v0.2.2
[v0.2.1]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.2.0...v0.2.1
[v0.2.0]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.1.5...v0.2.0
[v0.1.5]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.1.4...v0.1.5
[v0.1.4]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.1.3...v0.1.4
[v0.1.3]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.1.2...v0.1.3
[v0.1.2]: https://github.com/stm32-rs/stm32f3xx-hal/compare/v0.1.1...v0.1.2
[v0.1.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.1.1
