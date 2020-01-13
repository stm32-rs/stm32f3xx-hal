# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- Use Infallible error type for UART
- Implement blocking Write for UART
- Implement blocking Read for I2C

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

- Fixed wrong initialization of the SPI ([#35](https://github.com/stm32-rs/stm32f3xx-hal/pull/18))

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
- Bump `embedded-hal` version to `0.2.1` ([#11](https://github.com/stm32-rs/stm32f3xx-hal/pull/11))

### Fixed

- Various peripheral mappings for some devices ([#12](https://github.com/stm32-rs/stm32f3xx-hal/pull/12))

### Breaking changers

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

[Unreleased]: https://github.com/stm32-rs/stm32f1xx-hal/compare/v0.4.0...HEAD
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
