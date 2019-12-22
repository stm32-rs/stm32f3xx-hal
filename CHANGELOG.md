# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

- USB Driver for all devices except `stm32f301` and `stm32f334` as they have no
  USB peripheral. ([#24](https://github.com/stm32-rs/stm32f3xx-hal/pull/24))
- `StatefulOutputPin` and `ToggleableOutputPin` ([#25](https://github.com/stm32-rs/stm32f3xx-hal/pull/25))
- Support devices with 2-bit PLLSRC fields
  - This allow using 72 MHz `sysclk` on the `stm32f303`
- Analog gpio trait ([#33](https://github.com/stm32-rs/stm32f3xx-hal/pull/33))
- Add PWM Channels ([#34](https://github.com/stm32-rs/stm32f3xx-hal/pull/34))

### Breaking changes

- Alternate gpio functions are now made only available for devices, which have them.
  ([#21](https://github.com/stm32-rs/stm32f3xx-hal/pull/21))
- `stm32f303` is now split into `stm32f303xd` and `stm32f303xe` as they provide
  different alternate gpio functions. `stm32f303` is still available.


## [v0.3.0] - 2019-08-26

### Added

- HSE and USB clock are now suppported ([#18](https://github.com/stm32-rs/stm32f3xx-hal/pull/18))

### Changed

- Bump `stm32f3` version to `0.8.0` ([#19](https://github.com/stm32-rs/stm32f3xx-hal/pull/19))

## [v0.2.3] - 2019-07-07

### Fixed

- Fix timer initialization ([#17](https://github.com/stm32-rs/stm32f3xx-hal/pull/17))

## [v0.2.2] - 2019-07-06

## Fix

- Missing `stm32f303` timers ([#16](https://github.com/stm32-rs/stm32f3xx-hal/pull/16))

## [v0.2.1] - 2019-07-06

### Added

- Fully erased pin ([#14](https://github.com/stm32-rs/stm32f3xx-hal/pull/14))

## [v0.2.0] - 2019-07-02

### Changed

- Bump `stm32f3` version to `0.7.1` ([#12](https://github.com/stm32-rs/stm32f3xx-hal/pull/12))
- BUmp `embedded-hal` version to `0.2.1` ([#11](https://github.com/stm32-rs/stm32f3xx-hal/pull/11))

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

- Implement GPIO InputPin traits ([#2](https://github.com/stm32-rs/stm32f3xx-hal/pull/2))
