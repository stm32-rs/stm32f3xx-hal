# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

<!-- 
  If you are adding a CHANGELOG entry with a link to the PR e.g. ([#123])
  do not forget to add the corresponding URL at the end of the file:

  [#123]: https://github.com/stm32-rs/stm32f3xx-hal/pull/123
-->

## Unreleased

No changes.

## [v0.9.1] - 2022-09-07

### Added

- Add a minimal DAC driver ([#318])

### Fixed

- Missing `MosiPin` impl for `PB5` ([#322])
- Read valid data from data register even if reception of next character
  has started ([#317])

## [v0.9.0] - 2022-03-06

### Added

- Generic `into_af_push_pull<A>` and `into_af_open_drain<A>` ([#308])
- `BusClock` and `BusTimerClock` traits ([#302])
- `RccBus`, `Enable`, `Reset` traits and implementations for peripherals ([#299])
- Support cortex-m-rt `v0.7.0` but still allow `v0.6.13` ([#283])
- Make timer `InterruptTypes` fields public to be useful. ([#304])
- Add support for the internal **signature** peripheral ([#281])
- Add common derives to `Toggle`. ([#281])
- Add `is_interrupt_configured` and `configured_interrupts` function to
  `serial::Serial` and `timer::Timer`. ([#281])

### Fixed

- Fix `can` support. Can would not build for `stm32f302x6` for example.
  Also support `can` for every chip other than `stm32f301` and `stm32f318`.
  ([#283])
- Fix wrong ADC votlage regulator startup time calculation effecting
  calibration. ([#281])

### Breaking Changes

- Make `rtc` an optional feature. Without that feature `rtcc` as a dependency is
  not needed. ([#283])
- Update `rtcc` to `v0.3.0`. For a detailed changelog, go [here]. ([#314])
- Enable `rt`, `usb`, `can`, `rtc` and `ld` feature by default.
  To disable that behavior, set `default-features = false`. ([#283])
- The MSRV was bumped to 1.54 ([#308])
- Update `stm32f3` pac to v0.14.0 ([#282])
- Remove the `bxcan` re-export. ([#304])
- The LICENSE has been changed from BSD 0-clause to the familiar dual license
  MIT / Apache 2.0. This has been done, as this fits the ecosystem better and
  the dependencies where using the MIT / Apache 2.0 dual licensing already,
  which resulted into the situtation, that BSD 0-clause was not affectivly
  0-clause as MIT conditions had to be met anyways. (🧂 IANAL). ([#309])
- Renamed `Serial::raw_read` to `Serial::read_data_register`. ([#281])
- Seal `FlashExt` and `RccExt` traits. These are no longer implementable by
  a user of this crate. ([#281])
- Move ADC from a macro to a generic implementation, meaning that
  it is possible to obtain an ADC instance via `Adc::new` instead of
  `Adc::adc1`. ([#281])
- Remove `adc::ClockMode` and the clock configuration of the ADC.
  - Clock configuration effects the common ADC which inturn controls e.g. ADC1
  and ADC2, which the old API did not resemble.
  - The API provides no clock configuration for ADC peripherals yet, but
  correctly acts upon changes done through the `pac`. ([#281])
- Major rework of the ADC implementation: ([#281])
  - Add `CommonAdc` support.
  - Add internal sensor peripheral support like `TemeperaturSensor` and similar.
  - Lift restriction of the ADC implementation for `stm32f303`, though
  `stm32f373` is still not supported.
  - Enable manual configuration of the Adc peripheral of most important features
    - `ConversionMode`
    - `Sequence` length
    - `OverrunMode`
    - etc.
  - Leverage type states to:
    - Allow a `Disabled` ADC, which can be manually calibrated.
    - Allow a `OneShot` ADC implementation through `into_oneshot` with an
    optimal configuration for the `OneShot` embedded-hal trait.
  - Support every possible ADC channel.
  - Add interrupt support.

[here]: https://github.com/eldruin/rtcc-rs/blob/master/CHANGELOG.md#030---2022-02-19

## [v0.8.2] - 2021-12-14

- Add missing SPI impls for the `gpio-f303` device groups (e.g. stm32f303vc) ([#304])

## [v0.8.1] - 2021-10-27

### Fixed

- ADC Channel SMP Register mismatch. ([#291])
  - While `read()`ing a Pin mapped to channel `10` and unexpected panic happened.

## [v0.8.0] - 2021-08-16

### Added

- Readd MonoTimer. This was accidentally removed before. ([#247])
  - Different to before, frequency() and now() now do not consume the MonoTimer.
    `&self` is used instead. ([#267])
- Basic serial implementation also available for UART4 and UART5 ([#246])
- Implement serial DMA also for Serial ([#246])
- Add [`enumset`][] as a optional dependency, which allow more ergonomic functions
  regarding enums. This is especially useful for status event query functions.
  ([#253])
- As serial `Error` and serial `Events` share things in common. `TryFrom<Event>`
  and `From<Error>` is implemented for conversions. ([#253])
- Add serial character match function, with which events can be triggered for
  the set character. ([#253])
- Add receiver timeout function, which configures the serial peripheral to
  trigger an event, if nothing happened after a certain time on the serial
  receiver line. ([#253])
- Add `raw_read()` function, which does no error handling and also does not
  clear any `Event` by itself. Useful, if the error_handling has to be done in
  another context (like an interrupt rountine). ([#253])
- Introduce `Toggle`, with `On` and `Off` as a convinience wrapper around `bool`
  for configuration purposes. ([#253])
- Add an associated const to `serial::Instance` to return the corresponding
  `pac::Interrupt`-number, which is useful to `unmask()` interrupts.
  An `interrupt()` function to `Serial` was also added for convinience. ([#253])
- Add a `Serial::is_busy()` function to check, if the serial device is busy.
  Useful to block on this function, e.g. `while serial.is_busy() {}`. ([#253])
- Add `BaudTable` a convinience wrapper around `Baud` to configure the `Serial`
  to the most commen baud rates. ([#253])
- Add `Serial::detect_overrun()` function, to en- or disable the overrun
  detection of the `Serial`. If overrun is disabled (enabled by default), than
  newly arrived bytes are overwriting the current data in the read receive
  register without throwing any event. ([#253]).
- Lift generic constraint of most `Serial` method on `TxPin` and `RxPin`.
  This should make it easier to generically use the `Serial` peripheral. ([#253])
- Greatly increase coverage of `Debug` and `defmt::Format` implementations.
  Almost all important types should now be supported. ([#265])
- Add a `free()` function to the RTC implementation to retrieve the passed-in
  peripheral. ([#266])
- Implement an API to return the corresponding interrupt number of timer through
  the newly introduced `InterruptNumber` trait, where the `Interrupt` is held
  as an associated const. ([#267])
  - Depending on the timer one `Interrupt` or a bundle of `InterruptTypes` is
    returned.
  - On the bases of these interrupts, the interrupt controller (NVIC) can
    be set to mask or unmask these interrupts.
- Implement the `embedded-hal::timer::Cancel` trait for timers. ([#267])
- Add `use_pll` to `CFGR` - the clock configuration - to force to use the PLL
  source for the systemclock. Also `Clocks::pllclk()` was introduced to be able
  to check, whether PLL is used.
- Add unsafe peripheral function to access underlying peripheral ([#277])
- Implement `fmt::Write` for `Serial` ([#278])
  - This allowes using `writeln!` in combination with `Serial`.

[`enumset`]: https://crates.io/crates/enumset

### Changed

- `PXx` struct (representing a generic GPIO pin) implements `Send` and `Sync` ([#251])
- Each pin aliases (`PA0`, `PA1`, ..) are defined under `gpio` module directly.
  Re-export from gpio port sub-modules are provided for compatibility. ([#257])
- The `embedded-hal` Read implementation of `Serial` does now return `WoudBlock`
  if the peripheral is busy. Also if an `Overrun` occured the receive data
  register (RDR) is flushed to have a more consistent API. ([#253])
- Remove `cargo-metadata` as a build dependency to cut down dependencies and
  the reliance on `resolver = "2"`. ([#270])

### Fixed

- The "unproven" feature, which itself does enable the "embedded-hal/unproven"
  features, can no longer be disabled. ([#259])
  1. The "unproven" features are no longer unproven and used anywhere anyways.
  2. This crate was not building successfully without the unproven feature.
- Set the correct baud rate for chips where `USART1SW_A::PCLK` leads to a
  baud rate derived from PCLK1, rather than the ports own bus clock, PCLK2.
  ([#260])

### Breaking Changes

- Refactor CAN to use the [`bxCan`](https://github.com/stm32-rs/bxcan) crate. ([#207])
- Add support for configuring parity and stop bits in addition to baud rate for
 `Serial` with `serial::config::Config`. ([#239])
- Implement `Serial::join` which allows to re-create the serial peripheral,
  when `Serial::split` was previously called. ([#252])
- Parameterized `usb::Peripheral` and `usb::UsbType` on the pin configuration
  used ([#255])
- Add (almost) all missing serial (interrupt and status) events.
  Also rename all the event names to be more descriptive. ([#253])
- A new serial interrupt API was introduced: ([#253])
  - `listen()` and `unlisten()` are renamed to `enable_interrupt()` and
  `disable_interrupt()`.
  - `is_tc()` and other non-parametrizable functions are removed.
  - `configure_interrupt()` was added, which can be parameterized.
  - `clear_event()` was added to clear specific events.
  - `clear_events()` was added to clear all events at once.
  - `is_event_triggered()` can check if an `Event` is triggered.
  - `triggered_events` returns an `EnumSet` of triggered events.
- Change gpio interrupt API to be more in line with the new serial interrupt
  API. ([#262])
- Move EXTI interrupt management to SysCfg. ([#262])
  - Becuase EXTI interrupt confiugration could cancel out, make it more obvious
  in that SysCfg manages the interrupts, not the pin itself.
  Change `make_interrupt_source()` to `SysCfg::select_exti_interrupt_source()`.
- Change dma interrupt API to be more in line with the new serial interrupt
  API. ([#263])
- Change timer interrupt API to the same of the serial interface. ([#264])
- Make timer Events `#[non_exhaustive]`. ([#264])
- Renames timers `release()` function to `free()` to be more in line with the
  rest of this crate. ([#264])
- rtc's `Error` type and `OperationMode` and `CkMode` of adc are now `#[non_exhaustive]`.
  ([#266])
- All non-camel-case types are chaged to be consistently camel-case types.
  Types which do not follow these rules are re-exported types by `stm32f3` for
  example. ([#266])
- Adc's `SampleTime` type has been reworked and is now a consistent wrapper around
  the underlying types for `stm32f3`'s `SMP9_A` and `SMP18_A` type. ([#266])
- Rename `CkMode` to `ClockMode` ([#266])
- Rename `stm32-usbd` feature to `usb`. `stm32-usbd` is still used as a
  dependency. ([#271])
- Rework the timer implementation: ([#267])
  - `PclkSrc` trait was removed in favor of generic `Instance` trait, which
    is a common ground for all available timers.
  - `Timer::tim1` and so on are renamed to `Timer::new`
  - The implementation of the timers are corrected to better represent the
    avaibilities of timers of the hardware.
  - `Timer::new` now does not take a timeout value. This means, that the
    timer will now not be started automatically and one has to explicitly call
    `start()`.
- Rework SPI implementation: ([#273])
  - A generic `Instance` trait now represents all Spi peripherals.
  - `Spi::spi1` and so on are renamed to `Spi::new`.
  - Add SPI configuration type to be passed into `Spi::new`
- Remove public fields from `Adc` and `Rtc` ([#277])

## [v0.7.0] - 2021-06-18

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

[#322]: https://github.com/stm32-rs/stm32f3xx-hal/pull/322
[#318]: https://github.com/stm32-rs/stm32f3xx-hal/pull/318
[#317]: https://github.com/stm32-rs/stm32f3xx-hal/pull/317
[#314]: https://github.com/stm32-rs/stm32f3xx-hal/pull/314
[#309]: https://github.com/stm32-rs/stm32f3xx-hal/pull/309
[#308]: https://github.com/stm32-rs/stm32f3xx-hal/pull/308
[#304]: https://github.com/stm32-rs/stm32f3xx-hal/pull/304
[#302]: https://github.com/stm32-rs/stm32f3xx-hal/pull/302
[#299]: https://github.com/stm32-rs/stm32f3xx-hal/pull/299
[#291]: https://github.com/stm32-rs/stm32f3xx-hal/pull/291
[#283]: https://github.com/stm32-rs/stm32f3xx-hal/pull/283
[#282]: https://github.com/stm32-rs/stm32f3xx-hal/pull/282
[#281]: https://github.com/stm32-rs/stm32f3xx-hal/pull/281
[#278]: https://github.com/stm32-rs/stm32f3xx-hal/pull/278
[#277]: https://github.com/stm32-rs/stm32f3xx-hal/pull/277
[#273]: https://github.com/stm32-rs/stm32f3xx-hal/pull/273
[#271]: https://github.com/stm32-rs/stm32f3xx-hal/pull/271
[#270]: https://github.com/stm32-rs/stm32f3xx-hal/pull/270
[#267]: https://github.com/stm32-rs/stm32f3xx-hal/pull/267
[#266]: https://github.com/stm32-rs/stm32f3xx-hal/pull/266
[#265]: https://github.com/stm32-rs/stm32f3xx-hal/pull/265
[#264]: https://github.com/stm32-rs/stm32f3xx-hal/pull/264
[#263]: https://github.com/stm32-rs/stm32f3xx-hal/pull/263
[#262]: https://github.com/stm32-rs/stm32f3xx-hal/pull/262
[#260]: https://github.com/stm32-rs/stm32f3xx-hal/pull/260
[#259]: https://github.com/stm32-rs/stm32f3xx-hal/pull/259
[#257]: https://github.com/stm32-rs/stm32f3xx-hal/pull/257
[#255]: https://github.com/stm32-rs/stm32f3xx-hal/pull/255
[#253]: https://github.com/stm32-rs/stm32f3xx-hal/pull/253
[#252]: https://github.com/stm32-rs/stm32f3xx-hal/pull/252
[#251]: https://github.com/stm32-rs/stm32f3xx-hal/pull/251
[#247]: https://github.com/stm32-rs/stm32f3xx-hal/pull/247
[#246]: https://github.com/stm32-rs/stm32f3xx-hal/pull/246
[#239]: https://github.com/stm32-rs/stm32f3xx-hal/pull/239
[#238]: https://github.com/stm32-rs/stm32f3xx-hal/pull/238
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
[#207]: https://github.com/stm32-rs/stm32f3xx-hal/pull/207
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
[v0.9.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.9.1
[v0.9.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.9.0
[v0.8.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.8.1
[v0.8.1]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.8.1
[v0.8.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.8.0
[v0.7.0]: https://github.com/stm32-rs/stm32f3xx-hal/releases/tag/v0.7.0
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
