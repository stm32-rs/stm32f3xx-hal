# stm32f3xx-hal

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
