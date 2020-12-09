# Codegen

This crate provides code-generation for the stm32f3xx-hal. It reads information
from an [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
database and uses that to output code that can directly be included into the
source code of the stm32f3xx-hal crate.

For more information on how the STM32CubeMX database is structured, check out
the README in the [cube-parse](https://github.com/dbrgn/cube-parse) repository.

Because by default cargo tries to use the `x86_64-unknown-linux-gnu` target,
when building `codegen`, due to what's specified in the `.cargo/config`, you
need to manually specify your host's target if it differs from that, e.g.:

```bash
$ cargo run --target x86_64-apple-darwin -- help
```

`codgen` can generate the following code:

- [GPIO mappings](#gpio-mappings)

## GPIO mappings

Running `codegen`'s `gpio` subcommand generates the `gpio!` macro
invocations at the end of `src/gpio.rs`. Re-generating those macro-invocations
is simply a matter of deleting the old ones and then executing:

```bash
$ cargo run -- gpio $cubemx_db_path >> ../src/gpio.rs
```

`$cubemx_db_path` must be the path to the `db/` directory under an
STM32CubeMX installation. With a default Linux install, this would be
`/opt/stm32cubemx/db`.

The generated `gpio!` invocations are gated by features whose names are derived
from the respective GPIO internal peripheral (IP) version:

- gpio-f302
- gpio-f303
- gpio-f303e
- gpio-f333
- gpio-f373

`codegen` collects those IP versions from the relevant GPIO IP description
files (located at `$cubemx_db_path/mcu/IP/GPIO-*.xml`). The root `<IP>` element
has a `Version` attribute with a value in the form of "STM32Fxxx_gpio_v1_0".
The feature name is constructed by dropping the parts constant between all
version strings and prepending "gpio-".

Note that the GPIO IP version names don't necessarily match the MCUs they are
used in. For example, the GPIOs in `STM32F302xB` MCUs have the IP version
"STM32F303_gpio_v1_0". The MCU features of the `stm32f3xx-hal` also select the
correct `gpio-*` features, so users generally don't have to care about these
details.
