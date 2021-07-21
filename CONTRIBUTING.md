# CONTRIBUTING.md

This document explains some concepts used for this repository and answers some
questions for contributors.

## Abstracting a new peripheral

- Useful documentations are
  - STM datasheets - a list of these can be found in the
    [`stm32f3`][device-support] README.
  - Documentation of [`stm32f3`](https://docs.rs/stm32f3/latest/stm32f3/)
    itself.
  - The HTML table overview of
    [all documented registers](https://stm32-rs.github.io/stm32-rs/).
- While designing, take inspiration of existing abstraction. For example from
  the existing `stm32` abstractions ([`stm32f1xx`], [`stm32f4xx`],
  [`stm32f7xx`], ...) or the [`nrf-hal`]
- Incomplete abstraction of peripherals are OK!
  - Reading through the documentation can be daunting.
  - Rather concentrate on a few features you want to support.
  - Think about extensibility.
    - If you know your implementation is incomplete, leave it in a state,
      where it is easy to at least add the most common features afterwards
      without having to introduce [Breaking Changes](#breaking-changes).
      For example use [`#[non_exhaustive]`][non-exhaustive] where it does make
      sense.

[device-support]: https://github.com/stm32-rs/stm32-rs-nightlies/tree/master/stm32f3#supported-devices
[non-exhaustive]: https://rust-lang.github.io/rfcs/2008-non-exhaustive.html
[`stm32f1xx`]: https://github.com/stm32-rs/stm32f1xx-hal
[`stm32f4xx`]: https://github.com/stm32-rs/stm32f4xx-hal
[`stm32f7xx`]: https://github.com/stm32-rs/stm32f7xx-hal
[`nrf-hal`]: https://github.com/nrf-rs/nrf-hal

## Breaking Changes

- Breaking changes are defined [here][breaking]
- Do **not fear** to introduce a breaking change!
  - This crate is a long way of being stable API wise (v1.0.0) and the best
  practices in the embedded ecosystem are still evolving.
  Also, introducing new useful compiler features like [`const-generics`] means
  that there is no other way, then introducing them with a breaking change.
  (_[`min_const_generics`] are already usable_)

[breaking]: https://doc.rust-lang.org/cargo/reference/semver.html
[`const-generics`]: https://rust-lang.github.io/rfcs/2000-const-generics.html
[`min_const_generics`]: https://blog.rust-lang.org/2021/02/26/const-generics-mvp-beta.html

## Test your changes

If possible, test if your changes still work with the testsuite (see
[here](testsuite/README.md)).

This for now requires a stm32f3-discovery board, which you might not have.
[Adjustments](testsuite/README.md#using-a-different-board) can be made to be
able to run it on your board.

If you are not sure and / or want to test your changes on the full testsuite
either way, ask a contributor who should have a board available to test your
changes on. (In the future, there might also be a job running, which automatically
tests your changes on a physical board. 🚀)

## CHANGELOG entry

For any **user** visible change, please a note, what changed in the [CHANGELOG](CHANGELOG.md)

For easier traceability, a link to the corresponding PR implementing that
change is appended, e.g.

```markdown
### CHANGED

- Output pins have gained the `random()` method, which randomly chooses
  the output state. ([#123]) <!-- LINK to the PR -->

<!-- ... almost at the end of the file -->
<!-- This is needed to make the link actually work! -->
[#123]: https://github.com/stm32-rs/stm32f3xx-hal/pull/123
```

## MSVR Job is failing

If the MSRV ("minimal supported rust version") CI job fails, feel free to update
it. Places are:

- in the [ci.yml](.github/workflows/ci.yml)
- in the [README](README.md)

## API Design

Some design patterns to take into consideration, while introducing crafting
APIs.

_This part is written without much research and out of the position
of the current experience of the author / maintainer, so take it with a grain of
salt 🧂_

### Embedded Rust designing patterns

There are some designing patterns written down in "[The Embedded Rust Book]".
This crate does not follow them for 100%, but the aim is to do so!

[The Embedded Rust Book]: https://docs.rust-embedded.org/book/design-patterns/hal/checklist.html

### Principle of least surprise

If adding a new feature does mean designing a new API try to follow the
["Principle of least surprise"][POLS]. The type system should guide the user
towards the right usage of the API and if that is not possible, try to document
common pitfalls and surprising behavior.

[POLS]: https://en.wikipedia.org/wiki/Principle_of_least_astonishment

### Misc

- Add `#[derive(Debug)]` and `#[cfg_attr(feature = "defmt",
  derive(defmt::Format))]` to new `struct`s and `enum`s where possible. And add
  `#[derive(Copy, Clone)]` for `enum`s and small `struct`s, which is not bound
  to a resource (like a peripheral) where leveraging the move semantics does not
  make sense.

### Parametrization before nameability

E.g. don't introduce a bool for every Enum variant `fn is_enum_variant_1()`, `fn
is_enum_variant_2()`, `...`, rather use `fn is_enum_variant(enum: Enum)` if
possible.
