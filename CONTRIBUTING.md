- If the MSRV (Minimal supported rust version) CI job fails, feel free to update
  it. Places are:
  - in the [ci.yml](.github/workflows/ci.yml)
  - in the [README.md][]
- For any **user** visible change, please a note, what changed in the
  [CHANGELOG.md]
  - For easier trackability a link to the corresponding PR impelemnting that
  change is appended, e.g.

```markdown
### CHANGED

- Output pins has gained the `random()` method, which randomly chooses
  the output state. ([#123]) <!-- LINK to the PR -->

<!-- ... almost at the end of the file -->
<!-- This is needed to make the link actually work! -->
[#123]: https://github.com/stm32-rs/stm32f3xx-hal/pull/123
```

- If possible, test if your changes still work with the testsuite (see
  [here](testsuite/README.md)).
