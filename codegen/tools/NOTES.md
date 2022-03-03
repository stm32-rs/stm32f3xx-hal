# ADC Channels

The ADC channel approach did not use the current codegen code, as
it was more convenient to query the xml database with a commandline tool,
to get an idea how it was structured.

This code **should** be integrated into the codegen rust source in the future.
But to make this happen codegen has to be refactored, as it is heavily
tailored to the `IP` resolution right now.

What really should happen though, is to integrate all those features into
[**`cube-parse`**](https://github.com/stm32-rs/cube-parse/)
and only have a shim of processing code, which tailors the output to our
macros.
