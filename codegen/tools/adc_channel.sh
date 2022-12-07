#!/usr/bin/env bash
# Using xq to query the stm32cubemx database for adc channels
# xq beeing a wrapper of jq converting xml to a json representation internally.
#
# This is the source of the CHANNELS dict in adc_channel.py
# TODO(Sh3Rm4n): This should probably be rewritten in the codegen part.
# Though therefor the codegen should be refactored.

set -euo pipefail

echo "CHANNELS = {"
for xml in /opt/stm32cubemx/db/mcu/STM32F3*.xml; do
    echo '"'$xml'": ['
    xq '.Mcu.Pin[]
        | select(.Signal[]?."@Name"? | startswith("ADC"))
        | [."@Name", (.Signal[]? | select(."@Name" | startswith("ADC")))."@Name"]
        | @csv' --raw-output < $xml \
        | rg -v EXTI \
        | rg -v OSC32 \
        | rg -v 'jq:' \
        | sed 's/IN//g'  \
        | sort -u \
        | awk '{print "("$0"),"}'
    echo '],'
done
echo "}"
