#!/usr/bin/env python3

"""
This script generates the ADC channel! macro with arguents for all pins
of all chip subfamilies.

The strategy is, to get the maximum possible support for the most
finest currently possible chip subfamily distinction.

That means, that some impls for some chips are more than the actual
chip is actually capable of.
"""

import re

import subprocess

# Hacky way to create a dict variable with a bash script
# which is then saved to a file which then can be directly imported.
channel = subprocess.check_output(['bash', 'adc_channel.sh'])
with open("gen.py", "wb") as f:
    f.write(channel)

import gen

CHIP_SELECT = {
    "stm32f301x6": r".*STM32F301.(6|8|\(6-8\)).*\.xml",
    "stm32f302x6": r".*STM32F302.(6|8|\(6-8\)).*\.xml",
    "stm32f303x6": r".*STM32F303.(6|8|\(6-8\)).*\.xml",
    "stm32f302xb": r".*STM32F302.(B|C|\(B-C\)|\(8-B-C\)).*\.xml",
    "stm32f303xb": r".*STM32F303.(B|C|\(B-C\)|\(8-B-C\)).*\.xml",
    "stm32f302xd": r".*STM32F302.(D|E|\(D-E\)).*\.xml",
    "stm32f303xd": r".*STM32F303.(D|E).*\.xml",
    "stm32f328": r".*STM32F328.*\.xml",
    "stm32f358": r".*STM32F358.*\.xml",
    "stm32f398": r".*STM32F398.*\.xml",
    "stm32f373": r".*STM32F37(3|8).*\.xml",
    "stm32f334": r".*STM32F334.*\.xml",
}


def main():
    chip_post = {}

    # Split ADC into adc and channel
    for chip_name, pin_list in gen.CHANNELS.items():
        pins = []
        for pin in pin_list:
            adc_list = []
            for i in [1, 2]:
                try:
                    (adc, channel) = pin[i].split("_")
                except IndexError:
                    continue
                adc_list.append((adc, int(channel)))
            pins.append((pin[0], adc_list))
        chip_post[chip_name] = pins

    chip_results = {}

    # Group into chip categories and select chip with max entries
    # Choosing the maximum list is a compromise right now:
    # 1. The stm32cubemx database is much more fine-grained about
    #    the types of chips than our current feature is
    # 2. Only allowing the minimal subset of all chip-subfamilies
    #    is can be pretty annoying and would severly restrict the
    #    channel implementation for the stm32f303xc and st32f303xd
    #    for example.
    # 3. This goes a little against the current impl strategy,
    #    that is, allowing more than a chip might be possible to use.
    #    But the alternative of finer feature selection is a pretty
    #    big change and gives no real benefit, other than annoying
    #    the user.
    for chip, regex in CHIP_SELECT.items():
        grouping = []
        for chip_name, _ in chip_post.items():
            if re.search(regex, chip_name):
                grouping.append(chip_post[chip_name])
        grouping = list(max(grouping))
        chip_results[chip] = grouping

    # Print the resulintg dictionary in a format, compatible with the
    # `channel!` macro.
    for type, result in chip_results.items():
        type_variant_map = {
            'x6': 'x8',
            'xb': 'xc',
            'xd': 'xe',
        }
        for key, value in type_variant_map.items():
            if type[-2:] == key:
                type_variant = type[:-2] + value
                print(f'}} else if #[cfg(any(feature = "{type}", feature = "{type_variant}"))] {{')
                break
        else:
            print(f'}} else if #[cfg(feature = "{type}")] {{')
        print("    channel!([")
        for elem in result:
            line = f"        ({', '.join([str(e) for e in elem])})"
            line = str(elem).replace("'", "")
            print("        " + line + ",")
        print("    ]);")


if __name__ == "__main__":
    main()
