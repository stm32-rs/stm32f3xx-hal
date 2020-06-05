#!/usr/bin/env python3

import json
import subprocess
import sys


def run_inner(args):
    print("Running `{}`...".format(" ".join(args)))
    ret = subprocess.call(args) == 0
    print("")
    return ret


def run(mcu, cargo_cmd):
    if mcu == "":
        return run_inner(cargo_cmd)
    else:
        return run_inner(cargo_cmd + ["--features={}".format(mcu)])


def main():
    cargo_meta = json.loads(
        subprocess.check_output("cargo metadata --no-deps --format-version=1",
                                shell=True,
                                universal_newlines=True))

    crate_info = cargo_meta["packages"][0]

    features = [
        "{},rt".format(feature)
        for feature, derived in crate_info["features"].items()
        if "device-selected" in derived
    ]

    cargo_build_cmd = ['cargo', 'build', '--verbose']

    cargo_build_examples_cmd = ['cargo', 'build', '--examples', '--verbose']

    if not all(map(lambda f: run(f, cargo_build_cmd), features)):
        sys.exit(-1)

    if not all(map(lambda f: run(f, cargo_build_examples_cmd), features)):
        sys.exit(-1)


if __name__ == "__main__":
    main()
