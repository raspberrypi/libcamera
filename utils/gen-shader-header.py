#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2025, Bryan O'Donoghue.
#
# Author: Bryan O'Donoghue <bryan.odonoghue@linaro.org>
#
# A Python script which takes a list of shader files and converts each of them
# into a C header.
#
import sys

try:
    with open(sys.argv[2], "rb") as file:
        data = file.read()
        data_len = len(data)

        name = sys.argv[1].replace(".", "_")
        name_len = name + "_len"

        j = 0
        print("unsigned char const", name, "[] = {")
        for ch in data:
            print(f"0x{ch:02x}, ", end="")
            j = (j + 1) % 16
            if j == 0:
                print()
        if j != 0:
            print()
        print("};")

        print()
        print(f"const unsigned int {name_len}={data_len};")

except FileNotFoundError:
    print(f"File {sys.argv[2]} not found", file=sys.stderr)
except IOError:
    print(f"Unable to read {sys.argv[2]}", file=sys.stderr)
