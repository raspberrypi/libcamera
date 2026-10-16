#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2025, Bryan O'Donoghue
# Copyright (C) 2026, Ideas on Board Oy
#
# Generate a C header file containing hex-encoded shader sources

import argparse
import binascii
import math
import pathlib
import sys


def process_file(path, out):
    data = open(path, 'rb').read()

    hex_data = [f'0x{c:02x}' for c in data]
    var_name = path.name.replace('.', '_')

    out.write(f'static constexpr std::array<unsigned char, {len(data)}> {var_name}{{\n')

    for i in range(math.ceil(len(data) / 16)):
        out.write('\t')
        out.write(', '.join(hex_data[16 * i:16 * (i + 1)]))
        out.write(',\n')

    out.write('};\n')


def main(argv):
    parser = argparse.ArgumentParser(
        description='Generate a C header file containing hex-encoded shader sources')
    parser.add_argument('--output', '-o', metavar='file', default=sys.stdout,
                        type=argparse.FileType('w', encoding='utf-8'),
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('inputs', nargs='+', type=pathlib.Path,
                        help='Input file names.')
    args = parser.parse_args(argv[1:])

    args.output.write('''\
/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* This file is auto-generated, do not edit! */

#pragma once

#include <array>

/*
 * List the names of the shaders at the top of header for readability's sake.
 *
''')

    for path in args.inputs:
        name = path.name.replace('.', '_')
        args.output.write(f' * - {name}\n')

    args.output.write('''\
 */

/* Hex encoded shader data */
''')

    for path in args.inputs:
        process_file(path, args.output)
        args.output.write('\n')

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

