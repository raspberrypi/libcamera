#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2021, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# Extract doxygen documentation from mojom files

import argparse
import re
import sys

regex_block_start = re.compile(r'^/\*\*$')
regex_block_end = re.compile(r'^ \*/$')
regex_spdx = re.compile(r'^/\* SPDX-License-Identifier: .* \*/$')


def main(argv):

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', dest='output', metavar='file',
                        type=argparse.FileType('w', encoding='utf-8'),
                        default=sys.stdout,
                        help='Output file name (default: standard output)')
    parser.add_argument('input', type=str,
                        help='Input file name.')
    args = parser.parse_args(argv[1:])

    lines = open(args.input, 'r').readlines()
    pipeline = args.input.split('/')[-1].replace('.mojom', '')

    if not regex_spdx.match(lines[0]):
        raise Exception(f'Missing SPDX license header in {args.input}')

    data = lines[0]
    data += f'''\
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Docs file for generated {pipeline}.mojom
 *
 * This file is auto-generated. Do not edit.
 */

namespace libcamera {{

'''

    in_block = False
    comment = ''
    for lineno, line in enumerate(lines, start=1):
        if regex_block_start.match(line):
            if in_block:
                raise SyntaxError('Expected end of comment',
                                  (args.input, lineno, 1, line))
            in_block = True
            comment = line
            continue

        if regex_block_end.match(line):
            if in_block:
                comment += line
                data += comment + '\n'
            in_block = False
            continue

        if in_block:
            comment += line

    data += '} /* namespace libcamera */\n'

    args.output.write(data)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
