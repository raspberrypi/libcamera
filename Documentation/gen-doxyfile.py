#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2024, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Generate Doxyfile from a template

import argparse
import os
import string
import sys


def main(argv):

    parser = argparse.ArgumentParser()
    parser.add_argument('-o', dest='output', metavar='file',
                        type=argparse.FileType('w', encoding='utf-8'),
                        default=sys.stdout,
                        help='Output file name (default: standard output)')
    parser.add_argument('template', metavar='doxyfile.tmpl',
                        type=argparse.FileType('r', encoding='utf-8'),
                        help='Doxyfile template')
    parser.add_argument('inputs', type=str, nargs='*',
                        help='Input files')

    args = parser.parse_args(argv[1:])

    inputs = [f'"{os.path.realpath(input)}"' for input in args.inputs]
    data = string.Template(args.template.read()).substitute({
        'inputs': (' \\\n' + ' ' * 25).join(inputs),
    })
    args.output.write(data)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
