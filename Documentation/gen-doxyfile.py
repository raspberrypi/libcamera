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


def fill_template(template, data):

    template = open(template, 'rb').read()
    template = template.decode('utf-8')
    template = string.Template(template)

    return template.substitute(data)


def main(argv):

    parser = argparse.ArgumentParser()
    parser.add_argument('-o', dest='output', metavar='file',
                        type=argparse.FileType('w', encoding='utf-8'),
                        default=sys.stdout,
                        help='Output file name (default: standard output)')
    parser.add_argument('template', metavar='doxyfile.tmpl', type=str,
                        help='Doxyfile template')
    parser.add_argument('inputs', type=str, nargs='*',
                        help='Input files')

    args = parser.parse_args(argv[1:])

    inputs = [f'"{os.path.realpath(input)}"' for input in args.inputs]
    data = fill_template(args.template, {'inputs': (' \\\n' + ' ' * 25).join(inputs)})
    args.output.write(data)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
