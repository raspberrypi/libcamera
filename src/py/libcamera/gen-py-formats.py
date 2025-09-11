#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Generate Python format definitions from YAML

import argparse
import string
import sys
import yaml


def generate(formats):
    fmts = []

    for format in formats:
        name, format = format.popitem()
        fmts.append(f'\t\t.def_readonly_static("{name}", &libcamera::formats::{name})')

    return {'formats': '\n'.join(fmts)}


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', dest='output', metavar='file', default=sys.stdout,
                        type=argparse.FileType('w', encoding='utf-8'),
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('input', type=argparse.FileType('rb'),
                        help='Input file name.')
    parser.add_argument('template', type=argparse.FileType('r', encoding='utf-8'),
                        help='Template file name.')
    args = parser.parse_args(argv[1:])

    formats = yaml.safe_load(args.input)['formats']

    data = generate(formats)
    data = string.Template(args.template.read()).substitute(data)

    args.output.write(data)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
