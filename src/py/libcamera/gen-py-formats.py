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


def fill_template(template, data):
    with open(template, encoding='utf-8') as f:
        template = f.read()

    template = string.Template(template)
    return template.substitute(data)


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', dest='output', metavar='file', type=str,
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('input', type=str,
                        help='Input file name.')
    parser.add_argument('template', type=str,
                        help='Template file name.')
    args = parser.parse_args(argv[1:])

    with open(args.input, encoding='utf-8') as f:
        formats = yaml.safe_load(f)['formats']

    data = generate(formats)
    data = fill_template(args.template, data)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(data)
    else:
        sys.stdout.write(data)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
