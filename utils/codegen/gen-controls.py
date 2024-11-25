#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2019, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Generate control definitions from YAML

import argparse
import jinja2
import os
import sys
import yaml

from controls import Control


def snake_case(s):
    return ''.join([c.isupper() and ('_' + c) or c for c in s]).strip('_')


def format_description(description):
    description = description.strip('\n').split('\n')
    for i in range(1, len(description)):
        line = description[i]
        description[i] = (line and ' * ' or ' *') + line
    return '\n'.join(description)


def extend_control(ctrl, id, ranges):
    ctrl.id = ranges[ctrl.vendor] + id + 1

    if ctrl.vendor != 'libcamera':
        ctrl.namespace = f'{ctrl.vendor}::'
    else:
        ctrl.namespace = ''

    return ctrl


def main(argv):

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', '-m', type=str, required=True, choices=['controls', 'properties'],
                        help='Mode of operation')
    parser.add_argument('--output', '-o', metavar='file', type=str,
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('--ranges', '-r', type=str, required=True,
                        help='Control id range reservation file.')
    parser.add_argument('--template', '-t', dest='template', type=str, required=True,
                        help='Template file name.')
    parser.add_argument('input', type=str, nargs='+',
                        help='Input file name.')

    args = parser.parse_args(argv[1:])

    ranges = {}
    with open(args.ranges, 'rb') as f:
        data = open(args.ranges, 'rb').read()
        ranges = yaml.safe_load(data)['ranges']

    controls = {}
    for input in args.input:
        data = yaml.safe_load(open(input, 'rb').read())

        vendor = data['vendor']
        if vendor not in ranges.keys():
            raise RuntimeError(f'Control id range is not defined for vendor {vendor}')

        ctrls = controls.setdefault(vendor, [])

        for i, ctrl in enumerate(data['controls']):
            ctrl = Control(*ctrl.popitem(), vendor, args.mode)
            ctrls.append(extend_control(ctrl, i, ranges))

    # Sort the vendors by range numerical value
    controls = [[vendor, ctrls] for vendor, ctrls in controls.items()]
    controls.sort(key=lambda item: ranges[item[0]])

    filename = {
        'controls': 'control_ids',
        'properties': 'property_ids',
    }[args.mode]

    data = {
        'filename': filename,
        'mode': args.mode,
        'controls': controls,
    }

    env = jinja2.Environment()
    env.filters['format_description'] = format_description
    env.filters['snake_case'] = snake_case
    template = env.from_string(open(args.template, 'r', encoding='utf-8').read())
    string = template.render(data)

    if args.output:
        output = open(args.output, 'w', encoding='utf-8')
        output.write(string)
        output.close()
    else:
        sys.stdout.write(string)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
