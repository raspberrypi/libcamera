#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Generate Python bindings controls from YAML

import argparse
import jinja2
import sys
import yaml

from controls import Control


def find_common_prefix(strings):
    prefix = strings[0]

    for string in strings[1:]:
        while string[:len(prefix)] != prefix and prefix:
            prefix = prefix[:len(prefix) - 1]
        if not prefix:
            break

    return prefix


def extend_control(ctrl, mode):
    if ctrl.vendor != 'libcamera':
        ctrl.klass = ctrl.vendor
        ctrl.namespace = f'{ctrl.vendor}::'
    else:
        ctrl.klass = mode
        ctrl.namespace = ''

    if not ctrl.is_enum:
        return ctrl

    if mode == 'controls':
        # Adjustments for controls
        if ctrl.name == 'LensShadingMapMode':
            prefix = 'LensShadingMapMode'
        else:
            prefix = find_common_prefix([e.name for e in ctrl.enum_values])
    else:
        # Adjustments for properties
        prefix = find_common_prefix([e.name for e in ctrl.enum_values])

    for enum in ctrl.enum_values:
        enum.py_name = enum.name[len(prefix):]

    return ctrl


def main(argv):
    headers = {
        'controls': 'control_ids.h',
        'properties': 'property_ids.h',
    }

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', '-m', type=str, required=True,
                        help='Mode is either "controls" or "properties"')
    parser.add_argument('--output', '-o', metavar='file', type=str,
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('--template', '-t', type=str, required=True,
                        help='Template file name.')
    parser.add_argument('input', type=str, nargs='+',
                        help='Input file name.')
    args = parser.parse_args(argv[1:])

    if not headers.get(args.mode):
        print(f'Invalid mode option "{args.mode}"', file=sys.stderr)
        return -1

    controls = []
    vendors = []

    for input in args.input:
        data = yaml.safe_load(open(input, 'rb').read())

        vendor = data['vendor']
        if vendor != 'libcamera':
            vendors.append(vendor)

        for ctrl in data['controls']:
            ctrl = Control(*ctrl.popitem(), vendor, args.mode)
            controls.append(extend_control(ctrl, args.mode))

    data = {
        'mode': args.mode,
        'header': headers[args.mode],
        'vendors': vendors,
        'controls': controls,
    }

    env = jinja2.Environment()
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
