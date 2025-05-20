#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2019, Google Inc.
# Copyright (C) 2024, Jaslo Ziska
#
# Authors:
# Laurent Pinchart <laurent.pinchart@ideasonboard.com>
# Jaslo Ziska <jaslo@ziska.de>
#
# Generate gstreamer control properties from YAML

import argparse
import jinja2
import re
import sys
import yaml

from controls import Control


exposed_controls = [
    'AeEnable', 'AeMeteringMode', 'AeConstraintMode', 'AeExposureMode',
    'ExposureValue', 'ExposureTime', 'ExposureTimeMode',
    'AnalogueGain', 'AnalogueGainMode', 'AeFlickerPeriod',
    'Brightness', 'Contrast', 'AwbEnable', 'AwbMode', 'ColourGains',
    'Saturation', 'Sharpness', 'ColourCorrectionMatrix', 'ScalerCrop',
    'DigitalGain', 'AfMode', 'AfRange', 'AfSpeed', 'AfMetering', 'AfWindows',
    'LensPosition', 'Gamma',
]


def find_common_prefix(strings):
    prefix = strings[0]

    for string in strings[1:]:
        while string[:len(prefix)] != prefix and prefix:
            prefix = prefix[:len(prefix) - 1]
        if not prefix:
            break

    return prefix


def format_description(description):
    # Substitute doxygen keywords \sa (see also) and \todo
    description = re.sub(r'\\sa((?: \w+)+)',
                         lambda match: 'See also: ' + ', '.join(
                             map(kebab_case, match.group(1).strip().split(' '))
                         ) + '.', description)
    description = re.sub(r'\\todo', 'Todo:', description)

    description = description.strip().split('\n')
    return '\n'.join([
        '"' + line.replace('\\', r'\\').replace('"', r'\"') + ' "' for line in description if line
    ]).rstrip()


# Custom filter to allow indenting by a string prior to Jinja version 3.0
#
# This function can be removed and the calls to indent_str() replaced by the
# built-in indent() filter when dropping Jinja versions older than 3.0
def indent_str(s, indention):
    s += '\n'

    lines = s.splitlines()
    rv = lines.pop(0)

    if lines:
        rv += '\n' + '\n'.join(
            indention + line if line else line for line in lines
        )

    return rv


def snake_case(s):
    return ''.join([
        c.isupper() and ('_' + c.lower()) or c for c in s
    ]).strip('_')


def kebab_case(s):
    return snake_case(s).replace('_', '-')


def extend_control(ctrl):
    if ctrl.vendor != 'libcamera':
        ctrl.namespace = f'{ctrl.vendor}::'
        ctrl.vendor_prefix = f'{ctrl.vendor}-'
    else:
        ctrl.namespace = ''
        ctrl.vendor_prefix = ''

    ctrl.is_array = ctrl.size is not None

    if ctrl.is_enum:
        # Remove common prefix from enum variant names
        prefix = find_common_prefix([enum.name for enum in ctrl.enum_values])
        for enum in ctrl.enum_values:
            enum.gst_name = kebab_case(enum.name.removeprefix(prefix))

        ctrl.gtype = 'enum'
        ctrl.default = '0'
    elif ctrl.element_type == 'bool':
        ctrl.gtype = 'boolean'
        ctrl.default = 'false'
    elif ctrl.element_type == 'float':
        ctrl.gtype = 'float'
        ctrl.default = '0'
        ctrl.min = '-G_MAXFLOAT'
        ctrl.max = 'G_MAXFLOAT'
    elif ctrl.element_type == 'int32_t':
        ctrl.gtype = 'int'
        ctrl.default = '0'
        ctrl.min = 'G_MININT'
        ctrl.max = 'G_MAXINT'
    elif ctrl.element_type == 'int64_t':
        ctrl.gtype = 'int64'
        ctrl.default = '0'
        ctrl.min = 'G_MININT64'
        ctrl.max = 'G_MAXINT64'
    elif ctrl.element_type == 'uint8_t':
        ctrl.gtype = 'uchar'
        ctrl.default = '0'
        ctrl.min = '0'
        ctrl.max = 'G_MAXUINT8'
    elif ctrl.element_type == 'Rectangle':
        ctrl.is_rectangle = True
        ctrl.default = '0'
        ctrl.min = '0'
        ctrl.max = 'G_MAXINT'
    else:
        raise RuntimeError(f'The type `{ctrl.element_type}` is unknown')

    return ctrl


def main(argv):
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o', metavar='file', type=str,
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('--template', '-t', dest='template', type=str, required=True,
                        help='Template file name.')
    parser.add_argument('input', type=str, nargs='+',
                        help='Input file name.')

    args = parser.parse_args(argv[1:])

    controls = {}
    for input in args.input:
        data = yaml.safe_load(open(input, 'rb').read())

        vendor = data['vendor']
        ctrls = controls.setdefault(vendor, [])

        for ctrl in data['controls']:
            ctrl = Control(*ctrl.popitem(), vendor, mode='controls')

            if ctrl.name in exposed_controls:
                ctrls.append(extend_control(ctrl))

    data = {'controls': list(controls.items())}

    env = jinja2.Environment()
    env.filters['format_description'] = format_description
    env.filters['indent_str'] = indent_str
    env.filters['snake_case'] = snake_case
    env.filters['kebab_case'] = kebab_case
    template = env.from_string(open(args.template, 'r', encoding='utf-8').read())
    string = template.render(data)

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as output:
            output.write(string)
    else:
        sys.stdout.write(string)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
