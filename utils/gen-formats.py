#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# gen-formats.py - Generate formats definitions from YAML

import argparse
import re
import string
import sys
import yaml


class DRMFourCC(object):
    format_regex = re.compile(r"#define (DRM_FORMAT_[A-Z0-9_]+)[ \t]+fourcc_code\(('.', '.', '.', '.')\)")
    mod_vendor_regex = re.compile(r"#define DRM_FORMAT_MOD_VENDOR_([A-Z0-9_]+)[ \t]+([0-9a-fA-Fx]+)")
    mod_regex = re.compile(r"#define ([A-Za-z0-9_]+)[ \t]+fourcc_mod_code\(([A-Z0-9_]+), ([0-9a-fA-Fx]+)\)")

    def __init__(self, filename):
        self.formats = {}
        self.vendors = {}
        self.mods = {}

        for line in open(filename, 'rb').readlines():
            line = line.decode('utf-8')

            match = DRMFourCC.format_regex.match(line)
            if match:
                format, fourcc = match.groups()
                self.formats[format] = fourcc
                continue

            match = DRMFourCC.mod_vendor_regex.match(line)
            if match:
                vendor, value = match.groups()
                self.vendors[vendor] = int(value, 0)
                continue

            match = DRMFourCC.mod_regex.match(line)
            if match:
                mod, vendor, value = match.groups()
                self.mods[mod] = (vendor, int(value, 0))
                continue

    def fourcc(self, name):
        return self.formats[name]

    def mod(self, name):
        vendor, value = self.mods[name]
        return self.vendors[vendor], value


def generate_h(formats, drm_fourcc):
    template = string.Template('constexpr PixelFormat ${name}{ __fourcc(${fourcc}), __mod(${mod}) };')

    fmts = []

    for format in formats:
        name, format = format.popitem()
        fourcc = drm_fourcc.fourcc(format['fourcc'])
        if format.get('big-endian'):
            fourcc += '| DRM_FORMAT_BIG_ENDIAN'

        data = {
            'name': name,
            'fourcc': fourcc,
            'mod': '0, 0',
        }

        mod = format.get('mod')
        if mod:
            data['mod'] = '%u, %u' % drm_fourcc.mod(mod)

        fmts.append(template.substitute(data))

    return {'formats': '\n'.join(fmts)}


def fill_template(template, data):

    template = open(template, 'rb').read()
    template = template.decode('utf-8')
    template = string.Template(template)
    return template.substitute(data)


def main(argv):

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', dest='output', metavar='file', type=str,
                        help='Output file name. Defaults to standard output if not specified.')
    parser.add_argument('input', type=str,
                        help='Input file name.')
    parser.add_argument('template', type=str,
                        help='Template file name.')
    parser.add_argument('drm_fourcc', type=str,
                        help='Path to drm_fourcc.h.')
    args = parser.parse_args(argv[1:])

    data = open(args.input, 'rb').read()
    formats = yaml.safe_load(data)['formats']
    drm_fourcc = DRMFourCC(args.drm_fourcc)

    data = generate_h(formats, drm_fourcc)
    data = fill_template(args.template, data)

    if args.output:
        output = open(args.output, 'wb')
        output.write(data.encode('utf-8'))
        output.close()
    else:
        sys.stdout.write(data)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
