#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Generate formats definitions from YAML

import argparse
import jinja2
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


def generate_formats(formats, drm_fourcc):
    fmts = []

    for format in formats:
        name, format = format.popitem()
        fourcc = drm_fourcc.fourcc(format['fourcc'])

        data = {
            'name': name,
            'fourcc': fourcc,
            'mod': '0, 0',
            'big_endian': format.get('big_endian'),
        }

        mod = format.get('mod')
        if mod:
            data['mod'] = '%u, %u' % drm_fourcc.mod(mod)

        fmts.append(data)

    return fmts


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

    env = jinja2.Environment()
    template = env.from_string(open(args.template, 'r', encoding='utf-8').read())
    string = template.render({
        'formats': generate_formats(formats, drm_fourcc),
    })

    if args.output:
        output = open(args.output, 'wb')
        output.write(string.encode('utf-8'))
        output.close()
    else:
        sys.stdout.write(string)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
