#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# gen-tp-header.py - Generate header file to contain lttng tracepoints

import datetime
import jinja2
import os
import sys

def main(argv):
    if len(argv) < 3:
        print(f'Usage: {argv[0]} output template tp_files...')
        return 1

    output = argv[1]
    template = argv[2]

    year = datetime.datetime.now().year
    path = output.replace('include/', '', 1)

    source = ''
    for fname in argv[3:]:
        source += open(fname, 'r', encoding='utf-8').read() + '\n\n'

    template = jinja2.Template(open(template, 'r', encoding='utf-8').read())
    string = template.render(year=year, path=path, source=source)

    f = open(output, 'w', encoding='utf-8').write(string)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
