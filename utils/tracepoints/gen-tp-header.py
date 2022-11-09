#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# gen-tp-header.py - Generate header file to contain lttng tracepoints

import datetime
import jinja2
import pathlib
import os
import sys

def main(argv):
    if len(argv) < 4:
        print(f'Usage: {argv[0]} include_build_dir output template tp_files...')
        return 1

    output = argv[2]
    template = argv[3]

    year = datetime.datetime.now().year
    path = pathlib.Path(output).absolute().relative_to(argv[1])

    source = ''
    for fname in argv[4:]:
        source += open(fname, 'r', encoding='utf-8').read() + '\n\n'

    template = jinja2.Template(open(template, 'r', encoding='utf-8').read())
    string = template.render(year=year, path=path, source=source)

    f = open(output, 'w', encoding='utf-8').write(string)

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
