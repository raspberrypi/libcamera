#!/usr/bin/python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2019, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# gen-controls.py - Generate control definitions from YAML

import argparse
import string
import sys
import yaml


def snake_case(s):
    return ''.join([c.isupper() and ('_' + c) or c for c in s]).strip('_')


def generate_cpp(controls):
    template = string.Template('''/**
${description}
 */
extern const Control<${type}> ${name}(${id_name}, "${name}");''')

    ctrls = []

    for ctrl in controls:
        name, ctrl = ctrl.popitem()
        id_name = snake_case(name).upper()

        description = ctrl['description'].strip('\n').split('\n')
        description[0] = '\\brief ' + description[0]
        description = '\n'.join([(line and ' * ' or ' *') + line for line in description])

        info = {
            'name': name,
            'type': ctrl['type'],
            'description': description,
            'id_name': id_name,
        }

        ctrls.append(template.substitute(info))

    return {'controls': '\n\n'.join(ctrls)}


def generate_h(controls):
    template = string.Template('''extern const Control<${type}> ${name};''')

    ctrls = []
    ids = []
    id_value = 1

    for ctrl in controls:
        name, ctrl = ctrl.popitem()
        id_name = snake_case(name).upper()

        ids.append('\t' + id_name + ' = ' + str(id_value) + ',')

        info = {
            'name': name,
            'type': ctrl['type'],
        }

        ctrls.append(template.substitute(info))
        id_value += 1

    return {'ids': '\n'.join(ids), 'controls': '\n'.join(ctrls)}


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
    args = parser.parse_args(argv[1:])

    data = open(args.input, 'rb').read()
    controls = yaml.safe_load(data)['controls']

    if args.template.endswith('.cpp.in'):
        data = generate_cpp(controls)
    elif args.template.endswith('.h.in'):
        data = generate_h(controls)
    else:
        raise RuntimeError('Unknown template type')

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
