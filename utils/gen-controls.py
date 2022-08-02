#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2019, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# gen-controls.py - Generate control definitions from YAML

import argparse
from functools import reduce
import operator
import string
import sys
import yaml


def snake_case(s):
    return ''.join([c.isupper() and ('_' + c) or c for c in s]).strip('_')


def format_description(description):
    description = description.strip('\n').split('\n')
    description[0] = '\\brief ' + description[0]
    return '\n'.join([(line and ' * ' or ' *') + line for line in description])


def get_ctrl_type(ctrl):
    ctrl_type = ctrl['type']
    ctrl_size_arr = ctrl.get('size')

    if ctrl_type == 'string':
        return 'std::string'
    elif ctrl_size_arr is not None:
        if len(ctrl_size_arr) > 0:
            # fixed-sized Span
            ctrl_span_size = reduce(operator.mul, ctrl_size_arr)
            return f"Span<const {ctrl_type}, {ctrl_span_size}>"
        else:
            # variable-sized Span
            return f"Span<const {ctrl_type}>"
    else:
        return ctrl_type


def generate_cpp(controls):
    enum_doc_start_template = string.Template('''/**
 * \\enum ${name}Enum
 * \\brief Supported ${name} values''')
    enum_doc_value_template = string.Template(''' * \\var ${value}
${description}''')
    doc_template = string.Template('''/**
 * \\var ${name}
${description}
 */''')
    def_template = string.Template('extern const Control<${type}> ${name}(${id_name}, "${name}");')
    enum_values_doc = string.Template('''/**
 * \\var ${name}Values
 * \\brief List of all $name supported values
 */''')
    enum_values_start = string.Template('''extern const std::array<const ControlValue, ${size}> ${name}Values = {''')
    enum_values_values = string.Template('''\tstatic_cast<int32_t>(${name}),''')

    ctrls_doc = []
    ctrls_def = []
    draft_ctrls_doc = []
    draft_ctrls_def = []
    ctrls_map = []

    for ctrl in controls:
        name, ctrl = ctrl.popitem()
        id_name = snake_case(name).upper()

        ctrl_type = get_ctrl_type(ctrl)

        info = {
            'name': name,
            'type': ctrl_type,
            'description': format_description(ctrl['description']),
            'id_name': id_name,
        }

        target_doc = ctrls_doc
        target_def = ctrls_def
        if ctrl.get('draft'):
            target_doc = draft_ctrls_doc
            target_def = draft_ctrls_def

        enum = ctrl.get('enum')
        if enum:
            enum_doc = []
            enum_doc.append(enum_doc_start_template.substitute(info))

            num_entries = 0
            for entry in enum:
                value_info = {
                    'name': name,
                    'value': entry['name'],
                    'description': format_description(entry['description']),
                }
                enum_doc.append(enum_doc_value_template.substitute(value_info))
                num_entries += 1

            enum_doc = '\n *\n'.join(enum_doc)
            enum_doc += '\n */'
            target_doc.append(enum_doc)

            values_info = {
                'name': info['name'],
                'size': num_entries,
            }
            target_doc.append(enum_values_doc.substitute(values_info))
            target_def.append(enum_values_start.substitute(values_info))
            for entry in enum:
                value_info = {
                    'name': entry['name']
                }
                target_def.append(enum_values_values.substitute(value_info))
            target_def.append("};")

        target_doc.append(doc_template.substitute(info))
        target_def.append(def_template.substitute(info))

        if ctrl.get('draft'):
            name = 'draft::' + name

        ctrls_map.append('\t{ ' + id_name + ', &' + name + ' },')

    return {
        'controls_doc': '\n\n'.join(ctrls_doc),
        'controls_def': '\n'.join(ctrls_def),
        'draft_controls_doc': '\n\n'.join(draft_ctrls_doc),
        'draft_controls_def': '\n\n'.join(draft_ctrls_def),
        'controls_map': '\n'.join(ctrls_map),
    }


def generate_h(controls):
    enum_template_start = string.Template('''enum ${name}Enum {''')
    enum_value_template = string.Template('''\t${name} = ${value},''')
    enum_values_template = string.Template('''extern const std::array<const ControlValue, ${size}> ${name}Values;''')
    template = string.Template('''extern const Control<${type}> ${name};''')

    ctrls = []
    draft_ctrls = []
    ids = []
    id_value = 1

    for ctrl in controls:
        name, ctrl = ctrl.popitem()
        id_name = snake_case(name).upper()

        ids.append('\t' + id_name + ' = ' + str(id_value) + ',')

        ctrl_type = get_ctrl_type(ctrl)

        info = {
            'name': name,
            'type': ctrl_type,
        }

        target_ctrls = ctrls
        if ctrl.get('draft'):
            target_ctrls = draft_ctrls

        enum = ctrl.get('enum')
        if enum:
            target_ctrls.append(enum_template_start.substitute(info))

            num_entries = 0
            for entry in enum:
                value_info = {
                    'name': entry['name'],
                    'value': entry['value'],
                }
                target_ctrls.append(enum_value_template.substitute(value_info))
                num_entries += 1
            target_ctrls.append("};")

            values_info = {
                'name': info['name'],
                'size': num_entries,
            }
            target_ctrls.append(enum_values_template.substitute(values_info))

        target_ctrls.append(template.substitute(info))
        id_value += 1

    return {
        'ids': '\n'.join(ids),
        'controls': '\n'.join(ctrls),
        'draft_controls': '\n'.join(draft_ctrls)
    }


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
