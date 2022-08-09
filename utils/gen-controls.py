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


class ControlEnum(object):
    def __init__(self, data):
        self.__data = data

    @property
    def description(self):
        """The enum description"""
        return self.__data.get('description')

    @property
    def name(self):
        """The enum name"""
        return self.__data.get('name')

    @property
    def value(self):
        """The enum value"""
        return self.__data.get('value')


class Control(object):
    def __init__(self, name, data):
        self.__name = name
        self.__data = data
        self.__enum_values = None
        self.__size = None

        enum_values = data.get('enum')
        if enum_values is not None:
            self.__enum_values = [ControlEnum(enum) for enum in enum_values]

        size = self.__data.get('size')
        if size is not None:
            if len(size) == 0:
                raise RuntimeError(f'Control `{self.__name}` size must have at least one dimension')

            # Compute the total number of elements in the array. If any of the
            # array dimension is a string, the array is variable-sized.
            num_elems = 1
            for dim in size:
                if type(dim) is str:
                    num_elems = 0
                    break

                dim = int(dim)
                if dim <= 0:
                    raise RuntimeError(f'Control `{self.__name}` size must have positive values only')

                num_elems *= dim

            self.__size = num_elems

    @property
    def description(self):
        """The control description"""
        return self.__data.get('description')

    @property
    def enum_values(self):
        """The enum values, if the control is an enumeration"""
        if self.__enum_values is None:
            return
        for enum in self.__enum_values:
            yield enum

    @property
    def is_enum(self):
        """Is the control an enumeration"""
        return self.__enum_values is not None

    @property
    def is_draft(self):
        """Is the control a draft control"""
        return self.__data.get('draft') is not None

    @property
    def name(self):
        """The control name (CamelCase)"""
        return self.__name

    @property
    def q_name(self):
        """The control name, qualified with a namespace"""
        ns = 'draft::' if self.is_draft else ''
        return ns + self.__name

    @property
    def type(self):
        typ = self.__data.get('type')
        size = self.__data.get('size')

        if typ == 'string':
            return 'std::string'

        if self.__size is None:
            return typ

        if self.__size:
            return f"Span<const {typ}, {self.__size}>"
        else:
            return f"Span<const {typ}>"


def snake_case(s):
    return ''.join([c.isupper() and ('_' + c) or c for c in s]).strip('_')


def format_description(description):
    description = description.strip('\n').split('\n')
    description[0] = '\\brief ' + description[0]
    return '\n'.join([(line and ' * ' or ' *') + line for line in description])


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
        id_name = snake_case(ctrl.name).upper()

        info = {
            'name': ctrl.name,
            'type': ctrl.type,
            'description': format_description(ctrl.description),
            'id_name': id_name,
        }

        target_doc = ctrls_doc
        target_def = ctrls_def
        if ctrl.is_draft:
            target_doc = draft_ctrls_doc
            target_def = draft_ctrls_def

        if ctrl.is_enum:
            enum_doc = []
            enum_doc.append(enum_doc_start_template.substitute(info))

            num_entries = 0
            for enum in ctrl.enum_values:
                value_info = {
                    'name': ctrl.name,
                    'value': enum.name,
                    'description': format_description(enum.description),
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
            for enum in ctrl.enum_values:
                value_info = {
                    'name': enum.name
                }
                target_def.append(enum_values_values.substitute(value_info))
            target_def.append("};")

        target_doc.append(doc_template.substitute(info))
        target_def.append(def_template.substitute(info))

        ctrls_map.append('\t{ ' + id_name + ', &' + ctrl.q_name + ' },')

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
        id_name = snake_case(ctrl.name).upper()

        ids.append('\t' + id_name + ' = ' + str(id_value) + ',')

        info = {
            'name': ctrl.name,
            'type': ctrl.type,
        }

        target_ctrls = ctrls
        if ctrl.is_draft:
            target_ctrls = draft_ctrls

        if ctrl.is_enum:
            target_ctrls.append(enum_template_start.substitute(info))

            num_entries = 0
            for enum in ctrl.enum_values:
                value_info = {
                    'name': enum.name,
                    'value': enum.value,
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
    controls = [Control(*ctrl.popitem()) for ctrl in controls]

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
