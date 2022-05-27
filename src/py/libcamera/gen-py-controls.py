#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Generate Python bindings controls from YAML

import argparse
import string
import sys
import yaml


def find_common_prefix(strings):
    prefix = strings[0]

    for string in strings[1:]:
        while string[:len(prefix)] != prefix and prefix:
            prefix = prefix[:len(prefix) - 1]
        if not prefix:
            break

    return prefix


def generate_py(controls, mode):
    out = ''

    for ctrl in controls:
        name, ctrl = ctrl.popitem()

        if ctrl.get('draft'):
            ns = 'libcamera::{}::draft::'.format(mode)
            container = 'draft'
        else:
            ns = 'libcamera::{}::'.format(mode)
            container = 'controls'

        out += f'\t{container}.def_readonly_static("{name}", static_cast<const libcamera::ControlId *>(&{ns}{name}));\n\n'

        enum = ctrl.get('enum')
        if not enum:
            continue

        cpp_enum = name + 'Enum'

        out += '\tpy::enum_<{}{}>({}, \"{}\")\n'.format(ns, cpp_enum, container, cpp_enum)

        if mode == 'controls':
            # Adjustments for controls
            if name == 'LensShadingMapMode':
                prefix = 'LensShadingMapMode'
            elif name == 'SceneFlicker':
                # If we strip the prefix, we would get '50Hz', which is illegal name
                prefix = ''
            else:
                prefix = find_common_prefix([e['name'] for e in enum])
        else:
            # Adjustments for properties
            prefix = find_common_prefix([e['name'] for e in enum])

        for entry in enum:
            cpp_enum = entry['name']
            py_enum = entry['name'][len(prefix):]

            out += '\t\t.value(\"{}\", {}{})\n'.format(py_enum, ns, cpp_enum)

        out += '\t;\n\n'

    return {'controls': out}


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
    parser.add_argument('--mode', type=str, required=True,
                        help='Mode is either "controls" or "properties"')
    args = parser.parse_args(argv[1:])

    if args.mode not in ['controls', 'properties']:
        print(f'Invalid mode option "{args.mode}"', file=sys.stderr)
        return -1

    data = open(args.input, 'rb').read()
    controls = yaml.safe_load(data)['controls']

    data = generate_py(controls, args.mode)

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
