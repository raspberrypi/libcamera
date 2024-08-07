#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Generate Python bindings controls from YAML

import argparse
import string
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


def generate_py(controls, mode):
    out = ''

    vendors_class_def = []
    vendor_defs = []
    vendors = []
    for vendor, ctrl_list in controls.items():
        for ctrl in ctrl_list:
            if vendor not in vendors and vendor != 'libcamera':
                vendor_mode_str = f'{vendor.capitalize()}{mode.capitalize()}'
                vendors_class_def.append('class Py{}\n{{\n}};\n'.format(vendor_mode_str))
                vendor_defs.append('\tauto {} = py::class_<Py{}>(controls, \"{}\");'.format(vendor, vendor_mode_str, vendor))
                vendors.append(vendor)

            if vendor != 'libcamera':
                ns = 'libcamera::{}::{}::'.format(mode, vendor)
                container = vendor
            else:
                ns = 'libcamera::{}::'.format(mode)
                container = 'controls'

            out += f'\t{container}.def_readonly_static("{ctrl.name}", static_cast<const libcamera::ControlId *>(&{ns}{ctrl.name}));\n\n'

            if not ctrl.is_enum:
                continue

            cpp_enum = ctrl.name + 'Enum'

            out += '\tpy::enum_<{}{}>({}, \"{}\")\n'.format(ns, cpp_enum, container, cpp_enum)

            if mode == 'controls':
                # Adjustments for controls
                if ctrl.name == 'LensShadingMapMode':
                    prefix = 'LensShadingMapMode'
                else:
                    prefix = find_common_prefix([e.name for e in ctrl.enum_values])
            else:
                # Adjustments for properties
                prefix = find_common_prefix([e.name for e in ctrl.enum_values])

            for entry in ctrl.enum_values:
                cpp_enum = entry.name
                py_enum = entry.name[len(prefix):]

                out += '\t\t.value(\"{}\", {}{})\n'.format(py_enum, ns, cpp_enum)

            out += '\t;\n\n'

    return {'controls': out,
            'vendors_class_def': '\n'.join(vendors_class_def),
            'vendors_defs': '\n'.join(vendor_defs)}


def fill_template(template, data):
    template = open(template, 'rb').read()
    template = template.decode('utf-8')
    template = string.Template(template)
    return template.substitute(data)


def main(argv):
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

    if args.mode not in ['controls', 'properties']:
        print(f'Invalid mode option "{args.mode}"', file=sys.stderr)
        return -1

    controls = {}
    for input in args.input:
        data = yaml.safe_load(open(input, 'rb').read())
        vendor = data['vendor']
        ctrls = data['controls']
        controls[vendor] = [Control(*ctrl.popitem(), vendor) for ctrl in ctrls]

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
