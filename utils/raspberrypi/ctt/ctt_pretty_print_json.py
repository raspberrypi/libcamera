#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright 2022 Raspberry Pi Ltd
#
# Script to pretty print a Raspberry Pi tuning config JSON structure in
# version 2.0 and later formats.

import argparse
import json
import textwrap


class Encoder(json.JSONEncoder):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.indentation_level = 0
        self.hard_break = 120
        self.custom_elems = {
            'table': 16,
            'luminance_lut': 16,
            'ct_curve': 3,
            'ccm': 3,
            'gamma_curve': 2,
            'y_target': 2,
            'prior': 2
        }

    def encode(self, o, node_key=None):
        if isinstance(o, (list, tuple)):
            # Check if we are a flat list of numbers.
            if not any(isinstance(el, (list, tuple, dict)) for el in o):
                s = ', '.join(json.dumps(el) for el in o)
                if node_key in self.custom_elems.keys():
                    # Special case handling to specify number of elements in a row for tables, ccm, etc.
                    self.indentation_level += 1
                    sl = s.split(', ')
                    num = self.custom_elems[node_key]
                    chunk = [self.indent_str + ', '.join(sl[x:x + num]) for x in range(0, len(sl), num)]
                    t = ',\n'.join(chunk)
                    self.indentation_level -= 1
                    output = f'\n{self.indent_str}[\n{t}\n{self.indent_str}]'
                elif len(s) > self.hard_break - len(self.indent_str):
                    # Break a long list with wraps.
                    self.indentation_level += 1
                    t = textwrap.fill(s, self.hard_break, break_long_words=False,
                                      initial_indent=self.indent_str, subsequent_indent=self.indent_str)
                    self.indentation_level -= 1
                    output = f'\n{self.indent_str}[\n{t}\n{self.indent_str}]'
                else:
                    # Smaller lists can remain on a single line.
                    output = f' [ {s} ]'
                return output
            else:
                # Sub-structures in the list case.
                self.indentation_level += 1
                output = [self.indent_str + self.encode(el) for el in o]
                self.indentation_level -= 1
                output = ',\n'.join(output)
                return f' [\n{output}\n{self.indent_str}]'

        elif isinstance(o, dict):
            self.indentation_level += 1
            output = []
            for k, v in o.items():
                if isinstance(v, dict) and len(v) == 0:
                    # Empty config block special case.
                    output.append(self.indent_str + f'{json.dumps(k)}: {{ }}')
                else:
                    # Only linebreak if the next node is a config block.
                    sep = f'\n{self.indent_str}' if isinstance(v, dict) else ''
                    output.append(self.indent_str + f'{json.dumps(k)}:{sep}{self.encode(v, k)}')
            output = ',\n'.join(output)
            self.indentation_level -= 1
            return f'{{\n{output}\n{self.indent_str}}}'

        else:
            return ' ' + json.dumps(o)

    @property
    def indent_str(self) -> str:
        return ' ' * self.indentation_level * self.indent

    def iterencode(self, o, **kwargs):
        return self.encode(o)


def pretty_print(in_json: dict) -> str:

    if 'version' not in in_json or \
       'target' not in in_json or \
       'algorithms' not in in_json or \
       in_json['version'] < 2.0:
        raise RuntimeError('Incompatible JSON dictionary has been provided')

    return json.dumps(in_json, cls=Encoder, indent=4, sort_keys=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=
                    'Prettify a version 2.0 camera tuning config JSON file.')
    parser.add_argument('input', type=str, help='Input tuning file.')
    parser.add_argument('output', type=str, nargs='?',
                        help='Output converted tuning file. If not provided, the input file will be updated in-place.',
                        default=None)
    args = parser.parse_args()

    with open(args.input, 'r') as f:
        in_json = json.load(f)

    out_json = pretty_print(in_json)

    with open(args.output if args.output is not None else args.input, 'w') as f:
        f.write(out_json)
