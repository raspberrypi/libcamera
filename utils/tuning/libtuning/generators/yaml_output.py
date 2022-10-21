# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright 2022 Paul Elder <paul.elder@ideasonboard.com>
#
# yaml_output.py - Generate tuning file in YAML format

from .generator import Generator

from numbers import Number
from pathlib import Path

import libtuning.utils as utils


class YamlOutput(Generator):
    def __init__(self):
        super().__init__()

    def _stringify_number_list(self, listt: list):
        line_wrap = 80

        line = '[ ' + ', '.join([str(x) for x in listt]) + ' ]'
        if len(line) <= line_wrap:
            return [line]

        out_lines = ['[']
        line = ' '
        for x in listt:
            x_str = str(x)
            # If the first number is longer than line_wrap, it'll add an extra line
            if len(line) + len(x_str) > line_wrap:
                out_lines.append(line)
                line = f'  {x_str},'
                continue
            line += f' {x_str},'
        out_lines.append(line)
        out_lines.append(']')

        return out_lines

    # @return Array of lines, and boolean of if all elements were numbers
    def _stringify_list(self, listt: list):
        out_lines = []

        all_numbers = set([isinstance(x, Number) for x in listt]).issubset({True})

        if all_numbers:
            return self._stringify_number_list(listt), True

        for value in listt:
            if isinstance(value, Number):
                out_lines.append(f'- {str(value)}')
            elif isinstance(value, str):
                out_lines.append(f'- "{value}"')
            elif isinstance(value, list):
                lines, all_numbers = self._stringify_list(value)

                if all_numbers:
                    out_lines.append( f'- {lines[0]}')
                    out_lines +=     [f'  {line}' for line in lines[1:]]
                else:
                    out_lines.append( f'-')
                    out_lines += [f'  {line}' for line in lines]
            elif isinstance(value, dict):
                lines = self._stringify_dict(value)
                out_lines.append( f'- {lines[0]}')
                out_lines +=     [f'  {line}' for line in lines[1:]]

        return out_lines, False

    def _stringify_dict(self, dictt: dict):
        out_lines = []

        for key in dictt:
            value = dictt[key]

            if isinstance(value, Number):
                out_lines.append(f'{key}: {str(value)}')
            elif isinstance(value, str):
                out_lines.append(f'{key}: "{value}"')
            elif isinstance(value, list):
                lines, all_numbers = self._stringify_list(value)

                if all_numbers:
                    out_lines.append( f'{key}: {lines[0]}')
                    out_lines +=     [f'{" " * (len(key) + 2)}{line}' for line in lines[1:]]
                else:
                    out_lines.append( f'{key}:')
                    out_lines +=     [f'  {line}' for line in lines]
            elif isinstance(value, dict):
                lines = self._stringify_dict(value)
                out_lines.append( f'{key}:')
                out_lines +=     [f'  {line}' for line in lines]

        return out_lines

    def write(self, output_file: Path, output_dict: dict, output_order: list):
        out_lines = [
            '%YAML 1.1',
            '---',
            'version: 1',
            # No need to condition this, as libtuning already guarantees that
            # we have at least one module. Even if the module has no output,
            # its prescence is meaningful.
            'algorithms:'
        ]

        for module in output_order:
            out_lines.append(f'  - {module.out_name}:')

            if len(output_dict[module]) == 0:
                continue

            if not isinstance(output_dict[module], dict):
                utils.eprint(f'Error: Output of {module.type} is not a dictionary')
                continue

            lines = self._stringify_dict(output_dict[module])
            out_lines += [f'      {line}' for line in lines]

        with open(output_file, 'w', encoding='utf-8') as f:
            for line in out_lines:
                f.write(f'{line}\n')
