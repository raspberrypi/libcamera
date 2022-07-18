#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Script to convert version 1.0 Raspberry Pi camera tuning files to version 2.0.
#
# Copyright 2022 Raspberry Pi Ltd

import argparse
import json
import sys

from ctt_pretty_print_json import pretty_print


def convert_v2(in_json: dict) -> str:

    if 'version' in in_json.keys() and in_json['version'] != 1.0:
        print(f'The JSON config reports version {in_json["version"]} that is incompatible with this tool.')
        sys.exit(-1)

    converted = {
        'version': 2.0,
        'target': 'bcm2835',
        'algorithms': [{algo: config} for algo, config in in_json.items()]
    }

    return pretty_print(converted)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=
                    'Convert the format of the Raspberry Pi camera tuning file from v1.0 to v2.0.\n')
    parser.add_argument('input', type=str, help='Input tuning file.')
    parser.add_argument('output', type=str, nargs='?',
                        help='Output converted tuning file. If not provided, the input file will be updated in-place.',
                        default=None)
    args = parser.parse_args()

    with open(args.input, 'r') as f:
        in_json = json.load(f)

    out_json = convert_v2(in_json)

    with open(args.output if args.output is not None else args.input, 'w') as f:
        f.write(out_json)
