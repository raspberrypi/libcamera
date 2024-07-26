#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Script to convert version 1.0 Raspberry Pi camera tuning files to version 2.0.
#
# Copyright 2022 Raspberry Pi Ltd

import argparse
import json
import numpy as np
import sys

from ctt_pretty_print_json import pretty_print
from ctt_pisp import grid_size as grid_size_pisp
from ctt_pisp import json_template as json_template_pisp
from ctt_vc4 import grid_size as grid_size_vc4
from ctt_vc4 import json_template as json_template_vc4


def interp_2d(in_ls, src_w, src_h, dst_w, dst_h):

    out_ls = np.zeros((dst_h, dst_w))
    for i in range(src_h):
        out_ls[i] = np.interp(np.linspace(0, dst_w - 1, dst_w),
                              np.linspace(0, dst_w - 1, src_w),
                              in_ls[i])
    for i in range(dst_w):
        out_ls[:,i] = np.interp(np.linspace(0, dst_h - 1, dst_h),
                                np.linspace(0, dst_h - 1, src_h),
                                out_ls[:src_h, i])
    return out_ls


def convert_target(in_json: dict, target: str):

    src_w, src_h = grid_size_pisp if target == 'vc4' else grid_size_vc4
    dst_w, dst_h = grid_size_vc4 if target == 'vc4' else grid_size_pisp
    json_template = json_template_vc4 if target == 'vc4' else json_template_pisp

    # ALSC grid sizes
    alsc = next(algo for algo in in_json['algorithms'] if 'rpi.alsc' in algo)['rpi.alsc']
    for colour in ['calibrations_Cr', 'calibrations_Cb']:
        if colour not in alsc:
            continue
        for temperature in alsc[colour]:
            in_ls = np.reshape(temperature['table'], (src_h, src_w))
            out_ls = interp_2d(in_ls, src_w, src_h, dst_w, dst_h)
            temperature['table'] = np.round(out_ls.flatten(), 3).tolist()

    if 'luminance_lut' in alsc:
        in_ls = np.reshape(alsc['luminance_lut'], (src_h, src_w))
        out_ls = interp_2d(in_ls, src_w, src_h, dst_w, dst_h)
        alsc['luminance_lut'] = np.round(out_ls.flatten(), 3).tolist()

    # Denoise blocks
    for i, algo in enumerate(in_json['algorithms']):
        if list(algo.keys())[0] == 'rpi.sdn':
            in_json['algorithms'][i] = {'rpi.denoise': json_template['rpi.sdn'] if target == 'vc4' else json_template['rpi.denoise']}
            break

    # AGC mode weights
    agc = next(algo for algo in in_json['algorithms'] if 'rpi.agc' in algo)['rpi.agc']
    if 'channels' in agc:
        for i, channel in enumerate(agc['channels']):
            target_agc_metering = json_template['rpi.agc']['channels'][i]['metering_modes']
            for mode, v in channel['metering_modes'].items():
                v['weights'] = target_agc_metering[mode]['weights']
    else:
        for mode, v in agc["metering_modes"].items():
            target_agc_metering = json_template['rpi.agc']['channels'][0]['metering_modes']
            v['weights'] = target_agc_metering[mode]['weights']

    # HDR
    if target == 'pisp':
        for i, algo in enumerate(in_json['algorithms']):
            if list(algo.keys())[0] == 'rpi.hdr':
                in_json['algorithms'][i] = {'rpi.hdr': json_template['rpi.hdr']}

    return in_json


def convert_v2(in_json: dict, target: str) -> str:

    if 'version' in in_json.keys() and in_json['version'] == 1.0:
        converted = {
            'version': 2.0,
            'target': target,
            'algorithms': [{algo: config} for algo, config in in_json.items()]
        }
    else:
        converted = in_json

    # Convert between vc4 <-> pisp targets. This is a best effort thing.
    if converted['target'] != target:
        converted = convert_target(converted, target)
        converted['target'] = target

    grid_size = grid_size_vc4[0] if target == 'vc4' else grid_size_pisp[0]
    return pretty_print(converted, custom_elems={'table': grid_size, 'luminance_lut': grid_size})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=
        'Convert the format of the Raspberry Pi camera tuning file from v1.0 to v2.0 and/or the vc4 <-> pisp targets.\n')
    parser.add_argument('input', type=str, help='Input tuning file.')
    parser.add_argument('-t', '--target', type=str, help='Target platform.',
                        choices=['pisp', 'vc4'], default='vc4')
    parser.add_argument('output', type=str, nargs='?',
                        help='Output converted tuning file. If not provided, the input file will be updated in-place.',
                        default=None)
    args = parser.parse_args()

    with open(args.input, 'r') as f:
        in_json = json.load(f)

    out_json = convert_v2(in_json, args.target)

    with open(args.output if args.output is not None else args.input, 'w') as f:
        f.write(out_json)
