# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2023, Raspberry Pi Ltd
#
# colourspace_calcs.py - Colourspace matrix generation
#
# This short utility generates colour space matrices and offsets for
# inclusion in the backend_default_config.json file.

import numpy as np

BT601 = np.array([[0.299, 0.5870, 0.1140], [-0.168736, -0.331264, 0.5], [0.5, -0.418688, -0.081312]])
REC709 = np.array([[0.2126, 0.7152, 0.0722], [-0.1146, -0.3854, 0.5], [0.5, -0.4542, -0.0458]])
REC2020 = np.array([[0.2627, 0.6780, 0.0593], [-0.13963006, -0.36036994, 0.5], [0.5, -0.4597857, -0.0402143]])

colour_spaces = {"select": "default"}

def flatten(array):
    return [round(num) for num in list(array.flatten())]

def add_entry(name, M, limited):
    offsets = np.array([0, 128, 128])
    scaling = np.array([[(235 - 16) / 255, 0, 0], [0, (240 - 16) / 255, 0], [0, 0, (240 - 16) / 255]])
    if limited:
        offsets = np.array([16, 128, 128])
        M = np.matmul(scaling, M)
    Mi = np.linalg.inv(M)
    colour_spaces[name] = {}
    colour_spaces[name]["ycbcr"] = {}
    colour_spaces[name]["ycbcr"]["coeffs"] = flatten(M * 1024)
    colour_spaces[name]["ycbcr"]["offsets"] = flatten(offsets * (2 ** 18))
    colour_spaces[name]["ycbcr_inverse"] = {}
    colour_spaces[name]["ycbcr_inverse"]["coeffs"] = flatten(Mi * 1024)
    inv_offsets = np.rint(np.dot(Mi, -offsets) * (2 ** 18))
    colour_spaces[name]["ycbcr_inverse"]["offsets"] = flatten(inv_offsets)
    if inv_offsets.min() < -2 ** 26 or inv_offsets.max() >= 2 ** 26:
        print("WARNING:", name, "will overflow!")

add_entry("default", BT601, limited=False)
add_entry("jpeg", BT601, limited=False)
add_entry("smpte170m", BT601, limited=True)
add_entry("rec709", REC709, limited=True)
add_entry("rec709_full", REC709, limited=False)
add_entry("bt2020", REC2020, limited=True)
add_entry("bt2020_full", REC2020, limited=False)

def print_dict(d, indent=0):
    print("{")
    indent += 4
    for i, (k, v) in enumerate(d.items()):
        if type(v) is dict:
            print(" " * indent, f'"{k}"', ": ", end='', sep='')
            print_dict(v, indent)
        else:
            print(" " * indent, f'"{k}"', ": ", v, end='', sep='')
        print("," if i < len(d) - 1 else "")
    indent -= 4
    print(" " * indent, "}", end ='', sep='')

final_dict = {"colour_encoding": colour_spaces}
print_dict(final_dict)
print()
