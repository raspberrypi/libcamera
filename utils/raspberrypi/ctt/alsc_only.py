#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2022, Raspberry Pi Ltd
#
# alsc tuning tool

import sys

from ctt import *
from ctt_tools import parse_input

if __name__ == '__main__':
    """
    initialise calibration
    """
    if len(sys.argv) == 1:
        print("""
    PiSP Lens Shading Camera Tuning Tool version 1.0

    Required Arguments:
    '-i' : Calibration image directory.
    '-o' : Name of output json file.

    Optional Arguments:
    '-t' : Target platform - 'pisp' or 'vc4'. Default 'vc4'
    '-c' : Config file for the CTT. If not passed, default parameters used.
    '-l' : Name of output log file. If not passed, 'ctt_log.txt' used.
              """)
        quit(0)
    else:
        """
        parse input arguments
        """
        json_output, directory, config, log_output, target = parse_input()
        if target == 'pisp':
            from ctt_pisp import json_template, grid_size
        elif target == 'vc4':
            from ctt_vc4 import json_template, grid_size

        run_ctt(json_output, directory, config, log_output, json_template, grid_size, target, alsc_only=True)
