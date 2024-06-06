#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2022, Raspberry Pi (Trading) Limited
#
# alsc_only.py - alsc tuning tool

import sys

from ctt_pisp import json_template, grid_size, target
from ctt_run import run_ctt
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
    '-c' : Config file for the CTT. If not passed, default parameters used.
    '-l' : Name of output log file. If not passed, 'ctt_log.txt' used.
              """)
        quit(0)
    else:
        """
        parse input arguments
        """
        json_output, directory, config, log_output = parse_input()
        run_ctt(json_output, directory, config, log_output, json_template, grid_size, target, alsc_only=True)
