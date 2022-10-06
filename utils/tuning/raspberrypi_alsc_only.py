#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# raspberrypi_alsc_only.py - Tuning script for raspberrypi, ALSC only

import sys

import libtuning as lt
from libtuning.parsers import RaspberryPiParser
from libtuning.generators import RaspberryPiOutput

from raspberrypi.alsc import ALSC

tuner = lt.Tuner('Raspberry Pi (ALSC only)')
tuner.add(ALSC)
tuner.set_input_parser(RaspberryPiParser())
tuner.set_output_formatter(RaspberryPiOutput())
tuner.set_output_order([ALSC])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
