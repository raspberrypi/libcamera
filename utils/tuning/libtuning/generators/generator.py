# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# generator.py - Base class for a generator to convert dict to tuning file

from pathlib import Path


class Generator(object):
    def __init__(self):
        pass

    def write(self, output_path: Path, output_dict: dict, output_order: list):
        raise NotImplementedError
