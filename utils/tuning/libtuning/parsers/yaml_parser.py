# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# yaml_parser.py - Parser for YAML format config file

from .parser import Parser


class YamlParser(Parser):
    def __init__(self):
        super().__init__()

    # \todo Implement this (it's fine for now as we don't need a config for
    # rkisp1 LSC, which is the only user of this so far)
    def parse(self, config_file: str, modules: list) -> (dict, list):
        return {}, []
