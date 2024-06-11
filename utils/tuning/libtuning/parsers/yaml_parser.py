# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# Parser for YAML format config file

from .parser import Parser
import yaml


class YamlParser(Parser):
    def __init__(self):
        super().__init__()

    def parse(self, config_file: str, modules: list) -> (dict, list):
        # Dummy implementation that just reads the file
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        return config, []
