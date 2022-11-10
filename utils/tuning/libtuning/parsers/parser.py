# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# parser.py - Base class for a parser for a specific format of config file

class Parser(object):
    def __init__(self):
        pass

    # @brief Parse a config file into a config dict
    # @details The config dict shall have one key 'general' with a dict value
    #          for general configuration options, and all other entries shall
    #          have the module as the key with its configuration options (as a
    #          dict) as the value. The config dict shall prune entries that are
    #          for modules that are not in @a modules.
    # @param config (str) Path to config file
    # @param modules (list) List of modules
    # @return (dict, list) Configuration and list of modules to disable
    def parse(self, config_file: str, modules: list) -> (dict, list):
        raise NotImplementedError
