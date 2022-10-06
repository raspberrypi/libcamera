# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# raspberrypi_parser.py - Parser for Raspberry Pi config file format

from .parser import Parser

import json
import numbers

import libtuning.utils as utils


class RaspberryPiParser(Parser):
    def __init__(self):
        super().__init__()

    # The string in the 'disable' and 'plot' lists are formatted as
    # 'rpi.{module_name}'.
    # @brief Enumerate, as a module, @a listt if its value exists in @a dictt
    #        and it is the name of a valid module in @a modules
    def _enumerate_rpi_modules(self, listt, dictt, modules):
        for x in listt:
            name = x.replace('rpi.', '')
            if name not in dictt:
                continue
            module = utils.get_module_by_typename(modules, name)
            if module is not None:
                yield module

    def _valid_macbeth_option(self, value):
        if not isinstance(value, dict):
            return False

        if list(value.keys()) != ['small', 'show']:
            return False

        for val in value.values():
            if not isinstance(val, numbers.Number):
                return False

        return True

    def parse(self, config_file: str, modules: list) -> (dict, list):
        with open(config_file, 'r') as config_json:
            config = json.load(config_json)

        disable = []
        for module in self._enumerate_rpi_modules(config['disable'], config, modules):
            disable.append(module)
            # Remove the disabled module's config too
            config.pop(module.name)
        config.pop('disable')

        # The raspberrypi config format has 'plot' map to a list of module
        # names which should be plotted. libtuning has each module contain the
        # plot information in itself so do this conversion.

        for module in self._enumerate_rpi_modules(config['plot'], config, modules):
            # It's fine to set the value of a potentially disabled module, as
            # the object still exists at this point
            module.appendValue('debug', 'plot')
        config.pop('plot')

        # Convert the keys from module name to module instance

        new_config = {}

        for module_name in config:
            module = utils.get_module_by_type_name(modules, module_name)
            if module is not None:
                new_config[module] = config[module_name]

        new_config['general'] = {}

        if 'blacklevel' in config:
            if not isinstance(config['blacklevel'], numbers.Number):
                raise TypeError('Config "blacklevel" must be a number')
            # Raspberry Pi's ctt config has magic blacklevel value -1 to mean
            # "get it from the image metadata". Since we already do that in
            # Image, don't save it to the config here.
            if config['blacklevel'] >= 0:
                new_config['general']['blacklevel'] = config['blacklevel']

        if 'macbeth' in config:
            if not self._valid_macbeth_option(config['macbeth']):
                raise TypeError('Config "macbeth" must be a dict: {"small": number, "show": number}')
            new_config['general']['macbeth'] = config['macbeth']
        else:
            new_config['general']['macbeth'] = {'small': 0, 'show': 0}

        return new_config, disable
