# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# libtuning.py - An infrastructure for camera tuning tools

import argparse

import libtuning as lt
import libtuning.utils as utils
from libtuning.utils import eprint

from enum import Enum, IntEnum


class Color(IntEnum):
    R = 0
    GR = 1
    GB = 2
    B = 3


class Debug(Enum):
    Plot = 1


# @brief What to do with the leftover pixels after dividing them into ALSC
#        sectors, when the division gradient is uniform
# @var Float Force floating point division so all sectors divide equally
# @var DistributeFront Divide the remainder equally (until running out,
#      obviously) into the existing sectors, starting from the front
# @var DistributeBack Same as DistributeFront but starting from the back
class Remainder(Enum):
    Float = 0
    DistributeFront = 1
    DistributeBack = 2


# @brief A helper class to contain a default value for a module configuration
# parameter
class Param(object):
    # @var Required The value contained in this instance is irrelevant, and the
    #      value must be provided by the tuning configuration file.
    # @var Optional If the value is not provided by the tuning configuration
    #      file, then the value contained in this instance will be used instead.
    # @var Hardcode The value contained in this instance will be used
    class Mode(Enum):
        Required = 0
        Optional = 1
        Hardcode = 2

    # @param name Name of the parameter. Shall match the name used in the
    #        configuration file for the parameter
    # @param required Whether or not a value is required in the config
    #        parameter of get_value()
    # @param val Default value (only relevant if mode is Optional)
    def __init__(self, name: str, required: Mode, val=None):
        self.name = name
        self.__required = required
        self.val = val

    def get_value(self, config: dict):
        if self.__required is self.Mode.Hardcode:
            return self.val

        if self.__required is self.Mode.Required and self.name not in config:
            raise ValueError(f'Parameter {self.name} is required but not provided in the configuration')

        return config[self.name] if self.required else self.val

    @property
    def required(self):
        return self.__required is self.Mode.Required

    # @brief Used by libtuning to auto-generate help information for the tuning
    #        script on the available parameters for the configuration file
    # \todo Implement this
    @property
    def info(self):
        raise NotImplementedError


class Tuner(object):

    # External functions

    def __init__(self, platform_name):
        self.name = platform_name
        self.modules = []
        self.parser = None
        self.generator = None
        self.output_order = []
        self.config = {}
        self.output = {}

    def add(self, module):
        self.modules.append(module)

    def set_input_parser(self, parser):
        self.parser = parser

    def set_output_formatter(self, output):
        self.generator = output

    def set_output_order(self, modules):
        self.output_order = modules

    # @brief Convert classes in self.output_order to the instances in self.modules
    def _prepare_output_order(self):
        output_order = self.output_order
        self.output_order = []
        for module_type in output_order:
            modules = [module for module in self.modules if module.type == module_type.type]
            if len(modules) > 1:
                eprint(f'Multiple modules found for module type "{module_type.type}"')
                return False
            if len(modules) < 1:
                eprint(f'No module found for module type "{module_type.type}"')
                return False
            self.output_order.append(modules[0])

        return True

    # \todo Validate parser and generator at Tuner construction time?
    def _validate_settings(self):
        if self.parser is None:
            eprint('Missing parser')
            return False

        if self.generator is None:
            eprint('Missing generator')
            return False

        if len(self.modules) == 0:
            eprint('No modules added')
            return False

        if len(self.output_order) != len(self.modules):
            eprint('Number of outputs does not match number of modules')
            return False

        return True

    def _process_args(self, argv, platform_name):
        parser = argparse.ArgumentParser(description=f'Camera Tuning for {platform_name}')
        parser.add_argument('-i', '--input', type=str, required=True,
                            help='''Directory containing calibration images (required).
                                    Images for ALSC must be named "alsc_{Color Temperature}k_1[u].dng",
                                    and all other images must be named "{Color Temperature}k_{Lux Level}l.dng"''')
        parser.add_argument('-o', '--output', type=str, required=True,
                            help='Output file (required)')
        # It is not our duty to scan all modules to figure out their default
        # options, so simply return an empty configuration if none is provided.
        parser.add_argument('-c', '--config', type=str, default='',
                            help='Config file (optional)')
        # \todo Check if we really need this or if stderr is good enough, or if
        # we want a better logging infrastructure with log levels
        parser.add_argument('-l', '--log', type=str, default=None,
                            help='Output log file (optional)')
        return parser.parse_args(argv[1:])

    def run(self, argv):
        args = self._process_args(argv, self.name)
        if args is None:
            return -1

        if not self._validate_settings():
            return -1

        if not self._prepare_output_order():
            return -1

        if len(args.config) > 0:
            self.config, disable = self.parser.parse(args.config, self.modules)
        else:
            self.config = {'general': {}}
            disable = []

        # Remove disabled modules
        for module in disable:
            if module in self.modules:
                self.modules.remove(module)

        for module in self.modules:
            if not module.validate_config(self.config):
                eprint(f'Config is invalid for module {module.type}')
                return -1

        has_lsc = any(isinstance(m, lt.modules.lsc.LSC) for m in self.modules)
        # Only one LSC module allowed
        has_only_lsc = has_lsc and len(self.modules) == 1

        images = utils.load_images(args.input, self.config, not has_only_lsc, has_lsc)
        if images is None or len(images) == 0:
            eprint(f'No images were found, or able to load')
            return -1

        # Do the tuning
        for module in self.modules:
            out = module.process(self.config, images, self.output)
            if out is None:
                eprint(f'Module {module.name} failed to process, aborting')
                break
            self.output[module] = out

        self.generator.write(args.output, self.output, self.output_order)

        return 0
