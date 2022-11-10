# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# module.py - Base class for algorithm-specific tuning modules


# @var type Type of the module. Defined in the base module.
# @var out_name The key that will be used for the algorithm in the algorithms
#               dictionary in the tuning output file
# @var hr_name Human-readable module name, mostly for debugging
class Module(object):
    type = 'base'
    hr_name = 'Base Module'
    out_name = 'GenericAlgorithm'

    def __init__(self):
        pass

    def validate_config(self, config: dict) -> bool:
        raise NotImplementedError

    # @brief Do the module's processing
    # @param config Full configuration from the input configuration file
    # @param images List of images to process
    # @param outputs The outputs of all modules that were executed before this
    #                module. Note that this is an input parameter, and the
    #                output of this module should be returned directly
    # @return Result of the module's processing. It may be empty. None
    #         indicates failure and that the result should not be used.
    def process(self, config: dict, images: list, outputs: dict) -> dict:
        raise NotImplementedError
