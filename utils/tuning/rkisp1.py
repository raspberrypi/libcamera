#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# rkisp1.py - Tuning script for rkisp1

import sys

import libtuning as lt
from libtuning.parsers import YamlParser
from libtuning.generators import YamlOutput
from libtuning.modules.lsc import LSCRkISP1

tuner = lt.Tuner('RkISP1')
tuner.add(LSCRkISP1(
          debug=[lt.Debug.Plot],
          # This is for the actual LSC tuning, and is part of the base LSC
          # module. rkisp1's table sector sizes (16x16 programmed as mirrored
          # 8x8) are separate, and is hardcoded in its specific LSC tuning
          # module.
          sector_shape=(17, 17),

          sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
          sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),

          # This is the function that will be used to average the pixels in
          # each sector. This can also be a custom function.
          sector_average_function=lt.average.Mean(),

          # This is the function that will be used to smooth the color ratio
          # values.  This can also be a custom function.
          smoothing_function=lt.smoothing.MedianBlur(3),
          ))
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(YamlOutput())
tuner.set_output_order([LSCRkISP1])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
