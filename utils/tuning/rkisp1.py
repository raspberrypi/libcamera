#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas On Board
#
# Tuning script for rkisp1

import logging
import sys

import coloredlogs
import libtuning as lt
from libtuning.generators import YamlOutput
from libtuning.modules.agc import AGCRkISP1
from libtuning.modules.awb import AWBRkISP1
from libtuning.modules.ccm import CCMRkISP1
from libtuning.modules.lsc import LSCRkISP1
from libtuning.modules.lux import LuxRkISP1
from libtuning.modules.static import StaticModule
from libtuning.parsers import YamlParser

coloredlogs.install(level=logging.INFO, fmt='%(name)s %(levelname)s %(message)s')

agc = AGCRkISP1(debug=[lt.Debug.Plot])
awb = AWBRkISP1(debug=[lt.Debug.Plot])
blc = StaticModule('BlackLevelCorrection')
ccm = CCMRkISP1(debug=[lt.Debug.Plot])
color_processing = StaticModule('ColorProcessing')
filter = StaticModule('Filter')
gamma_out = StaticModule('GammaOutCorrection', {'gamma': 2.2})
lsc = LSCRkISP1(debug=[lt.Debug.Plot],
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
                smoothing_function=lt.smoothing.MedianBlur(3),)
lux = LuxRkISP1(debug=[lt.Debug.Plot])

tuner = lt.Tuner('RkISP1')
tuner.add([agc, awb, blc, ccm, color_processing, filter, gamma_out, lsc, lux])
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(YamlOutput())

# Bayesian AWB uses the lux value, so insert the lux algorithm before AWB.
tuner.set_output_order([agc, lux, awb, blc, ccm, color_processing,
                        filter, gamma_out, lsc])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
