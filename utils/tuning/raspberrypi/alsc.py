# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# alsc.py - ALSC module instance for Raspberry Pi tuning scripts

import libtuning as lt
from libtuning.modules.lsc import ALSCRaspberryPi

ALSC = \
    ALSCRaspberryPi(do_color=lt.Param('do_alsc_colour', lt.Param.Mode.Optional, True),
                    luminance_strength=lt.Param('luminance_strength', lt.Param.Mode.Optional, 0.5),
                    debug=[lt.Debug.Plot],
                    sector_shape=(16, 12),
                    sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                    sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                    sector_average_function=lt.average.Mean(),
                    smoothing_function=lt.smoothing.MedianBlur(3),
                    )
