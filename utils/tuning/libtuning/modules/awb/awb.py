# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Ideas On Board

import logging

from ..module import Module

from libtuning.ctt_awb import awb
import numpy as np

logger = logging.getLogger(__name__)


class AWB(Module):
    type = 'awb'
    hr_name = 'AWB (Base)'
    out_name = 'GenericAWB'

    def __init__(self, *, debug: list):
        super().__init__()

        self.debug = debug

    def do_calculation(self, images):
        logger.info('Starting AWB calculation')

        imgs = [img for img in images if img.macbeth is not None]

        gains, _, _ = awb(imgs, None, None, False)
        gains = np.reshape(gains, (-1, 3))

        return [{
                    'ct': int(v[0]),
                    'gains': [float(1.0 / v[1]), float(1.0 / v[2])]
                } for v in gains]
