# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2025, Ideas on Board
#
# Base Lux tuning module

from ..module import Module

import logging
import numpy as np

logger = logging.getLogger(__name__)


class Lux(Module):
    type = 'lux'
    hr_name = 'Lux (Base)'
    out_name = 'GenericLux'

    def __init__(self, debug: list):
        super().__init__()

        self.debug = debug

    def calculate_lux_reference_values(self, images):
        # The lux calibration is done on a single image. For best effects, the
        # image with lux level closest to 1000 is chosen.
        imgs = [img for img in images if img.macbeth is not None]
        lux_values = [img.lux for img in imgs]
        index = lux_values.index(min(lux_values, key=lambda l: abs(1000 - l)))
        img = imgs[index]
        logger.info(f'Selected image {img.name} for lux calibration')

        if img.lux < 50:
            logger.warning(f'A Lux level of {img.lux} is very low for proper lux calibration')

        ref_y = self.calculate_y(img)
        exposure_time = img.exposure
        gain = img.againQ8_norm
        aperture = 1
        logger.info(f'RefY:{ref_y} Exposure time:{exposure_time}µs Gain:{gain} Aperture:{aperture}')
        return {'referenceY': ref_y,
                'referenceExposureTime': exposure_time,
                'referenceAnalogueGain': gain,
                'referenceDigitalGain': 1.0,
                'referenceLux': img.lux}

    def calculate_y(self, img):
        max16Bit = 0xffff
        # Average over all grey patches.
        ap_r = np.mean(img.patches[0][3::4]) / max16Bit
        ap_g = (np.mean(img.patches[1][3::4]) + np.mean(img.patches[2][3::4])) / 2 / max16Bit
        ap_b = np.mean(img.patches[3][3::4]) / max16Bit
        logger.debug(f'Averaged grey patches: Red: {ap_r}, Green: {ap_g}, Blue: {ap_b}')

        # Calculate white balance gains.
        gr = ap_g / ap_r
        gb = ap_g / ap_b
        logger.debug(f'WB gains: Red: {gr} Blue: {gb}')

        # Calculate the mean Y value of the whole image
        a_r = np.mean(img.channels[0]) * gr
        a_g = (np.mean(img.channels[1]) + np.mean(img.channels[2])) / 2
        a_b = np.mean(img.channels[3]) * gb
        y = 0.299 * a_r + 0.587 * a_g + 0.114 * a_b
        y /= max16Bit

        return y

