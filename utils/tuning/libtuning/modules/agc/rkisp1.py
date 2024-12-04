# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
#
# rkisp1.py - AGC module for tuning rkisp1

from .agc import AGC

import libtuning as lt


class AGCRkISP1(AGC):
    hr_name = 'AGC (RkISP1)'
    out_name = 'Agc'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # We don't actually need anything from the config file
    def validate_config(self, config: dict) -> bool:
        return True

    def _generate_metering_modes(self) -> dict:
        centre_weighted = [
            0, 0,  0, 0, 0,
            0, 6,  8, 6, 0,
            0, 8, 16, 8, 0,
            0, 6,  8, 6, 0,
            0, 0,  0, 0, 0
        ]

        spot = [
            0, 0,  0, 0, 0,
            0, 2,  4, 2, 0,
            0, 4, 16, 4, 0,
            0, 2,  4, 2, 0,
            0, 0,  0, 0, 0
        ]

        matrix = [1 for i in range(0, 25)]

        return {
            'MeteringCentreWeighted': centre_weighted,
            'MeteringSpot': spot,
            'MeteringMatrix': matrix
        }

    def _generate_exposure_modes(self) -> dict:
        normal = {'exposureTime': [100, 10000, 30000, 60000, 120000],
                  'gain': [2.0, 4.0, 6.0, 6.0, 6.0]}
        short = {'exposureTime': [100, 5000, 10000, 20000, 120000],
                 'gain': [2.0, 4.0, 6.0, 6.0, 6.0]}

        return {'ExposureNormal': normal, 'ExposureShort': short}

    def _generate_constraint_modes(self) -> dict:
        normal = {'lower': {'qLo': 0.98, 'qHi': 1.0, 'yTarget': 0.5}}
        highlight = {
            'lower': {'qLo': 0.98, 'qHi': 1.0, 'yTarget': 0.5},
            'upper': {'qLo': 0.98, 'qHi': 1.0, 'yTarget': 0.8}
        }

        return {'ConstraintNormal': normal, 'ConstraintHighlight': highlight}

    def _generate_y_target(self) -> list:
        return 0.5

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {}

        output['AeMeteringMode'] = self._generate_metering_modes()
        output['AeExposureMode'] = self._generate_exposure_modes()
        output['AeConstraintMode'] = self._generate_constraint_modes()
        output['relativeLuminanceTarget'] = self._generate_y_target()

        # \todo Debug functionality

        return output
