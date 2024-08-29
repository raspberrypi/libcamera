# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas on Board
#
# Ccm module for tuning rkisp1

from .ccm import CCM


class CCMRkISP1(CCM):
    hr_name = 'Crosstalk Correction (RkISP1)'
    out_name = 'Ccm'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # We don't need anything from the config file.
    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {}

        ccms = self.do_calibration(images)
        output['ccms'] = ccms

        return output
