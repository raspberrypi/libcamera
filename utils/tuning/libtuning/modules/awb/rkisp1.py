# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Ideas On Board
#
# AWB module for tuning rkisp1

from .awb import AWB

import libtuning as lt


class AWBRkISP1(AWB):
    hr_name = 'AWB (RkISP1)'
    out_name = 'Awb'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {}

        output['colourGains'] = self.do_calculation(images)

        return output
