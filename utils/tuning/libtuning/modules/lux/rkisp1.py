# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Ideas on Board
#
# Lux module for tuning rkisp1

from .lux import Lux


class LuxRkISP1(Lux):
    hr_name = 'Lux (RkISP1)'
    out_name = 'Lux'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # We don't need anything from the config file.
    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        return self.calculate_lux_reference_values(images)
