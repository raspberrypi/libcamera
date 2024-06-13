# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Ideas on Board
#
# Module implementation for static data

from .module import Module


# This module can be used in cases where the tuning file should contain
# static data.
class StaticModule(Module):
    def __init__(self, out_name: str, output: dict = {}):
        super().__init__()
        self.out_name = out_name
        self.hr_name = f'Static {out_name}'
        self.type = f'static_{out_name}'
        self.output = output

    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        return self.output
