# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>

from ..module import Module

import libtuning as lt


class AGC(Module):
    type = 'agc'
    hr_name = 'AGC (Base)'
    out_name = 'GenericAGC'

    # \todo Add sector shapes and stuff just like lsc
    def __init__(self, *,
                 debug: list):
        super().__init__()

        self.debug = debug
