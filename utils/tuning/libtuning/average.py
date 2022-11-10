# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# average.py - Wrapper for numpy averaging functions to enable duck-typing

import numpy as np


# @brief Wrapper for np averaging functions so that they can be duck-typed
class Average(object):
    def __init__(self):
        pass

    def average(self, np_array):
        raise NotImplementedError


class Mean(Average):
    def average(self, np_array):
        return np.mean(np_array)
