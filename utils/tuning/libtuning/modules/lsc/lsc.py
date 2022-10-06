# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>

from ..module import Module

import libtuning as lt
import libtuning.utils as utils

import numpy as np


class LSC(Module):
    type = 'lsc'
    hr_name = 'LSC (Base)'
    out_name = 'GenericLSC'

    def __init__(self, *,
                 debug: list,
                 sector_shape: tuple,
                 sector_x_gradient: lt.Gradient,
                 sector_y_gradient: lt.Gradient,
                 sector_average_function: lt.Average,
                 smoothing_function: lt.Smoothing):
        super().__init__()

        self.debug = debug

        self.sector_shape = sector_shape
        self.sector_x_gradient = sector_x_gradient
        self.sector_y_gradient = sector_y_gradient
        self.sector_average_function = sector_average_function

        self.smoothing_function = smoothing_function

    def _enumerate_lsc_images(self, images):
        for image in images:
            if image.lsc_only:
                yield image

    def _get_grid(self, channel, img_w, img_h):
        # List of number of pixels in each sector
        sectors_x = self.sector_x_gradient.distribute(img_w / 2, self.sector_shape[0])
        sectors_y = self.sector_y_gradient.distribute(img_h / 2, self.sector_shape[1])

        grid = []

        r = 0
        for y in sectors_y:
            c = 0
            for x in sectors_x:
                grid.append(self.sector_average_function.average(channel[r:r + y, c:c + x]))
                c += x
            r += y

        return np.array(grid)

    def _lsc_single_channel(self, channel: np.array,
                            image: lt.Image, green_grid: np.array = None):
        grid = self._get_grid(channel, image.w, image.h)
        grid -= image.blacklevel_16
        if green_grid is None:
            table = np.reshape(1 / grid, self.sector_shape[::-1])
        else:
            table = np.reshape(green_grid / grid, self.sector_shape[::-1])
        table = self.smoothing_function.smoothing(table)

        if green_grid is None:
            table = table / np.min(table)

        return table, grid
