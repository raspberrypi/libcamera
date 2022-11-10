# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# gradient.py - Gradients that can be used to distribute or map numbers

import libtuning as lt

import math
from numbers import Number


# @brief Gradient for how to allocate pixels to sectors
# @description There are no parameters for the gradients as the domain is the
#              number of pixels and the range is the number of sectors, and
#              there is only one curve that has a startpoint and endpoint at
#              (0, 0) and at (#pixels, #sectors). The exception is for curves
#              that *do* have multiple solutions for only two points, such as
#              gaussian, and curves of higher polynomial orders if we had them.
#
# \todo There will probably be a helper in the Gradient class, as I have a
# feeling that all the other curves (besides Linear and Gaussian) can be
# implemented in the same way.
class Gradient(object):
    def __init__(self):
        pass

    # @brief Distribute pixels into sectors (only in one dimension)
    # @param domain Number of pixels
    # @param sectors Number of sectors
    # @return A list of number of pixels in each sector
    def distribute(self, domain: list, sectors: list) -> list:
        raise NotImplementedError

    # @brief Map a number on a curve
    # @param domain Domain of the curve
    # @param rang Range of the curve
    # @param x Input on the domain of the curve
    # @return y from the range of the curve
    def map(self, domain: tuple, rang: tuple, x: Number) -> Number:
        raise NotImplementedError


class Linear(Gradient):
    # @param remainder Mode of handling remainder
    def __init__(self, remainder: lt.Remainder = lt.Remainder.Float):
        self.remainder = remainder

    def distribute(self, domain: list, sectors: list) -> list:
        size = domain / sectors
        rem = domain % sectors

        if rem == 0:
            return [int(size)] * sectors

        size = math.ceil(size)
        rem = domain % size
        output_sectors = [int(size)] * (sectors - 1)

        if self.remainder == lt.Remainder.Float:
            size = domain / sectors
            output_sectors = [size] * sectors
        elif self.remainder == lt.Remainder.DistributeFront:
            output_sectors.append(int(rem))
        elif self.remainder == lt.Remainder.DistributeBack:
            output_sectors.insert(0, int(rem))
        else:
            raise ValueError

        return output_sectors

    def map(self, domain: tuple, rang: tuple, x: Number) -> Number:
        m = (rang[1] - rang[0]) / (domain[1] - domain[0])
        b = rang[0] - m * domain[0]
        return m * x + b
