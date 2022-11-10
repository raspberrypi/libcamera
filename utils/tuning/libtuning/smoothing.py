# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# smoothing.py - Wrapper for cv2 smoothing functions to enable duck-typing

import cv2


# @brief Wrapper for cv2 smoothing functions so that they can be duck-typed
class Smoothing(object):
    def __init__(self):
        pass

    def smoothing(self, src):
        raise NotImplementedError


class MedianBlur(Smoothing):
    def __init__(self, ksize):
        self.ksize = ksize

    def smoothing(self, src):
        return cv2.medianBlur(src.astype('float32'), self.ksize).astype('float64')
