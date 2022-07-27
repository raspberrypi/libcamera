# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_lux.py - camera tuning tool for lux level

from ctt_tools import *


"""
Find lux values from metadata and calculate Y
"""
def lux(Cam, Img):
    shutter_speed = Img.exposure
    gain = Img.againQ8_norm
    aperture = 1
    Cam.log += '\nShutter speed = {}'.format(shutter_speed)
    Cam.log += '\nGain = {}'.format(gain)
    Cam.log += '\nAperture = {}'.format(aperture)
    patches = [Img.patches[i] for i in Img.order]
    channels = [Img.channels[i] for i in Img.order]
    return lux_calc(Cam, Img, patches, channels), shutter_speed, gain


"""
perform lux calibration on bayer channels
"""
def lux_calc(Cam, Img, patches, channels):
    """
    find means color channels on grey patches
    """
    ap_r = np.mean(patches[0][3::4])
    ap_g = (np.mean(patches[1][3::4])+np.mean(patches[2][3::4]))/2
    ap_b = np.mean(patches[3][3::4])
    Cam.log += '\nAverage channel values on grey patches:'
    Cam.log += '\nRed = {:.0f} Green = {:.0f} Blue = {:.0f}'.format(ap_r, ap_b, ap_g)
    # print(ap_r, ap_g, ap_b)
    """
    calculate channel gains
    """
    gr = ap_g/ap_r
    gb = ap_g/ap_b
    Cam.log += '\nChannel gains: Red = {:.3f} Blue = {:.3f}'.format(gr, gb)

    """
    find means color channels on image and scale by gain
    note greens are averaged together (treated as one channel)
    """
    a_r = np.mean(channels[0])*gr
    a_g = (np.mean(channels[1])+np.mean(channels[2]))/2
    a_b = np.mean(channels[3])*gb
    Cam.log += '\nAverage channel values over entire image scaled by channel gains:'
    Cam.log += '\nRed = {:.0f} Green = {:.0f} Blue = {:.0f}'.format(a_r, a_b, a_g)
    # print(a_r, a_g, a_b)
    """
    Calculate y with top row of yuv matrix
    """
    y = 0.299*a_r + 0.587*a_g + 0.114*a_b
    Cam.log += '\nY value calculated: {}'.format(int(y))
    # print(y)
    return int(y)
