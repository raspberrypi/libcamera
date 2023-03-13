# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
#
# Debayering code from PiCamera documentation

from numpy.lib.stride_tricks import as_strided
import libcamera as libcam
import libcamera.utils
import numpy as np


def demosaic(data, r0, g0, g1, b0):
    # Separate the components from the Bayer data to RGB planes

    rgb = np.zeros(data.shape + (3,), dtype=data.dtype)
    rgb[r0[1]::2, r0[0]::2, 0] = data[r0[1]::2, r0[0]::2]  # Red
    rgb[g0[1]::2, g0[0]::2, 1] = data[g0[1]::2, g0[0]::2]  # Green
    rgb[g1[1]::2, g1[0]::2, 1] = data[g1[1]::2, g1[0]::2]  # Green
    rgb[b0[1]::2, b0[0]::2, 2] = data[b0[1]::2, b0[0]::2]  # Blue

    # Below we present a fairly naive de-mosaic method that simply
    # calculates the weighted average of a pixel based on the pixels
    # surrounding it. The weighting is provided by a byte representation of
    # the Bayer filter which we construct first:

    bayer = np.zeros(rgb.shape, dtype=np.uint8)
    bayer[r0[1]::2, r0[0]::2, 0] = 1  # Red
    bayer[g0[1]::2, g0[0]::2, 1] = 1  # Green
    bayer[g1[1]::2, g1[0]::2, 1] = 1  # Green
    bayer[b0[1]::2, b0[0]::2, 2] = 1  # Blue

    # Allocate an array to hold our output with the same shape as the input
    # data. After this we define the size of window that will be used to
    # calculate each weighted average (3x3). Then we pad out the rgb and
    # bayer arrays, adding blank pixels at their edges to compensate for the
    # size of the window when calculating averages for edge pixels.

    output = np.empty(rgb.shape, dtype=rgb.dtype)
    window = (3, 3)
    borders = (window[0] - 1, window[1] - 1)
    border = (borders[0] // 2, borders[1] // 2)

    rgb = np.pad(rgb, [
        (border[0], border[0]),
        (border[1], border[1]),
        (0, 0),
    ], 'constant')
    bayer = np.pad(bayer, [
        (border[0], border[0]),
        (border[1], border[1]),
        (0, 0),
    ], 'constant')

    # For each plane in the RGB data, we use a nifty numpy trick
    # (as_strided) to construct a view over the plane of 3x3 matrices. We do
    # the same for the bayer array, then use Einstein summation on each
    # (np.sum is simpler, but copies the data so it's slower), and divide
    # the results to get our weighted average:

    for plane in range(3):
        p = rgb[..., plane]
        b = bayer[..., plane]
        pview = as_strided(p, shape=(
            p.shape[0] - borders[0],
            p.shape[1] - borders[1]) + window, strides=p.strides * 2)
        bview = as_strided(b, shape=(
            b.shape[0] - borders[0],
            b.shape[1] - borders[1]) + window, strides=b.strides * 2)
        psum = np.einsum('ijkl->ij', pview)
        bsum = np.einsum('ijkl->ij', bview)
        output[..., plane] = psum // bsum

    return output


def to_rgb(fmt, size, data):
    w = size.width
    h = size.height

    if fmt == libcam.formats.YUYV:
        # YUV422
        yuyv = data.reshape((h, w // 2 * 4))

        # YUV444
        yuv = np.empty((h, w, 3), dtype=np.uint8)
        yuv[:, :, 0] = yuyv[:, 0::2]                    # Y
        yuv[:, :, 1] = yuyv[:, 1::4].repeat(2, axis=1)  # U
        yuv[:, :, 2] = yuyv[:, 3::4].repeat(2, axis=1)  # V

        m = np.array([
            [1.0, 1.0, 1.0],
            [-0.000007154783816076815, -0.3441331386566162, 1.7720025777816772],
            [1.4019975662231445, -0.7141380310058594, 0.00001542569043522235]
        ])

        rgb = np.dot(yuv, m)
        rgb[:, :, 0] -= 179.45477266423404
        rgb[:, :, 1] += 135.45870971679688
        rgb[:, :, 2] -= 226.8183044444304
        rgb = rgb.astype(np.uint8)

    elif fmt == libcam.formats.RGB888:
        rgb = data.reshape((h, w, 3))
        rgb[:, :, [0, 1, 2]] = rgb[:, :, [2, 1, 0]]

    elif fmt == libcam.formats.BGR888:
        rgb = data.reshape((h, w, 3))

    elif fmt in [libcam.formats.ARGB8888, libcam.formats.XRGB8888]:
        rgb = data.reshape((h, w, 4))
        rgb = np.flip(rgb, axis=2)
        # drop alpha component
        rgb = np.delete(rgb, np.s_[0::4], axis=2)

    elif str(fmt).startswith('S'):
        fmt = str(fmt)
        bayer_pattern = fmt[1:5]
        bitspp = int(fmt[5:])

        if bitspp == 8:
            data = data.reshape((h, w))
            data = data.astype(np.uint16)
        elif bitspp in [10, 12]:
            data = data.view(np.uint16)
            data = data.reshape((h, w))
        else:
            raise Exception('Bad bitspp:' + str(bitspp))

        idx = bayer_pattern.find('R')
        assert(idx != -1)
        r0 = (idx % 2, idx // 2)

        idx = bayer_pattern.find('G')
        assert(idx != -1)
        g0 = (idx % 2, idx // 2)

        idx = bayer_pattern.find('G', idx + 1)
        assert(idx != -1)
        g1 = (idx % 2, idx // 2)

        idx = bayer_pattern.find('B')
        assert(idx != -1)
        b0 = (idx % 2, idx // 2)

        rgb = demosaic(data, r0, g0, g1, b0)
        rgb = (rgb >> (bitspp - 8)).astype(np.uint8)

    else:
        rgb = None

    return rgb


# A naive format conversion to 24-bit RGB
def mfb_to_rgb(mfb: libcamera.utils.MappedFrameBuffer, cfg: libcam.StreamConfiguration):
    data = np.array(mfb.planes[0], dtype=np.uint8)
    rgb = to_rgb(cfg.pixel_format, cfg.size, data)
    return rgb
