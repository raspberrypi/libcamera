# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# image.py - Container for an image and associated metadata

import binascii
import numpy as np
from pathlib import Path
import pyexiv2 as pyexif
import rawpy as raw
import re

import libtuning as lt
import libtuning.utils as utils


class Image:
    def __init__(self, path: Path):
        self.path = path
        self.lsc_only = False
        self.color = -1
        self.lux = -1

        try:
            self._load_metadata_exif()
        except Exception as e:
            utils.eprint(f'Failed to load metadata from {self.path}: {e}')
            raise e

        try:
            self._read_image_dng()
        except Exception as e:
            utils.eprint(f'Failed to load image data from {self.path}: {e}')
            raise e

    @property
    def name(self):
        return self.path.name

    # May raise KeyError as there are too many to check
    def _load_metadata_exif(self):
        # RawPy doesn't load all the image tags that we need, so we use py3exiv2
        metadata = pyexif.ImageMetadata(str(self.path))
        metadata.read()

        # The DNG and TIFF/EP specifications use different IFDs to store the
        # raw image data and the Exif tags. DNG stores them in a SubIFD and in
        # an Exif IFD respectively (named "SubImage1" and "Photo" by pyexiv2),
        # while TIFF/EP stores them both in IFD0 (name "Image"). Both are used
        # in "DNG" files, with libcamera-apps following the DNG recommendation
        # and applications based on picamera2 following TIFF/EP.
        #
        # This code detects which tags are being used, and therefore extracts the
        # correct values.
        try:
            self.w = metadata['Exif.SubImage1.ImageWidth'].value
            subimage = 'SubImage1'
            photo = 'Photo'
        except KeyError:
            self.w = metadata['Exif.Image.ImageWidth'].value
            subimage = 'Image'
            photo = 'Image'
        self.pad = 0
        self.h = metadata[f'Exif.{subimage}.ImageLength'].value
        white = metadata[f'Exif.{subimage}.WhiteLevel'].value
        self.sigbits = int(white).bit_length()
        self.fmt = (self.sigbits - 4) // 2
        self.exposure = int(metadata[f'Exif.{photo}.ExposureTime'].value * 1000000)
        self.againQ8 = metadata[f'Exif.{photo}.ISOSpeedRatings'].value * 256 / 100
        self.againQ8_norm = self.againQ8 / 256
        self.camName = metadata['Exif.Image.Model'].value
        self.blacklevel = int(metadata[f'Exif.{subimage}.BlackLevel'].value[0])
        self.blacklevel_16 = self.blacklevel << (16 - self.sigbits)

        # Channel order depending on bayer pattern
        # The key is the order given by exif, where 0 is R, 1 is G, and 2 is B
        # The value is the index where the color can be found, where the first
        # is R, then G, then G, then B.
        bayer_case = {
            '0 1 1 2': (lt.Color.R, lt.Color.GR, lt.Color.GB, lt.Color.B),
            '1 2 0 1': (lt.Color.GB, lt.Color.R, lt.Color.B, lt.Color.GR),
            '2 1 1 0': (lt.Color.B, lt.Color.GB, lt.Color.GR, lt.Color.R),
            '1 0 2 1': (lt.Color.GR, lt.Color.R, lt.Color.B, lt.Color.GB)
        }
        # Note: This needs to be in IFD0
        cfa_pattern = metadata[f'Exif.{subimage}.CFAPattern'].value
        self.order = bayer_case[cfa_pattern]

    def _read_image_dng(self):
        raw_im = raw.imread(str(self.path))
        raw_data = raw_im.raw_image
        shift = 16 - self.sigbits
        c0 = np.left_shift(raw_data[0::2, 0::2].astype(np.int64), shift)
        c1 = np.left_shift(raw_data[0::2, 1::2].astype(np.int64), shift)
        c2 = np.left_shift(raw_data[1::2, 0::2].astype(np.int64), shift)
        c3 = np.left_shift(raw_data[1::2, 1::2].astype(np.int64), shift)
        self.channels = [c0, c1, c2, c3]
        # Reorder the channels into R, GR, GB, B
        self.channels = [self.channels[i] for i in self.order]

    # \todo Move this to macbeth.py
    def get_patches(self, cen_coords, size=16):
        saturated = False

        # Obtain channel widths and heights
        ch_w, ch_h = self.w, self.h
        cen_coords = list(np.array((cen_coords[0])).astype(np.int32))
        self.cen_coords = cen_coords

        # Squares are ordered by stacking macbeth chart columns from left to
        # right. Some useful patch indices:
        #     white = 3
        #     black = 23
        #     'reds' = 9, 10
        #     'blues' = 2, 5, 8, 20, 22
        #     'greens' = 6, 12, 17
        #     greyscale = 3, 7, 11, 15, 19, 23
        all_patches = []
        for ch in self.channels:
            ch_patches = []
            for cen in cen_coords:
                # Macbeth centre is placed at top left of central 2x2 patch to
                # account for rounding. Patch pixels are sorted by pixel
                # brightness so spatial information is lost.
                patch = ch[cen[1] - 7:cen[1] + 9, cen[0] - 7:cen[0] + 9].flatten()
                patch.sort()
                if patch[-5] == (2**self.sigbits - 1) * 2**(16 - self.sigbits):
                    saturated = True
                ch_patches.append(patch)

            all_patches.append(ch_patches)

        self.patches = all_patches

        return not saturated
