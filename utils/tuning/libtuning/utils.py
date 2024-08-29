# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# Utilities for libtuning

import cv2
import decimal
import math
import numpy as np
import os
from pathlib import Path
import re
import sys
import logging

import libtuning as lt
from libtuning.image import Image
from .macbeth import locate_macbeth

logger = logging.getLogger(__name__)

# Utility functions


def get_module_by_type_name(modules, name):
    for module in modules:
        if module.type == name:
            return module
    return None


# Private utility functions


def _list_image_files(directory):
    d = Path(directory)
    files = [d.joinpath(f) for f in os.listdir(d)
             if re.search(r'\.(jp[e]g$)|(dng$)', f)]
    files.sort()
    return files


def _parse_image_filename(fn: Path):
    lsc_only = False
    color_temperature = None
    lux = None

    parts = fn.stem.split('_')
    for part in parts:
        if part == 'alsc':
            lsc_only = True
            continue
        r = re.match(r'(\d+)[kK]', part)
        if r:
            color_temperature = int(r.group(1))
            continue
        r = re.match(r'(\d+)[lLuU]', part)
        if r:
            lux = int(r.group(1))

    if color_temperature is None:
        logger.error(f'The file name of "{fn.name}" does not contain a color temperature')

    if lux is None and lsc_only is False:
        logger.error(f'The file name of "{fn.name}" must either contain alsc or a lux level')

    return color_temperature, lux, lsc_only


# \todo Implement this from check_imgs() in ctt.py
def _validate_images(images):
    return True


# Public utility functions


# @brief Load images into a single list of Image instances
# @param input_dir Directory from which to load image files
# @param config Configuration dictionary
# @param load_nonlsc Whether or not to load non-lsc images
# @param load_lsc Whether or not to load lsc-only images
# @return A list of Image instances
def load_images(input_dir: str, config: dict, load_nonlsc: bool, load_lsc: bool) -> list:
    files = _list_image_files(input_dir)
    if len(files) == 0:
        logger.error(f'No images found in {input_dir}')
        return None

    images = []
    for f in files:
        color, lux, lsc_only = _parse_image_filename(f)

        if color is None:
            logger.warning(f'Ignoring "{f.name}" as it has no associated color temperature')
            continue

        logger.info(f'Process image "{f.name}" (color={color}, lux={lux}, lsc_only={lsc_only})')

        # Skip lsc image if we don't need it
        if lsc_only and not load_lsc:
            logger.warning(f'Skipping {f.name} as this tuner has no LSC module')
            continue

        # Skip non-lsc image if we don't need it
        if not lsc_only and not load_nonlsc:
            logger.warning(f'Skipping {f.name} as this tuner only has an LSC module')
            continue

        # Load image
        try:
            image = Image(f)
        except Exception as e:
            logger.error(f'Failed to load image {f.name}: {e}')
            continue

        # Populate simple fields
        image.lsc_only = lsc_only
        image.color = color
        image.lux = lux

        # Black level comes from the TIFF tags, but they are overridable by the
        # config file.
        if 'blacklevel' in config['general']:
            image.blacklevel_16 = config['general']['blacklevel']

        if lsc_only:
            images.append(image)
            continue

        # Handle macbeth
        macbeth = locate_macbeth(image, config)
        if macbeth is None:
            continue

        images.append(image)

    if not _validate_images(images):
        return None

    return images



"""
Some code that will save virtual macbeth charts that show the difference between optimised matrices and non optimised matrices

The function creates an image that is 1550 by 1050 pixels wide, and fills it with patches which are 200x200 pixels in size
Each patch contains the ideal color, the color from the original matrix, and the color from the final matrix
_________________
|               |
|  Ideal Color  |
|_______________|
|  Old  |  new  |
| Color | Color |
|_______|_______|

Nice way of showing how the optimisation helps change the colors and the color matricies
"""
def visualise_macbeth_chart(macbeth_rgb, original_rgb, new_rgb, output_filename):
    image = np.zeros((1050, 1550, 3), dtype=np.uint8)
    colorindex = -1
    for y in range(6):
        for x in range(4):  # Creates 6 x 4 grid of macbeth chart
            colorindex += 1
            xlocation = 50 + 250 * x  # Means there is 50px of black gap between each square, more like the real macbeth chart.
            ylocation = 50 + 250 * y
            for g in range(200):
                for i in range(100):
                    image[xlocation + i, ylocation + g] = macbeth_rgb[colorindex]
            xlocation = 150 + 250 * x
            ylocation = 50 + 250 * y
            for i in range(100):
                for g in range(100):
                    image[xlocation + i, ylocation + g] = original_rgb[colorindex]  # Smaller squares below to compare the old colors with the new ones
            xlocation = 150 + 250 * x
            ylocation = 150 + 250 * y
            for i in range(100):
                for g in range(100):
                    image[xlocation + i, ylocation + g] = new_rgb[colorindex]

    im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(f'{output_filename} Generated Macbeth Chart.png', im_bgr)

