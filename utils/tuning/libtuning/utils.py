# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# utils.py - Utilities for libtuning

import decimal
import math
import numpy as np
import os
from pathlib import Path
import re
import sys

import libtuning as lt
from libtuning.image import Image
from libtuning.macbeth import locate_macbeth

# Utility functions


def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


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
    result = re.search(r'^(alsc_)?(\d+)[kK]_(\d+)?[lLuU]?.\w{3,4}$', fn.name)
    if result is None:
        eprint(f'The file name of {fn.name} is incorrectly formatted')
        return None, None, None

    color = int(result.group(2))
    lsc_only = result.group(1) is not None
    lux = None if lsc_only else int(result.group(3))

    return color, lux, lsc_only


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
        eprint(f'No images found in {input_dir}')
        return None

    images = []
    for f in files:
        color, lux, lsc_only = _parse_image_filename(f)
        if color is None:
            continue

        # Skip lsc image if we don't need it
        if lsc_only and not load_lsc:
            eprint(f'Skipping {f.name} as this tuner has no LSC module')
            continue

        # Skip non-lsc image if we don't need it
        if not lsc_only and not load_nonlsc:
            eprint(f'Skipping {f.name} as this tuner only has an LSC module')
            continue

        # Load image
        try:
            image = Image(f)
        except Exception as e:
            eprint(f'Failed to load image {f.name}: {e}')
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
        macbeth = locate_macbeth(config)
        if macbeth is None:
            continue

        images.append(image)

    if not _validate_images(images):
        return None

    return images
