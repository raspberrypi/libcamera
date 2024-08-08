#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# Run mojo code generator for generating libcamera IPC files

import os
import sys

sys.path.insert(0, f'{os.path.dirname(__file__)}/mojo/public/tools/bindings')

import mojo.public.tools.bindings.mojom_bindings_generator as generator

def _GetModulePath(path, output_dir):
    return os.path.join(output_dir, path.relative_path())


# Disable the attribute checker to support our custom attributes. Ideally we
# should add the attributes to the list of allowed attributes in
# utils/ipc/mojo/public/tools/bindings/checks/mojom_attributes_check.py, but
# we're trying hard to use the upstream mojom as-is.
if hasattr(generator, '_BUILTIN_CHECKS'):
    del generator._BUILTIN_CHECKS['attributes']

# Override the mojo code generator's generator list to only contain our
# libcamera generator
generator._BUILTIN_GENERATORS = {'libcamera': 'mojom_libcamera_generator'}

# Override the mojo code generator's _GetModulePath method to not add
# the '-module' suffix when searching for mojo modules, so that we can
# pass the path to the mojom module without having to trim the '-module' suffix
generator._GetModulePath = _GetModulePath

generator.main()
