#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# parser.py - Run mojo parser with python3

import os
import sys

# TODO set sys.pycache_prefix for >= python3.8
sys.dont_write_bytecode = True

# Make sure that mojom_parser.py can import mojom
sys.path.append(f'{os.path.dirname(__file__)}/mojo/public/tools/mojom')

import mojo.public.tools.mojom.mojom_parser as parser

parser.Run(sys.argv[1:])
