#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2020, Google Inc.
#
# Author: Paul Elder <paul.elder@ideasonboard.com>
#
# Run mojo parser with python3

import os
import sys

# Make sure that mojom_parser.py can import mojom
sys.path.insert(0, f'{os.path.dirname(__file__)}/mojo/public/tools/mojom')

import mojo.public.tools.mojom.mojom_parser as parser

parser.Run(sys.argv[1:])
