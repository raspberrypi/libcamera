# SPDX-License-Identifier: CC0-1.0
# Copyright (C) 2023, Raspberry Pi Ltd

backend_sources += files([
    'context_stage.cpp',
    'crop_stage.cpp',
    'input_stage.cpp',
    'output_stage.cpp',
    'pipeline.cpp',
    'pisp_tiling.cpp',
    'rescale_stage.cpp',
    'split_stage.cpp',
    'stages.cpp',
])

tiling_headers = files([
    'pisp_tiling.hpp',
    'types.hpp',
])

tiling_include_dir = backend_include_dir / 'tiling'
install_headers(tiling_headers, subdir: tiling_include_dir)
