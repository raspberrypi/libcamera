# SPDX-License-Identifier: CC0-1.0
# Copyright (C) 2023, Raspberry Pi Ltd

pisp_sources += files([
    'pisp_utils.cpp',
    'pisp_logging.cpp',
    'pisp_pwl.cpp',
])

common_headers = files([
    'pisp_common.h',
    'logging.hpp',
    'shm_mutex.hpp',
    'utils.hpp',
    'version.hpp',
])

common_include_dir = pisp_include_dir / 'common'
install_headers(common_headers, subdir: common_include_dir)
