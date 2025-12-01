# SPDX-License-Identifier: CC0-1.0
# Copyright (C) 2025, Raspberry Pi Ltd

pisp_sources += files([
    'backend_device.cpp',
    'media_device.cpp',
    'v4l2_device.cpp'
    ])

helper_headers = files([
    'backend_device.hpp',
    'device_fd.hpp',
    'media_device.hpp',
    'v4l2_device.hpp',
])

helper_include_dir = pisp_include_dir / 'helpers'
install_headers(helper_headers, subdir: helper_include_dir)
