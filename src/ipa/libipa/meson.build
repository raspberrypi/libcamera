# SPDX-License-Identifier: CC0-1.0

libipa_headers = files([
    'algorithm.h',
    'camera_sensor_helper.h',
    'histogram.h',
    'module.h',
])

libipa_sources = files([
    'algorithm.cpp',
    'camera_sensor_helper.cpp',
    'histogram.cpp',
    'module.cpp',
])

libipa_includes = include_directories('..')

libipa = static_library('ipa', [libipa_sources, libipa_headers],
                        include_directories : ipa_includes,
                        dependencies : libcamera_private)
