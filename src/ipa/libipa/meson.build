# SPDX-License-Identifier: CC0-1.0

libipa_headers = files([
    'algorithm.h',
    'camera_sensor_helper.h',
    'histogram.h'
])

libipa_sources = files([
    'camera_sensor_helper.cpp',
    'histogram.cpp',
    'libipa.cpp',
])

libipa_includes = include_directories('..')

libipa = static_library('ipa', [libipa_sources, libipa_headers],
                        include_directories : ipa_includes,
                        dependencies : libcamera_private)
