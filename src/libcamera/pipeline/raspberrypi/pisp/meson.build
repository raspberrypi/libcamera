# SPDX-License-Identifier: CC0-1.0

pisp_sources = files([
    'pisp.cpp',
    'rpi_stream.cpp',
])

subdir('common')
subdir('variants')
subdir('frontend')
subdir('backend')

pisp_sources += frontend_sources
pisp_sources += backend_sources

libcamera_sources += pisp_sources
