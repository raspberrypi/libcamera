# SPDX-License-Identifier: CC0-1.0

libcamera_sources += files([
    'dma_heaps.cpp',
    'vc4.cpp',
])

if not raspberrypi_common
    libcamera_sources += files([
        '../common/delayed_controls.cpp',
        '../common/pipeline_base.cpp',
        '../common/rpi_stream.cpp',
    ])

    includes += include_directories('../common/')
    raspberrypi_common = true
endif

subdir('data')
