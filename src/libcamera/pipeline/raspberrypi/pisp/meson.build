# SPDX-License-Identifier: CC0-1.0

pisp_sources = files([
    'pisp.cpp',
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

librt = cc.find_library('rt', required : true)
libpisp_dep = dependency('libpisp', fallback : ['libpisp', 'libpisp_dep'])

libcamera_sources += pisp_sources
libcamera_deps += [libpisp_dep, librt]

subdir('data')