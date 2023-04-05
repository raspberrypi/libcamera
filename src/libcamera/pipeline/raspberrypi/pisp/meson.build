# SPDX-License-Identifier: CC0-1.0

pisp_sources = files([
    '../common/delayed_controls.cpp',
    '../common/pipeline_base.cpp',
    '../common/rpi_stream.cpp',
    'pisp.cpp',
])

includes += include_directories('../common/')

librt = cc.find_library('rt', required : true)
libpisp_dep = dependency('libpisp', fallback : ['libpisp', 'libpisp_dep'])

libcamera_sources += pisp_sources
libcamera_deps += [libpisp_dep, librt]

subdir('data')