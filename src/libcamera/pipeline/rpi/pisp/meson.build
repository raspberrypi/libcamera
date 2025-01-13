# SPDX-License-Identifier: CC0-1.0

libcamera_internal_sources += files([
    'pisp.cpp',
])

librt = cc.find_library('rt', required : true)
libpisp_dep = dependency('libpisp', fallback : ['libpisp', 'libpisp_dep'])

libcamera_deps += [libpisp_dep, librt]

subdir('data')
