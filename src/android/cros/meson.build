# SPDX-License-Identifier: CC0-1.0

if get_option('android_platform') != 'cros'
   subdir_done()
endif

cros_hal_info_sources = files([
    'camera3_hal.cpp',
])

cros_hal_info = static_library('cros_hal_info',
                               cros_hal_info_sources,
                               dependencies : dependency('libcros_camera'),
                               c_args : '-Wno-shadow',
                               include_directories : [android_includes,
                                                      libcamera_includes])

libcamera_objects += cros_hal_info.extract_objects('camera3_hal.cpp')
