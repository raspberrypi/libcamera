# SPDX-License-Identifier: CC0-1.0

if get_option('android_platform') != 'cros'
   subdir_done()
endif

android_hal_sources += files([
    'camera3_hal.cpp',
])

android_deps += dependency('libcros_camera')

android_cpp_args += ['-DOS_CHROMEOS']
