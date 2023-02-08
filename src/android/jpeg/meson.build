# SPDX-License-Identifier: CC0-1.0

android_hal_sources += files([
    'encoder_libjpeg.cpp',
    'exif.cpp',
    'post_processor_jpeg.cpp',
    'thumbnailer.cpp'
])

platform = get_option('android_platform')
if platform == 'cros'
    android_hal_sources += files(['encoder_jea.cpp'])
    android_deps += [dependency('libcros_camera')]
endif
