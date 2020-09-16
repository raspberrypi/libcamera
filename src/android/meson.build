# SPDX-License-Identifier: CC0-1.0

android_deps = [
    dependency('libexif', required : get_option('android')),
    dependency('libjpeg', required : get_option('android')),
]

android_enabled = true

foreach dep : android_deps
    if not dep.found()
        android_enabled = false
        subdir_done()
    endif
endforeach

android_hal_sources = files([
    'camera3_hal.cpp',
    'camera_hal_manager.cpp',
    'camera_device.cpp',
    'camera_metadata.cpp',
    'camera_ops.cpp',
    'jpeg/encoder_libjpeg.cpp',
    'jpeg/exif.cpp',
])

android_camera_metadata_sources = files([
    'metadata/camera_metadata.c',
])

android_camera_metadata = static_library('camera_metadata',
                                         android_camera_metadata_sources,
                                         include_directories : android_includes)
