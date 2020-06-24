# SPDX-License-Identifier: CC0-1.0

android_hal_sources = files([
    'camera3_hal.cpp',
    'camera_hal_manager.cpp',
    'camera_device.cpp',
    'camera_metadata.cpp',
    'camera_ops.cpp',
    'jpeg/encoder_libjpeg.cpp',
])

android_camera_metadata_sources = files([
    'metadata/camera_metadata.c',
])

android_deps = [
    dependency('libjpeg'),
]

android_camera_metadata = static_library('camera_metadata',
                                         android_camera_metadata_sources,
                                         include_directories : android_includes)
