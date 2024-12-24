# SPDX-License-Identifier: CC0-1.0

android_deps = [
    dependency('libexif', required : get_option('android')),
    dependency('libjpeg', required : get_option('android')),
    libcamera_private,
    libyuv_dep,
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
    'camera_capabilities.cpp',
    'camera_device.cpp',
    'camera_hal_config.cpp',
    'camera_hal_manager.cpp',
    'camera_metadata.cpp',
    'camera_ops.cpp',
    'camera_request.cpp',
    'camera_stream.cpp',
    'hal_framebuffer.cpp',
    'yuv/post_processor_yuv.cpp'
])

android_cpp_args = []

subdir('cros')
subdir('jpeg')
subdir('mm')

android_camera_metadata_sources = files([
    'metadata/camera_metadata.c',
])

android_camera_metadata = static_library('camera_metadata',
                                         android_camera_metadata_sources,
                                         c_args : '-Wno-shadow',
                                         include_directories : android_includes)

libcamera_hal = shared_library('libcamera-hal',
                               android_hal_sources,
                               name_prefix : '',
                               link_with : android_camera_metadata,
                               install : true,
                               cpp_args : android_cpp_args,
                               include_directories : android_includes,
                               dependencies : android_deps)
