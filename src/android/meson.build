android_hal_sources = files([
    'camera3_hal.cpp',
    'camera_hal_manager.cpp',
    'camera_device.cpp',
    'camera_proxy.cpp',
    'thread_rpc.cpp'
])

android_camera_metadata_sources = files([
    'metadata/camera_metadata.c',
])

android_camera_metadata = static_library('camera_metadata',
                                         android_camera_metadata_sources,
                                         include_directories : android_includes)
