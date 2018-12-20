libcamera_sources = files([
    'camera.cpp',
    'camera_manager.cpp',
    'device_enumerator.cpp',
    'log.cpp',
    'main.cpp',
    'pipeline_handler.cpp',
])

libcamera_headers = files([
    'include/device_enumerator.h',
    'include/log.h',
    'include/pipeline_handler.h',
    'include/utils.h',
])

libcamera_internal_includes =  include_directories('include')

includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

subdir('pipeline')

libudev = dependency('libudev')

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           include_directories : includes,
                           dependencies : libudev)
