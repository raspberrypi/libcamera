libcamera_sources = files([
    'camera.cpp',
    'log.cpp',
    'main.cpp',
])

libcamera_headers = files([
    'include/log.h',
    'include/utils.h',
])

libcamera_internal_includes =  include_directories('include')

includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

libudev = dependency('libudev')

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           include_directories : includes,
                           dependencies : libudev)
