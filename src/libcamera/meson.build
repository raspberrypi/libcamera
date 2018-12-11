libcamera_sources = files([
    'log.cpp',
    'main.cpp',
])

libcamera_headers = files([
    'include/log.h',
    'include/utils.h',
])

includes = [
    libcamera_includes,
    include_directories('include'),
]

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           include_directories : includes)
