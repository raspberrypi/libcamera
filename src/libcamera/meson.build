sources = files([
    'log.cpp',
    'main.cpp',
])

includes = [
    libcamera_includes,
    include_directories('include'),
]

libcamera = shared_library('camera',
                           sources,
                           install : true,
                           include_directories : includes)
