# SPDX-License-Identifier: CC0-1.0

apps_sources = files([
    'image.cpp',
    'options.cpp',
    'stream_options.cpp',
])

apps_cpp_args = []

if libevent.found()
    apps_sources += files([
        'event_loop.cpp',
    ])
endif

if libtiff.found()
    apps_cpp_args += ['-DHAVE_TIFF']
    apps_sources += files([
        'dng_writer.cpp',
    ])
endif

apps_lib = static_library('apps', apps_sources,
                          cpp_args : apps_cpp_args,
                          dependencies : [libcamera_public])
