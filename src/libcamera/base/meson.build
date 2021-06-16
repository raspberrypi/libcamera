# SPDX-License-Identifier: CC0-1.0

libcamera_base_sources = files([
    'class.cpp',
    'bound_method.cpp',
    'event_dispatcher.cpp',
    'event_dispatcher_poll.cpp',
    'file.cpp',
    'log.cpp',
    'message.cpp',
    'object.cpp',
    'semaphore.cpp',
    'signal.cpp',
    'thread.cpp',
    'timer.cpp',
    'utils.cpp',
])

libcamera_base_deps = [
    dependency('threads'),
]

libcamera_base_lib = shared_library('libcamera-base',
                                    [libcamera_base_sources, libcamera_base_headers],
                                    name_prefix : '',
                                    install : true,
                                    include_directories : libcamera_includes,
                                    dependencies : libcamera_base_deps)

libcamera_base = declare_dependency(sources : [
                                        libcamera_base_headers,
                                    ],
                                    include_directories : libcamera_includes,
                                    link_with : libcamera_base_lib)

pkg_mod = import('pkgconfig')
pkg_mod.generate(libcamera_base_lib,
                 version : '1.0',
                 description : 'Camera support base utility library',
                 subdirs : 'libcamera')
