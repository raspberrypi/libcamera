# SPDX-License-Identifier: CC0-1.0

libcamera_base_sources = files([
    'backtrace.cpp',
    'class.cpp',
    'bound_method.cpp',
    'event_dispatcher.cpp',
    'event_dispatcher_poll.cpp',
    'event_notifier.cpp',
    'file.cpp',
    'flags.cpp',
    'log.cpp',
    'message.cpp',
    'mutex.cpp',
    'object.cpp',
    'semaphore.cpp',
    'shared_fd.cpp',
    'signal.cpp',
    'thread.cpp',
    'timer.cpp',
    'unique_fd.cpp',
    'utils.cpp',
])

libdw = dependency('libdw', required : false)
libunwind = dependency('libunwind', required : false)

if cc.has_header_symbol('execinfo.h', 'backtrace')
    config_h.set('HAVE_BACKTRACE', 1)
endif

if libdw.found()
    config_h.set('HAVE_DW', 1)
endif

if libunwind.found()
    config_h.set('HAVE_UNWIND', 1)
endif

libcamera_base_deps = [
    libatomic,
    libdw,
    libthreads,
    libunwind,
]

# Internal components must use the libcamera_base_private dependency to enable
# the use of headers which must not be exposed to the libcamera public api.
libcamera_base_args = [ '-DLIBCAMERA_BASE_PRIVATE' ]

libcamera_base_lib = shared_library('libcamera-base',
                                    [libcamera_base_sources, libcamera_base_headers],
                                    version : libcamera_version,
                                    soversion : libcamera_soversion,
                                    name_prefix : '',
                                    install : true,
                                    cpp_args : libcamera_base_args,
                                    include_directories : libcamera_includes,
                                    dependencies : libcamera_base_deps)

libcamera_base = declare_dependency(sources : [
                                        libcamera_base_headers,
                                    ],
                                    include_directories : libcamera_includes,
                                    link_with : libcamera_base_lib)

pkg_mod = import('pkgconfig')
pkg_mod.generate(libcamera_base_lib,
                 description : 'Camera support base utility library',
                 subdirs : 'libcamera')

libcamera_base_private = declare_dependency(dependencies : libcamera_base,
                                            compile_args : libcamera_base_args)
