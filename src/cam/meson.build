# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : false)

if not libevent.found()
    warning('libevent_pthreads not found, \'cam\' application will not be compiled')
    subdir_done()
endif

cam_sources = files([
    'buffer_writer.cpp',
    'capture.cpp',
    'event_loop.cpp',
    'main.cpp',
    'options.cpp',
    'stream_options.cpp',
])

cam  = executable('cam', cam_sources,
                  dependencies : [
                      libatomic,
                      libcamera_dep,
                      libevent,
                  ],
                  install : true)
