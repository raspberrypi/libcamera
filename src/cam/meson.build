# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : get_option('cam'))

if not libevent.found()
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
