# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : get_option('cam'))

if not libevent.found()
    cam_enabled = false
    subdir_done()
endif

cam_enabled = true

cam_sources = files([
    'camera_session.cpp',
    'event_loop.cpp',
    'file_sink.cpp',
    'frame_sink.cpp',
    'main.cpp',
    'options.cpp',
    'stream_options.cpp',
])

cam  = executable('cam', cam_sources,
                  dependencies : [
                      libatomic,
                      libcamera_public,
                      libevent,
                  ],
                  install : true)
