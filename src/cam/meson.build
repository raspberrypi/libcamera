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
    'image.cpp',
    'main.cpp',
    'options.cpp',
    'stream_options.cpp',
])

cam_cpp_args = []

libdrm = dependency('libdrm', required : false)

if libdrm.found()
cam_cpp_args += [ '-DHAVE_KMS' ]
cam_sources += files([
    'drm.cpp',
    'kms_sink.cpp'
])
endif

cam  = executable('cam', cam_sources,
                  dependencies : [
                      libatomic,
                      libcamera_public,
                      libdrm,
                      libevent,
                  ],
                  cpp_args : cam_cpp_args,
                  install : true)
