# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : get_option('cam'))

if not libevent.found()
    cam_enabled = false
    subdir_done()
endif

cam_enabled = true

cam_sources = files([
    'camera_session.cpp',
    'capture_script.cpp',
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
libjpeg = dependency('libjpeg', required : false)
libsdl2 = dependency('SDL2', required : false)

if libdrm.found()
    cam_cpp_args += [ '-DHAVE_KMS' ]
    cam_sources += files([
        'drm.cpp',
        'kms_sink.cpp'
    ])
endif

if libsdl2.found()
    cam_cpp_args += ['-DHAVE_SDL']
    cam_sources += files([
        'sdl_sink.cpp',
        'sdl_texture.cpp',
        'sdl_texture_yuv.cpp',
    ])

    if libjpeg.found()
        cam_cpp_args += ['-DHAVE_LIBJPEG']
        cam_sources += files([
            'sdl_texture_mjpg.cpp'
        ])
    endif
endif

cam  = executable('cam', cam_sources,
                  dependencies : [
                      libatomic,
                      libcamera_public,
                      libdrm,
                      libevent,
                      libjpeg,
                      libsdl2,
                      libyaml,
                  ],
                  cpp_args : cam_cpp_args,
                  install : true)
