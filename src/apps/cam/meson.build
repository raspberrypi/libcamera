# SPDX-License-Identifier: CC0-1.0

if opt_cam.disabled() or not libevent.found()
    cam_enabled = false
    subdir_done()
endif

cam_enabled = true

cam_sources = files([
    'camera_session.cpp',
    'capture_script.cpp',
    'file_sink.cpp',
    'frame_sink.cpp',
    'main.cpp',
])

cam_cpp_args = [apps_cpp_args]

cam_deps = [
    libatomic,
    libcamera_public,
    libevent,
    libthreads,
    libyaml,
    libtiff,
]

libdrm = dependency('libdrm', required : get_option('cam-output-kms'))
libsdl2 = dependency('SDL2', required : get_option('cam-output-sdl2'))
libjpeg = dependency('libjpeg', required : get_option('cam-jpeg'))

if libdrm.found()
    cam_cpp_args += [ '-DHAVE_KMS' ]
    cam_sources += files([
        'drm.cpp',
        'kms_sink.cpp'
    ])
    cam_deps += [
        libdrm,
    ]
endif

if libsdl2.found()
    cam_cpp_args += ['-DHAVE_SDL']
    cam_sources += files([
        'sdl_sink.cpp',
        'sdl_texture.cpp',
        'sdl_texture_1plane.cpp',
        'sdl_texture_yuv.cpp',
    ])
    cam_deps += [
        libsdl2,
    ]

    if libjpeg.found()
        cam_cpp_args += ['-DHAVE_LIBJPEG']
        cam_sources += files([
            'sdl_texture_mjpg.cpp'
        ])
        cam_deps += [
            libjpeg,
        ]
    endif
endif

cam  = executable('cam', cam_sources,
                  link_with : apps_lib,
                  dependencies : cam_deps,
                  cpp_args : cam_cpp_args,
                  install : true,
                  install_tag : 'bin')
