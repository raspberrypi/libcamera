# SPDX-License-Identifier: CC0-1.0

softisp_enabled = pipelines.contains('simple')
summary({'SoftISP support' : softisp_enabled}, section : 'Configuration')

if not softisp_enabled
    subdir_done()
endif

libegl = dependency('egl', required : get_option('softisp-gpu'))
libglesv2 = dependency('glesv2', required : get_option('softisp-gpu'))
mesa_works = cc.check_header('EGL/egl.h',
                             required : get_option('softisp-gpu'),
                             dependencies : libegl)

summary({'SoftISP GPU acceleration' : mesa_works}, section : 'Configuration')

libcamera_internal_sources += files([
    'benchmark.cpp',
    'debayer.cpp',
    'debayer_cpu.cpp',
    'software_isp.cpp',
    'swstats_cpu.cpp',
])

if mesa_works
    config_h.set('HAVE_DEBAYER_EGL', 1)
    libcamera_internal_sources += files([
        '../egl.cpp',
        'debayer_egl.cpp',
    ])
    libcamera_deps += [
        libegl,
        libglesv2,
    ]
endif
