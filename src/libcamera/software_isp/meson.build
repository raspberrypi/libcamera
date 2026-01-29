# SPDX-License-Identifier: CC0-1.0

softisp_enabled = pipelines.contains('simple')
summary({'SoftISP support' : softisp_enabled}, section : 'Configuration')
summary({'SoftISP GPU acceleration' : mesa_works}, section : 'Configuration')

if not softisp_enabled
    subdir_done()
endif

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
endif
