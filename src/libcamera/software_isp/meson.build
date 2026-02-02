# SPDX-License-Identifier: CC0-1.0

softisp_enabled = pipelines.contains('simple')
summary({'SoftISP support' : softisp_enabled}, section : 'Configuration')
summary({'SoftISP GPU acceleration' : gles_headless_enabled}, section : 'Configuration')

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

if softisp_enabled and gles_headless_enabled
    config_h.set('HAVE_DEBAYER_EGL', 1)
    libcamera_internal_sources += files([
        'debayer_egl.cpp',
    ])
endif
