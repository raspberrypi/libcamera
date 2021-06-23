# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : get_option('lc-compliance'))

if not libevent.found()
    lc_compliance_enabled = false
    subdir_done()
endif

lc_compliance_enabled = true

lc_compliance_sources = files([
    '../cam/event_loop.cpp',
    '../cam/options.cpp',
    'main.cpp',
    'results.cpp',
    'simple_capture.cpp',
    'single_stream.cpp',
])

lc_compliance  = executable('lc-compliance', lc_compliance_sources,
                            dependencies : [
                                libatomic,
                                libcamera_public,
                                libevent,
                            ],
                            install : true)
