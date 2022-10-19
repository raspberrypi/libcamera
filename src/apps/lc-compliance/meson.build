# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : get_option('lc-compliance'))
libgtest = dependency('gtest', required : get_option('lc-compliance'),
                      fallback : ['gtest', 'gtest_dep'])

if not (libevent.found() and libgtest.found())
    lc_compliance_enabled = false
    subdir_done()
endif

lc_compliance_enabled = true

lc_compliance_sources = files([
    '../cam/event_loop.cpp',
    '../cam/options.cpp',
    'environment.cpp',
    'main.cpp',
    'simple_capture.cpp',
    'capture_test.cpp',
])

lc_compliance  = executable('lc-compliance', lc_compliance_sources,
                            cpp_args : [ '-fexceptions' ],
                            dependencies : [
                                libatomic,
                                libcamera_public,
                                libevent,
                                libgtest,
                            ],
                            install : true)
