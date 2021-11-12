# SPDX-License-Identifier: CC0-1.0

libevent = dependency('libevent_pthreads', required : get_option('lc-compliance'))

if not libevent.found()
    lc_compliance_enabled = false
    subdir_done()
endif

if get_option('android_platform') == 'cros'
    libgtest = dependency('gtest', required : get_option('lc-compliance'))

    if not libgtest.found()
        lc_compliance_enabled = false
        subdir_done()
    endif

else
    libgtest_sp = subproject('gtest')
    libgtest = libgtest_sp.get_variable('gtest_dep')
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
