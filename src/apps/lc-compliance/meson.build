# SPDX-License-Identifier: CC0-1.0

libgtest = dependency('gtest', version : '>=1.10.0',
                      required : get_option('lc-compliance'),
                      fallback : ['gtest', 'gtest_dep'])

if opt_lc_compliance.disabled() or not libevent.found() or not libgtest.found()
    lc_compliance_enabled = false
    subdir_done()
endif

lc_compliance_enabled = true

lc_compliance_sources = files([
    'capture_test.cpp',
    'environment.cpp',
    'main.cpp',
    'simple_capture.cpp',
])

lc_compliance  = executable('lc-compliance', lc_compliance_sources,
                            cpp_args : [ '-fexceptions' ],
                            link_with : apps_lib,
                            dependencies : [
                                libatomic,
                                libcamera_public,
                                libevent,
                                libgtest,
                            ],
                            install : true,
                            install_tag : 'bin-devel')
