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
    'environment.cpp',
    'helpers/capture.cpp',
    'main.cpp',
    'test_base.cpp',
    'tests/capture_test.cpp',
])

lc_compliance_includes = ([
    include_directories('.'),
    include_directories('helpers/')
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
                            include_directories : lc_compliance_includes,
                            install : true,
                            install_tag : 'bin-devel')
