# SPDX-License-Identifier: CC0-1.0

if not gst_enabled
    subdir_done()
endif

gstreamer_tests = [
    {'name': 'single_stream_test', 'sources': ['gstreamer_single_stream_test.cpp']},
    {'name': 'multi_stream_test', 'sources': ['gstreamer_multi_stream_test.cpp']},
    {'name': 'device_provider_test', 'sources': ['gstreamer_device_provider_test.cpp']},
    {'name': 'memory_lifetime_test', 'sources': ['gstreamer_memory_lifetime_test.cpp']},
]
gstreamer_dep = dependency('gstreamer-1.0', required : true)
gstapp_dep = dependency('gstreamer-app-1.0', required : true)

gstreamer_test_args = []

if asan_enabled
    gstreamer_test_args += ['-D', 'HAVE_ASAN=1']
endif

foreach test : gstreamer_tests
    exe = executable(test['name'], test['sources'], 'gstreamer_test.cpp',
                     cpp_args : gstreamer_test_args,
                     dependencies : [libcamera_private, gstreamer_dep, gstapp_dep],
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'gstreamer', is_parallel : false,
         env : gst_env, should_fail : test.get('should_fail', false))
endforeach
