# SPDX-License-Identifier: CC0-1.0

if not gst_enabled
    subdir_done()
endif

gstreamer_tests = [
    {'name': 'single_stream_test', 'sources': ['gstreamer_single_stream_test.cpp']},
    {'name': 'multi_stream_test', 'sources': ['gstreamer_multi_stream_test.cpp']},
]
gstreamer_dep = dependency('gstreamer-1.0', required: true)

foreach test : gstreamer_tests
    exe = executable(test['name'], test['sources'], 'gstreamer_test.cpp',
                     dependencies : [libcamera_private, gstreamer_dep],
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'gstreamer', is_parallel : false)
endforeach
