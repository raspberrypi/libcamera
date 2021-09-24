# SPDX-License-Identifier: CC0-1.0

if not gst_enabled
    subdir_done()
endif

gstreamer_tests = [
    ['single_stream_test',   'gstreamer_single_stream_test.cpp'],
    ['multi_stream_test',    'gstreamer_multi_stream_test.cpp'],
]
gstreamer_dep = dependency('gstreamer-1.0', required: true)

foreach t : gstreamer_tests
    exe = executable(t[0], t[1], 'gstreamer_test.cpp',
                     dependencies : [libcamera_private, gstreamer_dep],
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'gstreamer', is_parallel : false)
endforeach
