# SPDX-License-Identifier: CC0-1.0

# Tests are listed in order of complexity.
# They are not alphabetically sorted.
v4l2_videodevice_tests = [
    [ 'double_open',        'double_open.cpp' ],
    [ 'controls',           'controls.cpp' ],
    [ 'formats',            'formats.cpp' ],
    [ 'request_buffers',    'request_buffers.cpp' ],
    [ 'buffer_cache',       'buffer_cache.cpp' ],
    [ 'stream_on_off',      'stream_on_off.cpp' ],
    [ 'capture_async',      'capture_async.cpp' ],
    [ 'buffer_sharing',     'buffer_sharing.cpp' ],
    [ 'v4l2_m2mdevice',     'v4l2_m2mdevice.cpp' ],
]

foreach t : v4l2_videodevice_tests
    exe = executable(t[0], [t[1], 'v4l2_videodevice_test.cpp'],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(t[0], exe, suite : 'v4l2_videodevice', is_parallel : false)
endforeach
