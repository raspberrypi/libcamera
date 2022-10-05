# SPDX-License-Identifier: CC0-1.0

# Tests are listed in order of complexity.
# They are not alphabetically sorted.
v4l2_videodevice_tests = [
    {'name': 'double_open', 'sources': ['double_open.cpp']},
    {'name': 'controls', 'sources': ['controls.cpp']},
    {'name': 'formats', 'sources': ['formats.cpp']},
    {'name': 'dequeue_watchdog', 'sources': ['dequeue_watchdog.cpp']},
    {'name': 'request_buffers', 'sources': ['request_buffers.cpp']},
    {'name': 'buffer_cache', 'sources': ['buffer_cache.cpp']},
    {'name': 'stream_on_off', 'sources': ['stream_on_off.cpp']},
    {'name': 'capture_async', 'sources': ['capture_async.cpp']},
    {'name': 'buffer_sharing', 'sources': ['buffer_sharing.cpp']},
    {'name': 'v4l2_m2mdevice', 'sources': ['v4l2_m2mdevice.cpp']},
]

foreach test : v4l2_videodevice_tests
    exe = executable(test['name'], [test['sources'], 'v4l2_videodevice_test.cpp'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test['name'], exe, suite : 'v4l2_videodevice', is_parallel : false)
endforeach
