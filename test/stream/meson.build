# SPDX-License-Identifier: CC0-1.0

stream_tests = [
    {'name': 'stream_colorspace', 'sources': ['stream_colorspace.cpp']},
    {'name': 'stream_formats', 'sources': ['stream_formats.cpp']},
]

foreach test : stream_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_public,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test['name'], exe, suite: 'stream')
endforeach
