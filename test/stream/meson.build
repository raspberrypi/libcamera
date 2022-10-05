# SPDX-License-Identifier: CC0-1.0

stream_tests = [
    ['stream_colorspace', 'stream_colorspace.cpp'],
    ['stream_formats',  'stream_formats.cpp'],
]

foreach test : stream_tests
    exe = executable(test[0], test[1],
                     dependencies : libcamera_public,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test[0], exe, suite: 'stream')
endforeach
