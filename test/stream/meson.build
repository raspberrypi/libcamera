# SPDX-License-Identifier: CC0-1.0

stream_tests = [
    [ 'stream_formats', 'stream_formats.cpp' ],
]

foreach t : stream_tests
  exe = executable(t[0], t[1],
                   dependencies : libcamera_dep,
                   link_with : test_libraries,
                   include_directories : test_includes_internal)
  test(t[0], exe, suite: 'stream')
endforeach
