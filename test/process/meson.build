# SPDX-License-Identifier: CC0-1.0

process_tests = [
    [ 'process_test',  'process_test.cpp' ],
]

foreach t : process_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'process', is_parallel : false)
endforeach
