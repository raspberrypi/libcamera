# SPDX-License-Identifier: CC0-1.0

process_tests = [
    ['process_test',   'process_test.cpp'],
]

foreach test : process_tests
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test[0], exe, suite : 'process', is_parallel : false)
endforeach
