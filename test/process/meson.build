# SPDX-License-Identifier: CC0-1.0

process_tests = [
    {'name': 'process_test', 'sources': ['process_test.cpp']},
]

foreach test : process_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'process', is_parallel : false)
endforeach
