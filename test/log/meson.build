# SPDX-License-Identifier: CC0-1.0

log_test = [
    ['log_api',     'log_api.cpp'],
    ['log_process', 'log_process.cpp'],
]

foreach test : log_test
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test[0], exe, suite : 'log')
endforeach
