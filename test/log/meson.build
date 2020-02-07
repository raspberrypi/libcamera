# SPDX-License-Identifier: CC0-1.0

log_test = [
    ['log_api',     'log_api.cpp'],
    ['log_process', 'log_process.cpp'],
]

foreach t : log_test
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'log')
endforeach
