# SPDX-License-Identifier: CC0-1.0

log_test = [
    {'name': 'log_api', 'sources': ['log_api.cpp']},
    {'name': 'log_process', 'sources': ['log_process.cpp']},
]

foreach test : log_test
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'log')
endforeach
