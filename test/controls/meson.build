# SPDX-License-Identifier: CC0-1.0

control_tests = [
    {'name': 'control_info', 'sources': ['control_info.cpp']},
    {'name': 'control_info_map', 'sources': ['control_info_map.cpp']},
    {'name': 'control_list', 'sources': ['control_list.cpp']},
    {'name': 'control_value', 'sources': ['control_value.cpp']},
]

foreach test : control_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_public,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test['name'], exe, suite : 'controls', is_parallel : false)
endforeach
