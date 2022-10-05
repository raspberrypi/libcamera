# SPDX-License-Identifier: CC0-1.0

control_tests = [
    ['control_info',                'control_info.cpp'],
    ['control_info_map',            'control_info_map.cpp'],
    ['control_list',                'control_list.cpp'],
    ['control_value',               'control_value.cpp'],
]

foreach test : control_tests
    exe = executable(test[0], test[1],
                     dependencies : libcamera_public,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test[0], exe, suite : 'controls', is_parallel : false)
endforeach
