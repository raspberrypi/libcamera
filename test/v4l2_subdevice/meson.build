# SPDX-License-Identifier: CC0-1.0

v4l2_subdevice_tests = [
    {'name': 'list_formats', 'sources': ['list_formats.cpp']},
    {'name': 'test_formats', 'sources': ['test_formats.cpp']},
]

foreach test : v4l2_subdevice_tests
    exe = executable(test['name'], test['sources'], 'v4l2_subdevice_test.cpp',
        dependencies : libcamera_private,
        link_with : test_libraries,
        include_directories : test_includes_internal)
    test(test['name'], exe, suite : 'v4l2_subdevice', is_parallel : false)
endforeach
