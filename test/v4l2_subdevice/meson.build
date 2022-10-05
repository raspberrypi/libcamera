# SPDX-License-Identifier: CC0-1.0

v4l2_subdevice_tests = [
  ['list_formats',              'list_formats.cpp'],
  ['test_formats',              'test_formats.cpp'],
]

foreach test : v4l2_subdevice_tests
    exe = executable(test[0], [test[1], 'v4l2_subdevice_test.cpp'],
        dependencies : libcamera_private,
        link_with : test_libraries,
        include_directories : test_includes_internal)
    test(test[0], exe, suite : 'v4l2_subdevice', is_parallel : false)
endforeach
