# SPDX-License-Identifier: CC0-1.0

lib_mdev_test_sources = files([
    'media_device_test.cpp',
])

media_device_tests = [
    {'name': 'media_device_acquire', 'sources': ['media_device_acquire.cpp']},
    {'name': 'media_device_print_test', 'sources': ['media_device_print_test.cpp']},
    {'name': 'media_device_link_test', 'sources': ['media_device_link_test.cpp']},
]

lib_mdev_test = static_library('lib_mdev_test', lib_mdev_test_sources,
                               dependencies : libcamera_private,
                               include_directories : test_includes_internal)

foreach test : media_device_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : [test_libraries, lib_mdev_test],
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'media_device', is_parallel : false)
endforeach
