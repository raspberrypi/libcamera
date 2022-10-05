# SPDX-License-Identifier: CC0-1.0

lib_mdev_test_sources = files([
    'media_device_test.cpp',
])

media_device_tests = [
    ['media_device_acquire',            'media_device_acquire.cpp'],
    ['media_device_print_test',         'media_device_print_test.cpp'],
    ['media_device_link_test',          'media_device_link_test.cpp'],
]

lib_mdev_test = static_library('lib_mdev_test', lib_mdev_test_sources,
                               dependencies : libcamera_private,
                               include_directories : test_includes_internal)

foreach test : media_device_tests
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : [test_libraries, lib_mdev_test],
                     include_directories : test_includes_internal)

    test(test[0], exe, suite : 'media_device', is_parallel : false)
endforeach
