lib_mdev_test_sources = files([
    'media_device_test.cpp',
])

media_device_tests = [
    ['media_device_acquire',            'media_device_acquire.cpp'],
    ['media_device_print_test',         'media_device_print_test.cpp'],
    ['media_device_link_test',          'media_device_link_test.cpp'],
]

lib_mdev_test = static_library('lib_mdev_test', lib_mdev_test_sources,
                               dependencies : libcamera_dep,
                               include_directories : test_includes_internal)

foreach t : media_device_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : [test_libraries, lib_mdev_test],
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'media_device', is_parallel : false)
endforeach
