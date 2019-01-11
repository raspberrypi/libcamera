media_device_tests = [
    ['media_device_print_test',         'media_device_print_test.cpp'],
    ['media_device_link_test',          'media_device_link_test.cpp'],
]

foreach t : media_device_tests
    exe = executable(t[0], t[1],
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite: 'media_device', is_parallel: false)
endforeach
