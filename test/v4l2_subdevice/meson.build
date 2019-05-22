v4l2_subdevice_tests = [
  [ 'list_formats',             'list_formats.cpp'],
  [ 'test_formats',             'test_formats.cpp'],
]

foreach t : v4l2_subdevice_tests
    exe = executable(t[0], [t[1], 'v4l2_subdevice_test.cpp'],
        dependencies : libcamera_dep,
        link_with : test_libraries,
        include_directories : test_includes_internal)
    test(t[0], exe, suite : 'v4l2_subdevice', is_parallel : false)
endforeach
