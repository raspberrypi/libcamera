# Tests are listed in order of complexity.
# They are not alphabetically sorted.
v4l2_device_tests = [
  [ 'double_open',        'double_open.cpp' ],
  [ 'request_buffers',    'request_buffers.cpp' ],
]

foreach t : v4l2_device_tests
  exe = executable(t[0], [t[1], 'v4l2_device_test.cpp'],
		   link_with : test_libraries,
		   include_directories : test_includes_internal)
  test(t[0], exe, suite: 'v4l2_device', is_parallel: false)
endforeach
