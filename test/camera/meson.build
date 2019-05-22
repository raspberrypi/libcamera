# Tests are listed in order of complexity.
# They are not alphabetically sorted.
camera_tests = [
    [ 'configuration_default',  'configuration_default.cpp' ],
    [ 'configuration_set',      'configuration_set.cpp' ],
    [ 'statemachine',           'statemachine.cpp' ],
    [ 'capture',                'capture.cpp' ],
]

foreach t : camera_tests
    exe = executable(t[0], [t[1], 'camera_test.cpp'],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(t[0], exe, suite : 'camera', is_parallel : false)
endforeach
