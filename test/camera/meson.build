# SPDX-License-Identifier: CC0-1.0

# Tests are listed in order of complexity.
# They are not alphabetically sorted.
camera_tests = [
    ['configuration_default',   'configuration_default.cpp'],
    ['configuration_set',       'configuration_set.cpp'],
    ['buffer_import',           'buffer_import.cpp'],
    ['statemachine',            'statemachine.cpp'],
    ['capture',                 'capture.cpp'],
    ['camera_reconfigure',      'camera_reconfigure.cpp'],
]

foreach test : camera_tests
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test[0], exe, suite : 'camera', is_parallel : false)
endforeach
