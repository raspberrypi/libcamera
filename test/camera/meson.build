# SPDX-License-Identifier: CC0-1.0

# Tests are listed in order of complexity.
# They are not alphabetically sorted.
camera_tests = [
    {'name': 'configuration_default', 'sources': ['configuration_default.cpp']},
    {'name': 'configuration_set', 'sources': ['configuration_set.cpp']},
    {'name': 'buffer_import', 'sources': ['buffer_import.cpp']},
    {'name': 'statemachine', 'sources': ['statemachine.cpp']},
    {'name': 'capture', 'sources': ['capture.cpp']},
    {'name': 'camera_reconfigure', 'sources': ['camera_reconfigure.cpp']},
]

foreach test : camera_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test['name'], exe, suite : 'camera', is_parallel : false)
endforeach
