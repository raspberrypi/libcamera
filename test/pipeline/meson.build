# SPDX-License-Identifier: CC0-1.0

pipeline_tests = [
    {'name': 'duplicate_camera_id', 'sources': ['duplicate_camera_id.cpp']},
]

foreach test : pipeline_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'pipeline', is_parallel : true)
endforeach
