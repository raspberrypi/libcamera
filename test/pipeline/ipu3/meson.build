# SPDX-License-Identifier: CC0-1.0

ipu3_test = [
    {'name': 'ipu3_pipeline_test', 'sources': ['ipu3_pipeline_test.cpp']},
]

foreach test : ipu3_test
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'ipu3', is_parallel : false)
endforeach
