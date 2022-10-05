# SPDX-License-Identifier: CC0-1.0

rkisp1_test = [
    {'name': 'rkisp1_pipeline_test', 'sources': ['rkisp1_pipeline_test.cpp']},
]

foreach test : rkisp1_test
    exe = executable(test['name'], test['sources'],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test['name'], exe, suite : 'rkisp1', is_parallel : false)
endforeach
