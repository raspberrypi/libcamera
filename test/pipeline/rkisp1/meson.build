# SPDX-License-Identifier: CC0-1.0

rkisp1_test = [
    ['rkisp1_pipeline_test',            'rkisp1_pipeline_test.cpp'],
]

foreach test : rkisp1_test
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test[0], exe, suite : 'rkisp1', is_parallel : false)
endforeach
