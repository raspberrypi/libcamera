# SPDX-License-Identifier: CC0-1.0

ipu3_test = [
    ['ipu3_pipeline_test',            'ipu3_pipeline_test.cpp'],
]

foreach test : ipu3_test
    exe = executable(test[0], test[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(test[0], exe, suite : 'ipu3', is_parallel : false)
endforeach
