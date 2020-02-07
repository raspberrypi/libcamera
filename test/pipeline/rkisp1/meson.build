# SPDX-License-Identifier: CC0-1.0

rkisp1_test = [
    ['rkisp1_pipeline_test',            'rkisp1_pipeline_test.cpp'],
]

foreach t : rkisp1_test
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'rkisp1', is_parallel : false)
endforeach
