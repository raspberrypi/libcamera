ipu3_test = [
    ['ipu3_pipeline_test',            'ipu3_pipeline_test.cpp'],
]

foreach t : ipu3_test
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'ipu3', is_parallel : false)
endforeach
