control_tests = [
    [ 'control_info',   'control_info.cpp' ],
    [ 'control_list',   'control_list.cpp' ],
    [ 'control_value',  'control_value.cpp' ],
]

foreach t : control_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(t[0], exe, suite : 'controls', is_parallel : false)
endforeach
