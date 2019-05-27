ipa_test = [
    ['ipa_test', 'ipa_test.cpp'],
]

foreach t : ipa_test
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, suite : 'ipa', is_parallel : false)
endforeach
