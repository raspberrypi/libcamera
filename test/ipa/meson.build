ipa_modules_sources = [
    ['ipa-dummy-c',   'shared_test.c'],
    ['ipa-dummy-cpp', 'shared_test.cpp'],
]

foreach m : ipa_modules_sources
    shared_library(m, name_prefix : '',
                   dependencies : libcamera_dep,
                   include_directories : test_includes_public)
endforeach

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
