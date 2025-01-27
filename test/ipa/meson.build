# SPDX-License-Identifier: CC0-1.0

subdir('libipa')

ipa_test = [
    {'name': 'ipa_module_test', 'sources': ['ipa_module_test.cpp']},
    {'name': 'ipa_interface_test', 'sources': ['ipa_interface_test.cpp']},
]

foreach test : ipa_test
    exe = executable(test['name'], test['sources'],
                     dependencies : [libcamera_private, libipa_dep],
                     link_with : [test_libraries],
                     include_directories : [test_includes_internal])

    test(test['name'], exe, suite : 'ipa')
endforeach
