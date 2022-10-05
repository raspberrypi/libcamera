# SPDX-License-Identifier: CC0-1.0

ipa_test = [
    ['ipa_module_test',     'ipa_module_test.cpp'],
    ['ipa_interface_test',  'ipa_interface_test.cpp'],
]

foreach test : ipa_test
    exe = executable(test[0], [test[1], libcamera_generated_ipa_headers],
                     dependencies : libcamera_private,
                     link_with : [libipa, test_libraries],
                     include_directories : [libipa_includes, test_includes_internal])

    test(test[0], exe, suite : 'ipa')
endforeach
