# SPDX-License-Identifier: CC0-1.0

ipa_test = [
    ['ipa_module_test',     'ipa_module_test.cpp'],
    ['ipa_interface_test',  'ipa_interface_test.cpp'],
]

foreach t : ipa_test
    exe = executable(t[0], [t[1], libcamera_generated_ipa_headers],
                     dependencies : libcamera_private,
                     link_with : [libipa, test_libraries],
                     include_directories : [libipa_includes, test_includes_internal])

    test(t[0], exe, suite : 'ipa')
endforeach
