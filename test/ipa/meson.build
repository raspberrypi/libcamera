# SPDX-License-Identifier: CC0-1.0

ipa_test = [
    {'name': 'ipa_module_test', 'sources': ['ipa_module_test.cpp']},
    {'name': 'ipa_interface_test', 'sources': ['ipa_interface_test.cpp']},
]

foreach test : ipa_test
    exe = executable(test['name'], test['sources'], libcamera_generated_ipa_headers,
                     dependencies : libcamera_private,
                     link_with : [libipa, test_libraries],
                     include_directories : [libipa_includes, test_includes_internal])

    test(test['name'], exe, suite : 'ipa')
endforeach
