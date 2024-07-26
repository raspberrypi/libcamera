# SPDX-License-Identifier: CC0-1.0

rkisp1_ipa_test = [
    {'name': 'rkisp1-utils', 'sources': ['rkisp1-utils.cpp']},
]

foreach test : rkisp1_ipa_test
    exe = executable(test['name'], test['sources'], libcamera_generated_ipa_headers,
                     dependencies : [libcamera_private, libipa_dep],
                     link_with : [test_libraries],
                     include_directories : [test_includes_internal,
                                            '../../../src/ipa/rkisp1/'])

    test(test['name'], exe, suite : 'ipa')
endforeach
