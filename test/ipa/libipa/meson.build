# SPDX-License-Identifier: CC0-1.0

libipa_test = [
    {'name': 'interpolator', 'sources': ['interpolator.cpp']},
    {'name': 'vector', 'sources': ['vector.cpp']},
]

foreach test : libipa_test
    exe = executable(test['name'], test['sources'],
                     dependencies : [libcamera_private, libipa_dep],
                     implicit_include_directories : false,
                     link_with : [test_libraries],
                     include_directories : [test_includes_internal,
                                            '../../../src/ipa/libipa/'])

    test(test['name'], exe, suite : 'ipa')
endforeach
