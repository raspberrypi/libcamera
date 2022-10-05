# SPDX-License-Identifier: CC0-1.0

subdir('generated_serializer')

serialization_tests = [
    {'name': 'control_serialization', 'sources': ['control_serialization.cpp']},
    {'name': 'ipa_data_serializer_test', 'sources': ['ipa_data_serializer_test.cpp']},
]

foreach test : serialization_tests
    exe = executable(test['name'], test['sources'], 'serialization_test.cpp',
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)
    test(test['name'], exe, suite : 'serialization', is_parallel : false)
endforeach
