# SPDX-License-Identifier: CC0-1.0

subdir('include/libcamera/ipa')

exe = executable('generated_serializer_test',
                 [
                     'generated_serializer_test.cpp',
                     generated_test_header,
                     generated_test_serializer,
                 ],
                 dependencies : libcamera_private,
                 link_with : test_libraries,
                 include_directories : [
                     test_includes_internal,
                     './include',
                 ])

test('generated_serializer_test', exe,
     suite : 'generated_serializer', is_parallel : false)
