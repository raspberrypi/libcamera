subdir('libtest')

test_libraries = [libcamera, libtest]

test_includes_public = [
    libtest_includes,
    libcamera_includes,
]

test_includes_internal = [
    test_includes_public,
    libcamera_internal_includes,
]

list = executable('list', 'list.cpp',
                  link_with : test_libraries,
                  include_directories : test_includes_public)

subdir('media_device')

test('List Camera API tests', list)
test('Media Device Test', media_device_test)
