libtest_sources = files([
    'test.cpp',
])

libtest = static_library('libtest', libtest_sources)
libtest_includes = include_directories('./')

test_init = executable('test_init', 'init.cpp',
                       link_with : libcamera,
                       include_directories : libcamera_includes)

list = executable('list', 'list.cpp',
                    link_with : [libcamera, libtest],
                    include_directories : libcamera_includes)

subdir('media_device')

test('Initialisation test', test_init)
test('List Camera API tests', list)
test('Media Device Test', media_device_test)
