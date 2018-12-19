libtest_sources = files([
    'test.cpp',
])

libtest = static_library('libtest', libtest_sources)

test_init = executable('test_init', 'init.cpp',
                       link_with : libcamera,
                       include_directories : libcamera_includes)

test('Initialisation test', test_init)
