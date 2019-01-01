libtest_sources = files([
    'test.cpp',
])

libtest = static_library('libtest', libtest_sources)

libtest_includes = include_directories('.')

test_libraries = [libcamera, libtest]

test_includes_public = [
    libtest_includes,
    libcamera_includes,
]

test_includes_internal = [
    test_includes_public,
    libcamera_internal_includes,
]
