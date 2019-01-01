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

subdir('media_device')

public_tests = [
    ['list',            'list.cpp'],
]

internal_tests = [
]

foreach t : public_tests
    exe = executable(t[0], t[1],
                     link_with : test_libraries,
                     include_directories : test_includes_public)

    test(t[0], exe)
endforeach

foreach t : internal_tests
    exe = executable(t[0], t[1],
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe)
endforeach
