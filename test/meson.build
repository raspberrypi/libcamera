subdir('libtest')

subdir('media_device')
subdir('pipeline')
subdir('v4l2_device')

public_tests = [
    ['event',           'event.cpp'],
    ['list-cameras',    'list-cameras.cpp'],
    ['signal',          'signal.cpp'],
    ['timer',           'timer.cpp'],
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
