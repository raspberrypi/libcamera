subdir('libtest')

subdir('camera')
subdir('ipa')
subdir('media_device')
subdir('pipeline')
subdir('v4l2_device')
subdir('v4l2_subdevice')

public_tests = [
    ['event',                           'event.cpp'],
    ['event-dispatcher',                'event-dispatcher.cpp'],
    ['geometry',                        'geometry.cpp'],
    ['list-cameras',                    'list-cameras.cpp'],
    ['signal',                          'signal.cpp'],
    ['timer',                           'timer.cpp'],
]

internal_tests = [
]

foreach t : public_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_public)

    test(t[0], exe)
endforeach

foreach t : internal_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_dep,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe)
endforeach
