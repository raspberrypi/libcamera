subdir('libtest')

subdir('camera')
subdir('ipa')
subdir('ipc')
subdir('media_device')
subdir('pipeline')
subdir('stream')
subdir('v4l2_subdevice')
subdir('v4l2_videodevice')

public_tests = [
    ['event',                           'event.cpp'],
    ['event-dispatcher',                'event-dispatcher.cpp'],
    ['geometry',                        'geometry.cpp'],
    ['list-cameras',                    'list-cameras.cpp'],
    ['signal',                          'signal.cpp'],
    ['timer',                           'timer.cpp'],
]

internal_tests = [
    ['camera-sensor',                   'camera-sensor.cpp'],
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
