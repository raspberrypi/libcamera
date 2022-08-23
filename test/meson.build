# SPDX-License-Identifier: CC0-1.0

if not get_option('test')
    test_enabled = false
    subdir_done()
endif

test_enabled = true

subdir('libtest')

subdir('camera')
subdir('controls')
subdir('gstreamer')
subdir('ipa')
subdir('ipc')
subdir('log')
subdir('media_device')
subdir('pipeline')
subdir('process')
subdir('py')
subdir('serialization')
subdir('stream')
subdir('v4l2_compat')
subdir('v4l2_subdevice')
subdir('v4l2_videodevice')

public_tests = [
    ['color-space',                     'color-space.cpp'],
    ['geometry',                        'geometry.cpp'],
    ['public-api',                      'public-api.cpp'],
    ['signal',                          'signal.cpp'],
    ['span',                            'span.cpp'],
]

internal_tests = [
    ['bayer-format',                    'bayer-format.cpp'],
    ['byte-stream-buffer',              'byte-stream-buffer.cpp'],
    ['camera-sensor',                   'camera-sensor.cpp'],
    ['delayed_controls',                'delayed_controls.cpp'],
    ['event',                           'event.cpp'],
    ['event-dispatcher',                'event-dispatcher.cpp'],
    ['event-thread',                    'event-thread.cpp'],
    ['file',                            'file.cpp'],
    ['flags',                           'flags.cpp'],
    ['hotplug-cameras',                 'hotplug-cameras.cpp'],
    ['message',                         'message.cpp'],
    ['object',                          'object.cpp'],
    ['object-delete',                   'object-delete.cpp'],
    ['object-invoke',                   'object-invoke.cpp'],
    ['pixel-format',                    'pixel-format.cpp'],
    ['shared-fd',                       'shared-fd.cpp'],
    ['signal-threads',                  'signal-threads.cpp'],
    ['threads',                         'threads.cpp'],
    ['timer',                           'timer.cpp'],
    ['timer-thread',                    'timer-thread.cpp'],
    ['unique-fd',                       'unique-fd.cpp'],
    ['utils',                           'utils.cpp'],
    ['yaml-parser',                     'yaml-parser.cpp'],
]

internal_non_parallel_tests = [
    ['fence',                           'fence.cpp'],
    ['mapped-buffer',                   'mapped-buffer.cpp'],
]

foreach t : public_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_public,
                     link_with : test_libraries,
                     include_directories : test_includes_public)

    test(t[0], exe)
endforeach

foreach t : internal_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe)
endforeach

foreach t : internal_non_parallel_tests
    exe = executable(t[0], t[1],
                     dependencies : libcamera_private,
                     link_with : test_libraries,
                     include_directories : test_includes_internal)

    test(t[0], exe, is_parallel : false)
endforeach
