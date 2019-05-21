libcamera_sources = files([
    'buffer.cpp',
    'camera.cpp',
    'camera_manager.cpp',
    'camera_sensor.cpp',
    'device_enumerator.cpp',
    'device_enumerator_sysfs.cpp',
    'event_dispatcher.cpp',
    'event_dispatcher_poll.cpp',
    'event_notifier.cpp',
    'formats.cpp',
    'geometry.cpp',
    'ipa_module.cpp',
    'log.cpp',
    'media_device.cpp',
    'media_object.cpp',
    'object.cpp',
    'pipeline_handler.cpp',
    'request.cpp',
    'signal.cpp',
    'stream.cpp',
    'timer.cpp',
    'utils.cpp',
    'v4l2_device.cpp',
    'v4l2_subdevice.cpp',
])

libcamera_headers = files([
    'include/camera_sensor.h',
    'include/device_enumerator.h',
    'include/device_enumerator_sysfs.h',
    'include/device_enumerator_udev.h',
    'include/event_dispatcher_poll.h',
    'include/formats.h',
    'include/ipa_module.h',
    'include/log.h',
    'include/media_device.h',
    'include/media_object.h',
    'include/pipeline_handler.h',
    'include/utils.h',
    'include/v4l2_device.h',
    'include/v4l2_subdevice.h',
])

libcamera_internal_includes =  include_directories('include')

includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

subdir('pipeline')

libudev = dependency('libudev', required : false)

if libudev.found()
    config_h.set('HAVE_LIBUDEV', 1)
    libcamera_sources += files([
        'device_enumerator_udev.cpp',
    ])
endif

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           include_directories : includes,
                           dependencies : libudev)

libcamera_dep = declare_dependency(sources : [libcamera_api, libcamera_h],
                                   include_directories : libcamera_includes,
                                   link_with : libcamera)
