libcamera_sources = files([
    'buffer.cpp',
    'camera.cpp',
    'camera_manager.cpp',
    'device_enumerator.cpp',
    'event_dispatcher.cpp',
    'event_dispatcher_poll.cpp',
    'event_notifier.cpp',
    'geometry.cpp',
    'log.cpp',
    'media_device.cpp',
    'media_object.cpp',
    'pipeline_handler.cpp',
    'request.cpp',
    'signal.cpp',
    'stream.cpp',
    'timer.cpp',
    'v4l2_device.cpp',
    'v4l2_subdevice.cpp',
])

libcamera_headers = files([
    'include/device_enumerator.h',
    'include/event_dispatcher_poll.h',
    'include/log.h',
    'include/media_device.h',
    'include/media_object.h',
    'include/pipeline_handler.h',
    'include/utils.h',
    'include/v4l2_device.h',
])

libcamera_internal_includes =  include_directories('include')

includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

subdir('pipeline')

libudev = dependency('libudev')

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           include_directories : includes,
                           dependencies : libudev)
