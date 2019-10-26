libcamera_sources = files([
    'bound_method.cpp',
    'buffer.cpp',
    'byte_stream_buffer.cpp',
    'camera.cpp',
    'camera_controls.cpp',
    'camera_manager.cpp',
    'camera_sensor.cpp',
    'controls.cpp',
    'control_serializer.cpp',
    'control_validator.cpp',
    'device_enumerator.cpp',
    'device_enumerator_sysfs.cpp',
    'event_dispatcher.cpp',
    'event_dispatcher_poll.cpp',
    'event_notifier.cpp',
    'formats.cpp',
    'geometry.cpp',
    'ipa_controls.cpp',
    'ipa_interface.cpp',
    'ipa_manager.cpp',
    'ipa_module.cpp',
    'ipa_proxy.cpp',
    'ipc_unixsocket.cpp',
    'log.cpp',
    'media_device.cpp',
    'media_object.cpp',
    'message.cpp',
    'object.cpp',
    'pipeline_handler.cpp',
    'pixelformats.cpp',
    'process.cpp',
    'request.cpp',
    'semaphore.cpp',
    'signal.cpp',
    'stream.cpp',
    'thread.cpp',
    'timer.cpp',
    'utils.cpp',
    'v4l2_controls.cpp',
    'v4l2_device.cpp',
    'v4l2_subdevice.cpp',
    'v4l2_videodevice.cpp',
])

subdir('include')

libcamera_internal_includes =  include_directories('include')

includes = [
    libcamera_includes,
    libcamera_internal_includes,
]

subdir('pipeline')
subdir('proxy')

libudev = dependency('libudev', required : false)

if libudev.found()
    config_h.set('HAVE_LIBUDEV', 1)
    libcamera_sources += files([
        'device_enumerator_udev.cpp',
    ])
endif

gen_controls = files('gen-controls.py')

control_ids_cpp = custom_target('control_ids_cpp',
                                input : files('control_ids.yaml', 'control_ids.cpp.in'),
                                output : 'control_ids.cpp',
                                depend_files : gen_controls,
                                command : [gen_controls, '-o', '@OUTPUT@', '@INPUT@'])

libcamera_sources += control_ids_cpp
libcamera_sources += control_ids_h

gen_version = join_paths(meson.source_root(), 'utils', 'gen-version.sh')

version_cpp = vcs_tag(command : [gen_version, meson.build_root()],
                      input : 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

libcamera_sources += version_cpp

libcamera_deps = [
    cc.find_library('atomic', required: false),
    cc.find_library('dl'),
    libudev,
    dependency('threads'),
]

libcamera_link_with = []

if get_option('android')
    libcamera_sources += android_hal_sources
    includes += android_includes
    libcamera_link_with += android_camera_metadata
endif

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           link_with : libcamera_link_with,
                           include_directories : includes,
                           dependencies : libcamera_deps)

libcamera_dep = declare_dependency(sources : [libcamera_api, libcamera_ipa_api, libcamera_h],
                                   include_directories : libcamera_includes,
                                   link_with : libcamera)

subdir('proxy/worker')
