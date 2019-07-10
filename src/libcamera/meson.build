libcamera_sources = files([
    'buffer.cpp',
    'camera.cpp',
    'camera_manager.cpp',
    'camera_sensor.cpp',
    'controls.cpp',
    'device_enumerator.cpp',
    'device_enumerator_sysfs.cpp',
    'event_dispatcher.cpp',
    'event_dispatcher_poll.cpp',
    'event_notifier.cpp',
    'formats.cpp',
    'geometry.cpp',
    'ipa_interface.cpp',
    'ipa_manager.cpp',
    'ipa_module.cpp',
    'ipc_unixsocket.cpp',
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
    'v4l2_controls.cpp',
    'v4l2_device.cpp',
    'v4l2_subdevice.cpp',
    'v4l2_videodevice.cpp',
])

libcamera_headers = files([
    'include/camera_sensor.h',
    'include/device_enumerator.h',
    'include/device_enumerator_sysfs.h',
    'include/device_enumerator_udev.h',
    'include/event_dispatcher_poll.h',
    'include/formats.h',
    'include/ipa_manager.h',
    'include/ipa_module.h',
    'include/ipc_unixsocket.h',
    'include/log.h',
    'include/media_device.h',
    'include/media_object.h',
    'include/pipeline_handler.h',
    'include/utils.h',
    'include/v4l2_device.h',
    'include/v4l2_subdevice.h',
    'include/v4l2_videodevice.h',
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

gen_controls = files('gen-controls.awk')

control_types_cpp = custom_target('control_types_cpp',
                                  input : files('controls.cpp'),
                                  output : 'control_types.cpp',
                                  depend_files : gen_controls,
                                  command : [gen_controls, '@INPUT@', '--code', '@OUTPUT@'])

libcamera_sources += control_types_cpp

gen_version = join_paths(meson.source_root(), 'utils', 'gen-version.sh')

version_cpp = vcs_tag(command : [gen_version, meson.build_root()],
                      input : 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

libcamera_sources += version_cpp

libcamera_deps = [
    cc.find_library('dl'),
    libudev,
]

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           include_directories : includes,
                           dependencies : libcamera_deps)

libcamera_dep = declare_dependency(sources : [libcamera_api, libcamera_h],
                                   include_directories : libcamera_includes,
                                   link_with : libcamera)
