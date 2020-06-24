# SPDX-License-Identifier: CC0-1.0

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
    'file.cpp',
    'file_descriptor.cpp',
    'formats.cpp',
    'framebuffer_allocator.cpp',
    'geometry.cpp',
    'ipa_context_wrapper.cpp',
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
    'pixel_format.cpp',
    'process.cpp',
    'pub_key.cpp',
    'request.cpp',
    'semaphore.cpp',
    'signal.cpp',
    'stream.cpp',
    'sysfs.cpp',
    'thread.cpp',
    'timer.cpp',
    'utils.cpp',
    'v4l2_controls.cpp',
    'v4l2_device.cpp',
    'v4l2_pixelformat.cpp',
    'v4l2_subdevice.cpp',
    'v4l2_videodevice.cpp',
])

libcamera_sources += libcamera_public_headers

includes = [
    libcamera_includes,
]

subdir('pipeline')
subdir('proxy')

libatomic = cc.find_library('atomic', required : false)
libdl = cc.find_library('dl')
libgnutls = cc.find_library('gnutls', required : true)
libudev = dependency('libudev', required : false)

if libgnutls.found()
    config_h.set('HAVE_GNUTLS', 1)
endif

if libudev.found()
    config_h.set('HAVE_LIBUDEV', 1)
    libcamera_sources += files([
        'device_enumerator_udev.cpp',
    ])
endif

gen_controls = files('gen-controls.py')

control_sources = []

foreach source : control_source_files
    input_files = files(source +'.yaml', source + '.cpp.in')
    control_sources += custom_target(source + '_cpp',
                                     input : input_files,
                                     output : source + '.cpp',
                                     depend_files : gen_controls,
                                     command : [gen_controls, '-o', '@OUTPUT@', '@INPUT@'])
endforeach

libcamera_sources += control_sources

gen_version = join_paths(meson.source_root(), 'utils', 'gen-version.sh')

version_cpp = vcs_tag(command : [gen_version, meson.build_root()],
                      input : 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

libcamera_sources += version_cpp

if ipa_sign_module
    gen_ipa_pub_key = files('gen-ipa-pub-key.py')
    ipa_pub_key_cpp = custom_target('ipa_pub_key_cpp',
                                    input : [ ipa_priv_key, 'ipa_pub_key.cpp.in' ],
                                    output : 'ipa_pub_key.cpp',
                                    command : [ gen_ipa_pub_key, '@INPUT@', '@OUTPUT@' ])

    libcamera_sources += ipa_pub_key_cpp
endif

libcamera_deps = [
    libatomic,
    libdl,
    libgnutls,
    libudev,
    dependency('threads'),
]

libcamera_link_with = []

if get_option('android')
    libcamera_sources += android_hal_sources
    includes += android_includes
    libcamera_link_with += android_camera_metadata

    libcamera_deps += android_deps
endif

# We add '/' to the build_rpath as a 'safe' path to act as a boolean flag.
# The build_rpath is stripped at install time by meson, so we determine at
# runtime if the library is running from an installed location by checking
# for the presence or abscence of the dynamic tag.

libcamera = shared_library('camera',
                           libcamera_sources,
                           install : true,
                           link_with : libcamera_link_with,
                           include_directories : includes,
                           build_rpath : '/',
                           dependencies : libcamera_deps)

libcamera_dep = declare_dependency(sources : [
                                       libcamera_ipa_headers,
                                       libcamera_public_headers,
                                   ],
                                   include_directories : libcamera_includes,
                                   link_with : libcamera)

subdir('proxy/worker')
