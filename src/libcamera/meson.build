# SPDX-License-Identifier: CC0-1.0

libcamera_sources = files([
    'bayer_format.cpp',
    'buffer.cpp',
    'byte_stream_buffer.cpp',
    'camera.cpp',
    'camera_controls.cpp',
    'camera_manager.cpp',
    'camera_sensor.cpp',
    'camera_sensor_properties.cpp',
    'controls.cpp',
    'control_serializer.cpp',
    'control_validator.cpp',
    'delayed_controls.cpp',
    'device_enumerator.cpp',
    'device_enumerator_sysfs.cpp',
    'event_notifier.cpp',
    'file_descriptor.cpp',
    'formats.cpp',
    'framebuffer_allocator.cpp',
    'geometry.cpp',
    'ipa_controls.cpp',
    'ipa_data_serializer.cpp',
    'ipa_interface.cpp',
    'ipa_manager.cpp',
    'ipa_module.cpp',
    'ipa_proxy.cpp',
    'ipc_pipe.cpp',
    'ipc_pipe_unixsocket.cpp',
    'ipc_unixsocket.cpp',
    'media_device.cpp',
    'media_object.cpp',
    'pipeline_handler.cpp',
    'pixel_format.cpp',
    'process.cpp',
    'pub_key.cpp',
    'request.cpp',
    'source_paths.cpp',
    'stream.cpp',
    'sysfs.cpp',
    'transform.cpp',
    'v4l2_device.cpp',
    'v4l2_pixelformat.cpp',
    'v4l2_subdevice.cpp',
    'v4l2_videodevice.cpp',
])

libcamera_sources += libcamera_public_headers
libcamera_sources += libcamera_generated_ipa_headers
libcamera_sources += libcamera_tracepoint_header

includes = [
    libcamera_includes,
]

subdir('base')
subdir('ipa')
subdir('pipeline')
subdir('proxy')

libatomic = cc.find_library('atomic', required : false)
libdl = cc.find_library('dl')
libgnutls = cc.find_library('gnutls', required : true)
libudev = dependency('libudev', required : false)

if libgnutls.found()
    config_h.set('HAVE_GNUTLS', 1)
endif

if liblttng.found()
    config_h.set('HAVE_TRACING', 1)
    libcamera_sources += files(['tracepoints.cpp'])
endif

if libudev.found()
    config_h.set('HAVE_LIBUDEV', 1)
    libcamera_sources += files([
        'device_enumerator_udev.cpp',
    ])
endif

control_sources = []

foreach source : control_source_files
    input_files = files(source +'.yaml', source + '.cpp.in')
    control_sources += custom_target(source + '_cpp',
                                     input : input_files,
                                     output : source + '.cpp',
                                     command : [gen_controls, '-o', '@OUTPUT@', '@INPUT@'])
endforeach

libcamera_sources += control_sources

gen_version = meson.source_root() / 'utils' / 'gen-version.sh'

version_cpp = vcs_tag(command : [gen_version, meson.build_root()],
                      input : 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

libcamera_sources += version_cpp

if ipa_sign_module
    ipa_pub_key_cpp = custom_target('ipa_pub_key_cpp',
                                    input : [ipa_priv_key, 'ipa_pub_key.cpp.in'],
                                    output : 'ipa_pub_key.cpp',
                                    command : [gen_ipa_pub_key, '@INPUT@', '@OUTPUT@'])

    libcamera_sources += ipa_pub_key_cpp
endif

libcamera_deps = [
    libatomic,
    libcamera_base,
    libdl,
    libgnutls,
    liblttng,
    libudev,
]

# We add '/' to the build_rpath as a 'safe' path to act as a boolean flag.
# The build_rpath is stripped at install time by meson, so we determine at
# runtime if the library is running from an installed location by checking
# for the presence or abscence of the dynamic tag.

libcamera = shared_library('libcamera',
                           libcamera_sources,
                           name_prefix : '',
                           install : true,
                           include_directories : includes,
                           build_rpath : '/',
                           dependencies : libcamera_deps)

# TODO Drop libcamera_generated_ipa_headers from libcamera_dep as libcamera_dep
# is supposed to model the dependencies required for usage of the public API,
# not the internal API. It is needed as as some external components such as the
# unit tests make use of the generated headers and this creates a race in the
# build.

libcamera_dep = declare_dependency(sources : [
                                       libcamera_ipa_headers,
                                       libcamera_public_headers,
                                       libcamera_generated_ipa_headers,
                                   ],
                                   include_directories : libcamera_includes,
                                   dependencies: libcamera_base,
                                   link_with : libcamera)

pkg_mod = import('pkgconfig')
pkg_mod.generate(libcamera,
                 libraries : libcamera_base_lib,
                 version : '1.0',
                 description : 'Complex Camera Support Library',
                 subdirs : 'libcamera')

subdir('proxy/worker')
