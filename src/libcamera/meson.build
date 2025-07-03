# SPDX-License-Identifier: CC0-1.0

libcamera_public_sources = files([
    'camera.cpp',
    'camera_manager.cpp',
    'color_space.cpp',
    'controls.cpp',
    'fence.cpp',
    'framebuffer.cpp',
    'framebuffer_allocator.cpp',
    'geometry.cpp',
    'orientation.cpp',
    'pixel_format.cpp',
    'request.cpp',
    'stream.cpp',
    'transform.cpp',
])

libcamera_internal_sources = files([
    'bayer_format.cpp',
    'byte_stream_buffer.cpp',
    'camera_controls.cpp',
    'camera_lens.cpp',
    'clock_recovery.cpp',
    'control_serializer.cpp',
    'control_validator.cpp',
    'converter.cpp',
    'debug_controls.cpp',
    'delayed_controls.cpp',
    'device_enumerator.cpp',
    'device_enumerator_sysfs.cpp',
    'dma_buf_allocator.cpp',
    'formats.cpp',
    'ipa_controls.cpp',
    'ipa_data_serializer.cpp',
    'ipa_interface.cpp',
    'ipa_manager.cpp',
    'ipa_module.cpp',
    'ipa_proxy.cpp',
    'ipc_pipe.cpp',
    'ipc_pipe_unixsocket.cpp',
    'ipc_unixsocket.cpp',
    'mapped_framebuffer.cpp',
    'matrix.cpp',
    'media_device.cpp',
    'media_object.cpp',
    'media_pipeline.cpp',
    'pipeline_handler.cpp',
    'process.cpp',
    'pub_key.cpp',
    'shared_mem_object.cpp',
    'source_paths.cpp',
    'sysfs.cpp',
    'v4l2_device.cpp',
    'v4l2_pixelformat.cpp',
    'v4l2_subdevice.cpp',
    'v4l2_videodevice.cpp',
    'vector.cpp',
    'yaml_parser.cpp',
])

includes = [
    libcamera_includes,
]

libcamera_deps = []

libatomic = cc.find_library('atomic', required : false)
libthreads = dependency('threads')

subdir('base')
subdir('converter')
subdir('ipa')
subdir('pipeline')
subdir('proxy')
subdir('sensor')
subdir('software_isp')

null_dep = dependency('', required : false)

# TODO: Use dependency('dl') when updating to meson 0.62.0 or newer.
libdl = null_dep
if not cc.has_function('dlopen')
    libdl = cc.find_library('dl')
endif
libudev = dependency('libudev', required : get_option('udev'))
libyaml = dependency('yaml-0.1', default_options : [
    'default_library=static',
    'werror=false',
])

# Use one of gnutls or libcrypto (provided by OpenSSL), trying gnutls first.
libcrypto = dependency('gnutls', required : false)
if libcrypto.found()
    config_h.set('HAVE_GNUTLS', 1)
else
    libcrypto = dependency('libcrypto', required : false)
    if libcrypto.found()
        config_h.set('HAVE_CRYPTO', 1)
    endif
endif

if not libcrypto.found()
    warning('Neither gnutls nor libcrypto found, all IPA modules will be isolated')
    summary({'IPA modules signed with': 'None (modules will run isolated)'},
            section : 'Configuration')
else
    summary({'IPA modules signed with' : libcrypto.name()}, section : 'Configuration')
endif

if liblttng.found()
    tracing_enabled = true
    config_h.set('HAVE_TRACING', 1)
    libcamera_internal_sources += files(['tracepoints.cpp'])
else
    tracing_enabled = false
endif

if libudev.found()
    config_h.set('HAVE_LIBUDEV', 1)
    libcamera_internal_sources += files([
        'device_enumerator_udev.cpp',
    ])
endif

control_sources = []

controls_mode_files = {
    'controls': [
        controls_files,
        'control_ids.cpp',
    ],
    'properties': [
        properties_files,
        'property_ids.cpp',
    ],
}

foreach mode, inout_files : controls_mode_files
    input_files = inout_files[0]
    output_file = inout_files[1]

    template_file = files('control_ids.cpp.in')
    ranges_file = files('control_ranges.yaml')

    control_sources += custom_target(mode + '_ids_cpp',
                                     input : input_files,
                                     output : output_file,
                                     command : [gen_controls, '-o', '@OUTPUT@',
                                                '--mode', mode, '-t', template_file,
                                                '-r', ranges_file, '@INPUT@'],
                                     depend_files : [py_mod_controls],
                                     env : py_build_env)
endforeach

libcamera_public_sources += control_sources

gen_version = meson.project_source_root() / 'utils' / 'gen-version.sh'

# Use vcs_tag() and not configure_file() or run_command(), to ensure that the
# version gets updated with every ninja build and not just at meson setup time.
version_cpp = vcs_tag(command : [gen_version, meson.project_build_root(), meson.project_source_root()],
                      input : 'version.cpp.in',
                      output : 'version.cpp',
                      fallback : meson.project_version())

libcamera_public_sources += version_cpp

if ipa_sign_module
    ipa_pub_key_cpp = custom_target('ipa_pub_key_cpp',
                                    input : [ipa_priv_key, 'ipa_pub_key.cpp.in'],
                                    output : 'ipa_pub_key.cpp',
                                    command : [gen_ipa_pub_key, '@INPUT@', '@OUTPUT@'])

    libcamera_internal_sources += ipa_pub_key_cpp
endif

libcamera_deps += [
    libatomic,
    libcamera_base,
    libcamera_base_private,
    libcrypto,
    libdl,
    liblttng,
    libudev,
    libyaml,
]

# We add '/' to the build_rpath as a 'safe' path to act as a boolean flag.
# The build_rpath is stripped at install time by meson, so we determine at
# runtime if the library is running from an installed location by checking
# for the presence or abscence of the dynamic tag.

libcamera = shared_library('libcamera',
                           [
                               libcamera_public_headers,
                               libcamera_public_sources,
                               libcamera_ipa_headers,
                               libcamera_internal_headers,
                               libcamera_internal_sources,
                           ],
                           version : libcamera_version,
                           soversion : libcamera_soversion,
                           name_prefix : '',
                           install : true,
                           include_directories : includes,
                           build_rpath : '/',
                           dependencies : libcamera_deps)

libcamera_public = declare_dependency(sources : [
                                          libcamera_public_headers,
                                      ],
                                      include_directories : libcamera_includes,
                                      dependencies : libcamera_base,
                                      link_with : libcamera)

# Internal dependency for components and plugins which can use private APIs
libcamera_private = declare_dependency(sources : [
                                           libcamera_ipa_headers,
                                       ],
                                       dependencies : [
                                           libcamera_public,
                                           libcamera_base_private,
                                       ])

pkg_mod = import('pkgconfig')
pkg_mod.generate(libcamera,
                 libraries : libcamera_base_lib,
                 description : 'Complex Camera Support Library',
                 subdirs : 'libcamera')

meson.override_dependency('libcamera', libcamera_public)

subdir('proxy/worker')
