# SPDX-License-Identifier: CC0-1.0

libcamera_include_dir = 'libcamera' / 'libcamera'

libcamera_public_headers = files([
    'camera.h',
    'camera_manager.h',
    'color_space.h',
    'controls.h',
    'fence.h',
    'framebuffer.h',
    'framebuffer_allocator.h',
    'geometry.h',
    'logging.h',
    'pixel_format.h',
    'request.h',
    'stream.h',
    'transform.h',
])

subdir('base')
subdir('internal')
subdir('ipa')

install_headers(libcamera_public_headers,
                subdir : libcamera_include_dir)

#
# Generate headers from templates.
#

libcamera_headers_install_dir = get_option('includedir') / libcamera_include_dir

# control_ids.h and property_ids.h
control_source_files = [
    'control_ids',
    'property_ids',
]

control_headers = []

foreach header : control_source_files
    input_files = files('../../src/libcamera/' + header +'.yaml', header + '.h.in')
    control_headers += custom_target(header + '_h',
                                     input : input_files,
                                     output : header + '.h',
                                     command : [gen_controls, '-o', '@OUTPUT@', '@INPUT@'],
                                     install : true,
                                     install_dir : libcamera_headers_install_dir)
endforeach

libcamera_public_headers += control_headers

# formats.h
formats_h = custom_target('formats_h',
                          input : files(
                              '../../src/libcamera/formats.yaml',
                              'formats.h.in',
                              '../linux/drm_fourcc.h'
                          ),
                          output : 'formats.h',
                          command : [gen_formats, '-o', '@OUTPUT@', '@INPUT@'],
                          install : true,
                          install_dir : libcamera_headers_install_dir)
libcamera_public_headers += formats_h

# libcamera.h
libcamera_h = custom_target('gen-header',
                            input : 'meson.build',
                            output : 'libcamera.h',
                            command : [gen_header, meson.current_source_dir(), '@OUTPUT@'],
                            install : true,
                            install_dir : libcamera_headers_install_dir)

libcamera_public_headers += libcamera_h

# version.h
version = libcamera_version.split('.')
libcamera_version_config = configuration_data()
libcamera_version_config.set('LIBCAMERA_VERSION_MAJOR', version[0])
libcamera_version_config.set('LIBCAMERA_VERSION_MINOR', version[1])
libcamera_version_config.set('LIBCAMERA_VERSION_PATCH', version[2])

configure_file(input : 'version.h.in',
               output : 'version.h',
               configuration : libcamera_version_config,
               install_dir : libcamera_headers_install_dir)
