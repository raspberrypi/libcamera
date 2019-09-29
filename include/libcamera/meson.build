libcamera_api = files([
    'bound_method.h',
    'buffer.h',
    'camera.h',
    'camera_manager.h',
    'controls.h',
    'event_dispatcher.h',
    'event_notifier.h',
    'geometry.h',
    'logging.h',
    'object.h',
    'request.h',
    'signal.h',
    'stream.h',
    'timer.h',
])

include_dir = join_paths(libcamera_include_dir, 'libcamera')

install_headers(libcamera_api,
                subdir : include_dir)

gen_controls = files('../../src/libcamera/gen-controls.py')

control_ids_h = custom_target('control_ids_h',
                              input : files('../../src/libcamera/control_ids.yaml', 'control_ids.h.in'),
                              output : 'control_ids.h',
                              depend_files : gen_controls,
                              command : [gen_controls, '-o', '@OUTPUT@', '@INPUT@'],
                              install_dir : join_paths('include', include_dir))

libcamera_api += control_ids_h

gen_header = files('gen-header.sh')

libcamera_h = custom_target('gen-header',
                            input : 'meson.build',
                            output : 'libcamera.h',
                            command : [gen_header, meson.current_source_dir(), '@OUTPUT@'],
                            install : true,
                            install_dir : join_paths('include', include_dir))

version = libcamera_version.split('.')
libcamera_version_config = configuration_data()
libcamera_version_config.set('LIBCAMERA_VERSION_MAJOR', version[0])
libcamera_version_config.set('LIBCAMERA_VERSION_MINOR', version[1])
libcamera_version_config.set('LIBCAMERA_VERSION_PATCH', version[2])

configure_file(input : 'version.h.in',
               output : 'version.h',
               configuration : libcamera_version_config,
               install_dir : join_paths('include', include_dir))
