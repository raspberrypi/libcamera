libcamera_api = files([
    'buffer.h',
    'camera.h',
    'camera_manager.h',
    'event_dispatcher.h',
    'event_notifier.h',
    'geometry.h',
    'ipa/ipa_module_info.h',
    'object.h',
    'request.h',
    'signal.h',
    'stream.h',
    'timer.h',
])

gen_header = join_paths(meson.current_source_dir(), 'gen-header.sh')

libcamera_h = custom_target('gen-header',
                            input : 'meson.build',
                            output : 'libcamera.h',
                            command : [gen_header, meson.current_source_dir(), '@OUTPUT@'],
                            install : true,
                            install_dir : 'include/libcamera')

install_headers(libcamera_api,
                subdir : 'libcamera')
