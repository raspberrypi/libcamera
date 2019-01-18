libcamera_api = files([
    'buffer.h',
    'camera.h',
    'camera_manager.h',
    'control_ids.h',
    'controls.h',
    'event_dispatcher.h',
    'event_notifier.h',
    'geometry.h',
    'ipa/ipa_interface.h',
    'ipa/ipa_module_info.h',
    'object.h',
    'request.h',
    'signal.h',
    'stream.h',
    'timer.h',
])

gen_version = join_paths(meson.source_root(), 'utils', 'gen-version.sh')

version_h = vcs_tag(command : [gen_version, meson.current_source_dir()],
                    input : 'version.h.in',
                    output : 'version.h',
                    fallback : 'v0.0')

gen_header = files('gen-header.sh')

libcamera_h = custom_target('gen-header',
                            input : 'meson.build',
                            output : 'libcamera.h',
                            command : [gen_header, meson.current_source_dir(), '@OUTPUT@'],
                            install : true,
                            install_dir : 'include/libcamera')

install_headers(libcamera_api,
                subdir : 'libcamera')
