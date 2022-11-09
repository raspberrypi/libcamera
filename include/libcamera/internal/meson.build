# SPDX-License-Identifier: CC0-1.0

subdir('tracepoints')

libcamera_tracepoint_header = custom_target(
    'tp_header',
    input: ['tracepoints.h.in', tracepoint_files],
    output: 'tracepoints.h',
    command: [gen_tracepoints_header, include_build_dir, '@OUTPUT@', '@INPUT@'],
)

libcamera_internal_headers = files([
    'bayer_format.h',
    'byte_stream_buffer.h',
    'camera.h',
    'camera_controls.h',
    'camera_lens.h',
    'camera_sensor.h',
    'camera_sensor_properties.h',
    'control_serializer.h',
    'control_validator.h',
    'delayed_controls.h',
    'device_enumerator.h',
    'device_enumerator_sysfs.h',
    'device_enumerator_udev.h',
    'formats.h',
    'framebuffer.h',
    'ipa_manager.h',
    'ipa_module.h',
    'ipa_proxy.h',
    'ipc_unixsocket.h',
    'mapped_framebuffer.h',
    'media_device.h',
    'media_object.h',
    'pipeline_handler.h',
    'process.h',
    'pub_key.h',
    'request.h',
    'source_paths.h',
    'sysfs.h',
    'v4l2_device.h',
    'v4l2_pixelformat.h',
    'v4l2_subdevice.h',
    'v4l2_videodevice.h',
    'yaml_parser.h',
])
