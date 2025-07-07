# SPDX-License-Identifier: CC0-1.0

subdir('tracepoints')

libcamera_internal_headers = files([
    'bayer_format.h',
    'byte_stream_buffer.h',
    'camera.h',
    'camera_controls.h',
    'camera_lens.h',
    'camera_manager.h',
    'camera_sensor.h',
    'camera_sensor_properties.h',
    'clock_recovery.h',
    'control_serializer.h',
    'control_validator.h',
    'converter.h',
    'debug_controls.h',
    'delayed_controls.h',
    'device_enumerator.h',
    'device_enumerator_sysfs.h',
    'device_enumerator_udev.h',
    'dma_buf_allocator.h',
    'formats.h',
    'framebuffer.h',
    'ipa_data_serializer.h',
    'ipa_manager.h',
    'ipa_module.h',
    'ipa_proxy.h',
    'ipc_pipe.h',
    'ipc_unixsocket.h',
    'mapped_framebuffer.h',
    'matrix.h',
    'media_device.h',
    'media_object.h',
    'media_pipeline.h',
    'pipeline_handler.h',
    'process.h',
    'pub_key.h',
    'request.h',
    'shared_mem_object.h',
    'source_paths.h',
    'sysfs.h',
    'v4l2_device.h',
    'v4l2_pixelformat.h',
    'v4l2_subdevice.h',
    'v4l2_videodevice.h',
    'vector.h',
    'yaml_parser.h',
])

tracepoints_h = custom_target(
    'tp_header',
    input : ['tracepoints.h.in', tracepoint_files],
    output : 'tracepoints.h',
    command : [gen_tracepoints, include_build_dir, '@OUTPUT@', '@INPUT@'],
)

libcamera_internal_headers += tracepoints_h

subdir('converter')
subdir('software_isp')
