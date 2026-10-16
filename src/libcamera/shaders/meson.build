# SPDX-License-Identifier: CC0-1.0

# List of shader files to convert to header hex
# for the purposes of inclusion in OpenGL debayering
shader_files = files([
    'bayer_1x_packed.frag',
    'bayer_unpacked.frag',
    'bayer_unpacked.vert',
    'identity.vert',
])

# Generate header from shaders
libcamera_shader_header = custom_target(
    'gen-shader-headers',
    input : [shader_files],
    output : 'glsl_shaders.h',
    command : [gen_shader_header, '-o', '@OUTPUT@', '@INPUT@'],
)

libcamera_internal_headers += libcamera_shader_header
