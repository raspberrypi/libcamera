/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * YUV_3_planes_UV.frag - Fragment shader code for YUV420 format
 */

#ifdef GL_ES
precision mediump float;
#endif

varying vec2 textureOut;
uniform sampler2D tex_y;
uniform sampler2D tex_u;
uniform sampler2D tex_v;

const mat3 yuv2rgb_matrix = mat3(
	YUV2RGB_MATRIX
);

const vec3 yuv2rgb_offset = vec3(
	YUV2RGB_Y_OFFSET / 255.0, 128.0 / 255.0, 128.0 / 255.0
);

void main(void)
{
	vec3 yuv;

	yuv.x = texture2D(tex_y, textureOut).r;
	yuv.y = texture2D(tex_u, textureOut).r;
	yuv.z = texture2D(tex_v, textureOut).r;

	vec3 rgb = yuv2rgb_matrix * (yuv - yuv2rgb_offset);

	gl_FragColor = vec4(rgb, 1.0);
}
