/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * NV_2_planes_VU_f.glsl - Fragment shader code for NV21, NV61 and NV42 formats
 */

#ifdef GL_ES
precision mediump float;
#endif

varying vec2 textureOut;
uniform sampler2D tex_y;
uniform sampler2D tex_u;

void main(void)
{
	vec3 yuv;
	vec3 rgb;
	mat3 yuv2rgb_bt601_mat = mat3(
		vec3(1.164,  1.164, 1.164),
		vec3(0.000, -0.392, 2.017),
		vec3(1.596, -0.813, 0.000)
	);

	yuv.x = texture2D(tex_y, textureOut).r - 0.063;
	yuv.y = texture2D(tex_u, textureOut).g - 0.500;
	yuv.z = texture2D(tex_u, textureOut).r - 0.500;

	rgb = yuv2rgb_bt601_mat * yuv;
	gl_FragColor = vec4(rgb, 1.0);
}
