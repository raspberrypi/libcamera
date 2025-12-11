/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * YUV_packed.frag - Fragment shader code for YUYV packed formats
 */

#ifdef GL_ES
precision mediump float;
#endif

varying vec2 textureOut;

uniform sampler2D tex_y;
uniform vec2 tex_step;

const mat3 yuv2rgb_matrix = mat3(
	YUV2RGB_MATRIX
);

const vec3 yuv2rgb_offset = vec3(
	YUV2RGB_Y_OFFSET / 255.0, 128.0 / 255.0, 128.0 / 255.0
);

void main(void)
{
	/*
	 * The sampler won't interpolate the texture correctly along the X axis,
	 * as each RGBA pixel effectively stores two pixels. We thus need to
	 * interpolate manually.
	 *
	 * In integer texture coordinates, the Y values are layed out in the
	 * texture memory as follows:
	 *
	 * ...| Y  U  Y  V | Y  U  Y  V | Y  U  Y  V |...
	 * ...| R  G  B  A | R  G  B  A | R  G  B  A |...
	 *      ^     ^      ^     ^      ^     ^
	 *      |     |      |     |      |     |
	 *     n-1  n-0.5    n   n+0.5   n+1  n+1.5
	 *
	 * For a texture location x in the interval [n, n+1[, sample the left
	 * and right pixels at n and n+1, and interpolate them with
	 *
	 * left.r * (1 - a) + left.b * a	if fract(x) < 0.5
	 * left.b * (1 - a) + right.r * a	if fract(x) >= 0.5
	 *
	 * with a = fract(x * 2) which can also be written
	 *
	 * a = fract(x) * 2			if fract(x) < 0.5
	 * a = fract(x) * 2 - 1			if fract(x) >= 0.5
	 */
	vec2 pos = textureOut;
	float f_x = fract(pos.x / tex_step.x);

	vec4 left = texture2D(tex_y, vec2(pos.x - f_x * tex_step.x, pos.y));
	vec4 right = texture2D(tex_y, vec2(pos.x + (1.0 - f_x) * tex_step.x , pos.y));

#if defined(YUV_PATTERN_UYVY)
	float y_left = mix(left.g, left.a, f_x * 2.0);
	float y_right = mix(left.a, right.g, f_x * 2.0 - 1.0);
	vec2 uv = mix(left.rb, right.rb, f_x);
#elif defined(YUV_PATTERN_VYUY)
	float y_left = mix(left.g, left.a, f_x * 2.0);
	float y_right = mix(left.a, right.g, f_x * 2.0 - 1.0);
	vec2 uv = mix(left.br, right.br, f_x);
#elif defined(YUV_PATTERN_YUYV)
	float y_left = mix(left.r, left.b, f_x * 2.0);
	float y_right = mix(left.b, right.r, f_x * 2.0 - 1.0);
	vec2 uv = mix(left.ga, right.ga, f_x);
#elif defined(YUV_PATTERN_YVYU)
	float y_left = mix(left.r, left.b, f_x * 2.0);
	float y_right = mix(left.b, right.r, f_x * 2.0 - 1.0);
	vec2 uv = mix(left.ag, right.ag, f_x);
#else
#error Invalid pattern
#endif

	float y = mix(y_left, y_right, step(0.5, f_x));

	vec3 rgb = yuv2rgb_matrix * (vec3(y, uv) - yuv2rgb_offset);

	gl_FragColor = vec4(rgb, 1.0);
}
