/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 *
 * RGB.frag - Fragment shader code for RGB formats
 */

#ifdef GL_ES
precision mediump float;
#endif

varying vec2 textureOut;
uniform sampler2D tex_y;

void main(void)
{
	vec3 rgb;

	rgb = texture2D(tex_y, textureOut).RGB_PATTERN;

	gl_FragColor = vec4(rgb, 1.0);
}
