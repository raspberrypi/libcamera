/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * identity.vert - Identity vertex shader for pixel format conversion
 */

attribute vec4 vertexIn;
attribute vec2 textureIn;
varying vec2 textureOut;

uniform float stride_factor;

void main(void)
{
	gl_Position = vertexIn;
	textureOut = vec2(textureIn.x * stride_factor, textureIn.y);
}
