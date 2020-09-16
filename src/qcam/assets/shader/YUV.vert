/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * YUV.vert - Vertex shader for YUV to RGB conversion
 */

attribute vec4 vertexIn;
attribute vec2 textureIn;
varying vec2 textureOut;

void main(void)
{
	gl_Position = vertexIn;
	textureOut = textureIn;
}
