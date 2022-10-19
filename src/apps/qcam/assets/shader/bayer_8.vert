/* SPDX-License-Identifier: BSD-2-Clause */
/*
From http://jgt.akpeters.com/papers/McGuire08/

Efficient, High-Quality Bayer Demosaic Filtering on GPUs

Morgan McGuire

This paper appears in issue Volume 13, Number 4.
---------------------------------------------------------
Copyright (c) 2008, Morgan McGuire. All rights reserved.

Modified by Linaro Ltd to integrate it into libcamera.
Copyright (C) 2021, Linaro
*/

//Vertex Shader

attribute vec4 vertexIn;
attribute vec2 textureIn;

uniform vec2 tex_size;	/* The texture size in pixels */
uniform vec2 tex_step;

/** Pixel position of the first red pixel in the */
/**  Bayer pattern.  [{0,1}, {0, 1}]*/
uniform vec2            tex_bayer_first_red;

/** .xy = Pixel being sampled in the fragment shader on the range [0, 1]
    .zw = ...on the range [0, sourceSize], offset by firstRed */
varying vec4            center;

/** center.x + (-2/w, -1/w, 1/w, 2/w); These are the x-positions */
/** of the adjacent pixels.*/
varying vec4            xCoord;

/** center.y + (-2/h, -1/h, 1/h, 2/h); These are the y-positions */
/** of the adjacent pixels.*/
varying vec4            yCoord;

void main(void) {
    center.xy = textureIn;
    center.zw = textureIn * tex_size + tex_bayer_first_red;

    xCoord = center.x + vec4(-2.0 * tex_step.x,
                             -tex_step.x, tex_step.x, 2.0 * tex_step.x);
    yCoord = center.y + vec4(-2.0 * tex_step.y,
                              -tex_step.y, tex_step.y, 2.0 * tex_step.y);

    gl_Position = vertexIn;
}
