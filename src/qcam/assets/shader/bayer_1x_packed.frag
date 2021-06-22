/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Based on the code from http://jgt.akpeters.com/papers/McGuire08/
 *
 * Efficient, High-Quality Bayer Demosaic Filtering on GPUs
 *
 * Morgan McGuire
 *
 * This paper appears in issue Volume 13, Number 4.
 * ---------------------------------------------------------
 * Copyright (c) 2008, Morgan McGuire. All rights reserved.
 *
 *
 * Modified by Linaro Ltd for 10/12-bit packed vs 8-bit raw Bayer format,
 * and for simpler demosaic algorithm.
 * Copyright (C) 2020, Linaro
 *
 * bayer_1x_packed.frag - Fragment shader code for raw Bayer 10-bit and 12-bit
 * packed formats
 */

#ifdef GL_ES
precision mediump float;
#endif

/*
 * These constants are used to select the bytes containing the HS part of
 * the pixel value:
 * BPP - bytes per pixel,
 * THRESHOLD_L = fract(BPP) * 0.5 + 0.02
 * THRESHOLD_H = 1.0 - fract(BPP) * 1.5 + 0.02
 * Let X is the x coordinate in the texture measured in bytes (so that the
 * range is from 0 to (stride_-1)) aligned on the nearest pixel.
 * E.g. for RAW10P:
 * -------------+-------------------+-------------------+--
 *  pixel No    |  0   1    2   3   |  4   5    6   7   | ...
 * -------------+-------------------+-------------------+--
 *  byte offset | 0   1   2   3   4 | 5   6   7   8   9 | ...
 * -------------+-------------------+-------------------+--
 *      X       | 0.0 1.25 2.5 3.75 | 5.0 6.25 7.5 8.75 | ...
 * -------------+-------------------+-------------------+--
 * If fract(X) < THRESHOLD_L then the previous byte contains the LS
 * bits of the pixel values and needs to be skipped.
 * If fract(X) > THRESHOLD_H then the next byte contains the LS bits
 * of the pixel values and needs to be skipped.
 */
#if defined(RAW10P)
#define BPP		1.25
#define THRESHOLD_L	0.14
#define THRESHOLD_H	0.64
#elif defined(RAW12P)
#define BPP		1.5
#define THRESHOLD_L	0.27
#define THRESHOLD_H	0.27
#else
#error Invalid raw format
#endif


varying vec2 textureOut;

/* the texture size in pixels */
uniform vec2 tex_size;
uniform vec2 tex_step;
uniform vec2 tex_bayer_first_red;

uniform sampler2D tex_y;

void main(void)
{
	vec3 rgb;

	/*
	 * center_bytes holds the coordinates of the MS byte of the pixel
	 * being sampled on the [0, stride-1/height-1] range.
	 * center_pixel holds the coordinates of the pixel being sampled
	 * on the [0, width/height-1] range.
	 */
	vec2 center_bytes;
	vec2 center_pixel;

	/*
	 * x- and y-positions of the adjacent pixels on the [0, 1] range.
	 */
	vec2 xcoords;
	vec2 ycoords;

	/*
	 * The coordinates passed to the shader in textureOut may point
	 * to a place in between the pixels if the texture format doesn't
	 * match the image format. In particular, MIPI packed raw Bayer
	 * formats don't have a matching texture format.
	 * In this case align the coordinates to the left nearest pixel
	 * by hand.
	 */
	center_pixel = floor(textureOut * tex_size);
	center_bytes.y = center_pixel.y;

	/*
	 * Add a small number (a few mantissa's LSBs) to avoid float
	 * representation issues. Maybe paranoic.
	 */
	center_bytes.x = BPP * center_pixel.x + 0.02;

	float fract_x = fract(center_bytes.x);

	/*
	 * The below floor() call ensures that center_bytes.x points
	 * at one of the bytes representing the 8 higher bits of
	 * the pixel value, not at the byte containing the LS bits
	 * of the group of the pixels.
	 */
	center_bytes.x = floor(center_bytes.x);
	center_bytes *= tex_step;

	xcoords = center_bytes.x + vec2(-tex_step.x, tex_step.x);
	ycoords = center_bytes.y + vec2(-tex_step.y, tex_step.y);

	/*
	 * If xcoords[0] points at the byte containing the LS bits
	 * of the previous group of the pixels, move xcoords[0] one
	 * byte back.
	 */
	xcoords[0] += (fract_x < THRESHOLD_L) ? -tex_step.x : 0.0;

	/*
	 * If xcoords[1] points at the byte containing the LS bits
	 * of the current group of the pixels, move xcoords[1] one
	 * byte forward.
	 */
	xcoords[1] += (fract_x > THRESHOLD_H) ? tex_step.x : 0.0;

	vec2 alternate = mod(center_pixel.xy + tex_bayer_first_red, 2.0);
	bool even_col = alternate.x < 1.0;
	bool even_row = alternate.y < 1.0;

	/*
	 * We need to sample the central pixel and the ones with offset
	 * of -1 to +1 pixel in both X and Y directions. Let's name these
	 * pixels as below, where C is the central pixel:
	 *
	 *   +----+----+----+----+
	 *   | \ x|    |    |    |
	 *   |y \ | -1 |  0 | +1 |
	 *   +----+----+----+----+
	 *   | +1 | D2 | A1 | D3 |
	 *   +----+----+----+----+
	 *   |  0 | B0 |  C | B1 |
	 *   +----+----+----+----+
	 *   | -1 | D0 | A0 | D1 |
	 *   +----+----+----+----+
	 *
	 * In the below equations (0,-1).r means "r component of the texel
	 * shifted by -tex_step.y from the center_bytes one" etc.
	 *
	 * In the even row / even column (EE) case the colour values are:
	 *   R = C = (0,0).r,
	 *   G = (A0 + A1 + B0 + B1) / 4.0 =
	 *       ( (0,-1).r + (0,1).r + (-1,0).r + (1,0).r ) / 4.0,
	 *   B = (D0 + D1 + D2 + D3) / 4.0 =
	 *       ( (-1,-1).r + (1,-1).r + (-1,1).r + (1,1).r ) / 4.0
	 *
	 * For even row / odd column (EO):
	 *   R = (B0 + B1) / 2.0 = ( (-1,0).r + (1,0).r ) / 2.0,
	 *   G = C = (0,0).r,
	 *   B = (A0 + A1) / 2.0 = ( (0,-1).r + (0,1).r ) / 2.0
	 *
	 * For odd row / even column (OE):
	 *   R = (A0 + A1) / 2.0 = ( (0,-1).r + (0,1).r ) / 2.0,
	 *   G = C = (0,0).r,
	 *   B = (B0 + B1) / 2.0 = ( (-1,0).r + (1,0).r ) / 2.0
	 *
	 * For odd row / odd column (OO):
	 *   R = (D0 + D1 + D2 + D3) / 4.0 =
	 *       ( (-1,-1).r + (1,-1).r + (-1,1).r + (1,1).r ) / 4.0,
	 *   G = (A0 + A1 + B0 + B1) / 4.0 =
	 *       ( (0,-1).r + (0,1).r + (-1,0).r + (1,0).r ) / 4.0,
	 *   B = C = (0,0).r
	 */

	/*
	 * Fetch the values and precalculate the terms:
	 *   patterns.x = (A0 + A1) / 2.0
	 *   patterns.y = (B0 + B1) / 2.0
	 *   patterns.z = (A0 + A1 + B0 + B1) / 4.0
	 *   patterns.w = (D0 + D1 + D2 + D3) / 4.0
	 */
	#define fetch(x, y) texture2D(tex_y, vec2(x, y)).r

	float C = texture2D(tex_y, center_bytes).r;
	vec4 patterns = vec4(
		fetch(center_bytes.x, ycoords[0]),	/* A0: (0,-1) */
		fetch(xcoords[0], center_bytes.y),	/* B0: (-1,0) */
		fetch(xcoords[0], ycoords[0]),		/* D0: (-1,-1) */
		fetch(xcoords[1], ycoords[0]));		/* D1: (1,-1) */
	vec4 temp = vec4(
		fetch(center_bytes.x, ycoords[1]),	/* A1: (0,1) */
		fetch(xcoords[1], center_bytes.y),	/* B1: (1,0) */
		fetch(xcoords[1], ycoords[1]),		/* D3: (1,1) */
		fetch(xcoords[0], ycoords[1]));		/* D2: (-1,1) */
	patterns = (patterns + temp) * 0.5;
		/* .x = (A0 + A1) / 2.0, .y = (B0 + B1) / 2.0 */
		/* .z = (D0 + D3) / 2.0, .w = (D1 + D2) / 2.0 */
	patterns.w = (patterns.z + patterns.w) * 0.5;
	patterns.z = (patterns.x + patterns.y) * 0.5;

	rgb = even_col ?
		(even_row ?
			vec3(C, patterns.zw) :
			vec3(patterns.x, C, patterns.y)) :
		(even_row ?
			vec3(patterns.y, C, patterns.x) :
			vec3(patterns.wz, C));

	gl_FragColor = vec4(rgb, 1.0);
}
