/*
From http://jgt.akpeters.com/papers/McGuire08/

Efficient, High-Quality Bayer Demosaic Filtering on GPUs

Morgan McGuire

This paper appears in issue Volume 13, Number 4.
---------------------------------------------------------
Copyright (c) 2008, Morgan McGuire. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//Pixel Shader

/** Monochrome RGBA or GL_LUMINANCE Bayer encoded texture.*/
uniform sampler2D       source;
varying vec4            center;
varying vec4            yCoord;
varying vec4            xCoord;

void main(void) {
    #define fetch(x, y) texture2D(source, vec2(x, y)).r

    float C = texture2D(source, center.xy).r; // ( 0, 0)
    const vec4 kC = vec4( 4.0,  6.0,  5.0,  5.0) / 8.0;

    // Determine which of four types of pixels we are on.
    vec2 alternate = mod(floor(center.zw), 2.0);

    vec4 Dvec = vec4(
        fetch(xCoord[1], yCoord[1]),  // (-1,-1)
        fetch(xCoord[1], yCoord[2]),  // (-1, 1)
        fetch(xCoord[2], yCoord[1]),  // ( 1,-1)
        fetch(xCoord[2], yCoord[2])); // ( 1, 1)

    vec4 PATTERN = (kC.xyz * C).xyzz;

    // Can also be a dot product with (1,1,1,1) on hardware where that is
    // specially optimized.
    // Equivalent to: D = Dvec[0] + Dvec[1] + Dvec[2] + Dvec[3];
    Dvec.xy += Dvec.zw;
    Dvec.x  += Dvec.y;

    vec4 value = vec4(
        fetch(center.x, yCoord[0]),   // ( 0,-2)
        fetch(center.x, yCoord[1]),   // ( 0,-1)
        fetch(xCoord[0], center.y),   // (-1, 0)
        fetch(xCoord[1], center.y));  // (-2, 0)

    vec4 temp = vec4(
        fetch(center.x, yCoord[3]),   // ( 0, 2)
        fetch(center.x, yCoord[2]),   // ( 0, 1)
        fetch(xCoord[3], center.y),   // ( 2, 0)
        fetch(xCoord[2], center.y));  // ( 1, 0)

    // Even the simplest compilers should be able to constant-fold these to
    // avoid the division.
    // Note that on scalar processors these constants force computation of some
    // identical products twice.
    const vec4 kA = vec4(-1.0, -1.5,  0.5, -1.0) / 8.0;
    const vec4 kB = vec4( 2.0,  0.0,  0.0,  4.0) / 8.0;
    const vec4 kD = vec4( 0.0,  2.0, -1.0, -1.0) / 8.0;

    // Conserve constant registers and take advantage of free swizzle on load
    #define kE (kA.xywz)
    #define kF (kB.xywz)

    value += temp;

    // There are five filter patterns (identity, cross, checker,
    // theta, phi).  Precompute the terms from all of them and then
    // use swizzles to assign to color channels.
    //
    // Channel   Matches
    //   x       cross   (e.g., EE G)
    //   y       checker (e.g., EE B)
    //   z       theta   (e.g., EO R)
    //   w       phi     (e.g., EO R)
    #define A (value[0])
    #define B (value[1])
    #define D (Dvec.x)
    #define E (value[2])
    #define F (value[3])

    // Avoid zero elements. On a scalar processor this saves two MADDs
    // and it has no effect on a vector processor.
    PATTERN.yzw += (kD.yz * D).xyy;

    PATTERN += (kA.xyz * A).xyzx + (kE.xyw * E).xyxz;
    PATTERN.xw  += kB.xw * B;
    PATTERN.xz  += kF.xz * F;

    gl_FragColor.rgb = (alternate.y == 0.0) ?
        ((alternate.x == 0.0) ?
            vec3(C, PATTERN.xy) :
            vec3(PATTERN.z, C, PATTERN.w)) :
        ((alternate.x == 0.0) ?
            vec3(PATTERN.w, C, PATTERN.z) :
            vec3(PATTERN.yx, C));
}
