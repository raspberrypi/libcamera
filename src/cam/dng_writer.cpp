/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * dng_writer.cpp - DNG writer
 */

#include "dng_writer.h"

#include <algorithm>
#include <iostream>
#include <map>

#include <tiffio.h>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>

using namespace libcamera;

enum CFAPatternColour : uint8_t {
	CFAPatternRed = 0,
	CFAPatternGreen = 1,
	CFAPatternBlue = 2,
};

struct FormatInfo {
	uint8_t bitsPerSample;
	CFAPatternColour pattern[4];
	void (*packScanline)(void *output, const void *input,
			     unsigned int width);
	void (*thumbScanline)(const FormatInfo &info, void *output,
			      const void *input, unsigned int width,
			      unsigned int stride);
};

struct Matrix3d {
	Matrix3d()
	{
	}

	Matrix3d(float m0, float m1, float m2,
		 float m3, float m4, float m5,
		 float m6, float m7, float m8)
	{
		m[0] = m0, m[1] = m1, m[2] = m2;
		m[3] = m3, m[4] = m4, m[5] = m5;
		m[6] = m6, m[7] = m7, m[8] = m8;
	}

	Matrix3d(const Span<const float> &span)
		: Matrix3d(span[0], span[1], span[2],
			   span[3], span[4], span[5],
			   span[6], span[7], span[8])
	{
	}

	static Matrix3d diag(float diag0, float diag1, float diag2)
	{
		return Matrix3d(diag0, 0, 0, 0, diag1, 0, 0, 0, diag2);
	}

	static Matrix3d identity()
	{
		return Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1);
	}

	Matrix3d transpose() const
	{
		return { m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8] };
	}

	Matrix3d cofactors() const
	{
		return { m[4] * m[8] - m[5] * m[7],
			 -(m[3] * m[8] - m[5] * m[6]),
			 m[3] * m[7] - m[4] * m[6],
			 -(m[1] * m[8] - m[2] * m[7]),
			 m[0] * m[8] - m[2] * m[6],
			 -(m[0] * m[7] - m[1] * m[6]),
			 m[1] * m[5] - m[2] * m[4],
			 -(m[0] * m[5] - m[2] * m[3]),
			 m[0] * m[4] - m[1] * m[3] };
	}

	Matrix3d adjugate() const
	{
		return cofactors().transpose();
	}

	float determinant() const
	{
		return m[0] * (m[4] * m[8] - m[5] * m[7]) -
		       m[1] * (m[3] * m[8] - m[5] * m[6]) +
		       m[2] * (m[3] * m[7] - m[4] * m[6]);
	}

	Matrix3d inverse() const
	{
		return adjugate() * (1.0 / determinant());
	}

	Matrix3d operator*(const Matrix3d &other) const
	{
		Matrix3d result;
		for (unsigned int i = 0; i < 3; i++) {
			for (unsigned int j = 0; j < 3; j++) {
				result.m[i * 3 + j] =
					m[i * 3 + 0] * other.m[0 + j] +
					m[i * 3 + 1] * other.m[3 + j] +
					m[i * 3 + 2] * other.m[6 + j];
			}
		}
		return result;
	}

	Matrix3d operator*(float f) const
	{
		Matrix3d result;
		for (unsigned int i = 0; i < 9; i++)
			result.m[i] = m[i] * f;
		return result;
	}

	float m[9];
};

void packScanlineSBGGR8(void *output, const void *input, unsigned int width)
{
	const uint8_t *in = static_cast<const uint8_t *>(input);
	uint8_t *out = static_cast<uint8_t *>(output);

	std::copy(in, in + width, out);
}

void packScanlineSBGGR10P(void *output, const void *input, unsigned int width)
{
	const uint8_t *in = static_cast<const uint8_t *>(input);
	uint8_t *out = static_cast<uint8_t *>(output);

	/* \todo Can this be made more efficient? */
	for (unsigned int x = 0; x < width; x += 4) {
		*out++ = in[0];
		*out++ = (in[4] & 0x03) << 6 | in[1] >> 2;
		*out++ = (in[1] & 0x03) << 6 | (in[4] & 0x0c) << 2 | in[2] >> 4;
		*out++ = (in[2] & 0x0f) << 4 | (in[4] & 0x30) >> 2 | in[3] >> 6;
		*out++ = (in[3] & 0x3f) << 2 | (in[4] & 0xc0) >> 6;
		in += 5;
	}
}

void packScanlineSBGGR12P(void *output, const void *input, unsigned int width)
{
	const uint8_t *in = static_cast<const uint8_t *>(input);
	uint8_t *out = static_cast<uint8_t *>(output);

	/* \todo Can this be made more efficient? */
	for (unsigned int i = 0; i < width; i += 2) {
		*out++ = in[0];
		*out++ = (in[2] & 0x0f) << 4 | in[1] >> 4;
		*out++ = (in[1] & 0x0f) << 4 | in[2] >> 4;
		in += 3;
	}
}

void thumbScanlineSBGGRxxP(const FormatInfo &info, void *output,
			   const void *input, unsigned int width,
			   unsigned int stride)
{
	const uint8_t *in = static_cast<const uint8_t *>(input);
	uint8_t *out = static_cast<uint8_t *>(output);

	/* Number of bytes corresponding to 16 pixels. */
	unsigned int skip = info.bitsPerSample * 16 / 8;

	for (unsigned int x = 0; x < width; x++) {
		uint8_t value = (in[0] + in[1] + in[stride] + in[stride + 1]) >> 2;
		*out++ = value;
		*out++ = value;
		*out++ = value;
		in += skip;
	}
}

void packScanlineIPU3(void *output, const void *input, unsigned int width)
{
	const uint8_t *in = static_cast<const uint8_t *>(input);
	uint16_t *out = static_cast<uint16_t *>(output);

	/*
	 * Upscale the 10-bit format to 16-bit as it's not trivial to pack it
	 * as 10-bit without gaps.
	 *
	 * \todo Improve packing to keep the 10-bit sample size.
	 */
	unsigned int x = 0;
	while (true) {
		for (unsigned int i = 0; i < 6; i++) {
			*out++ = (in[1] & 0x03) << 14 | (in[0] & 0xff) << 6;
			if (++x >= width)
				return;

			*out++ = (in[2] & 0x0f) << 12 | (in[1] & 0xfc) << 4;
			if (++x >= width)
				return;

			*out++ = (in[3] & 0x3f) << 10 | (in[2] & 0xf0) << 2;
			if (++x >= width)
				return;

			*out++ = (in[4] & 0xff) <<  8 | (in[3] & 0xc0) << 0;
			if (++x >= width)
				return;

			in += 5;
		}

		*out++ = (in[1] & 0x03) << 14 | (in[0] & 0xff) << 6;
		if (++x >= width)
			return;

		in += 2;
	}
}

void thumbScanlineIPU3([[maybe_unused]] const FormatInfo &info, void *output,
		       const void *input, unsigned int width,
		       unsigned int stride)
{
	uint8_t *out = static_cast<uint8_t *>(output);

	for (unsigned int x = 0; x < width; x++) {
		unsigned int pixel = x * 16;
		unsigned int block = pixel / 25;
		unsigned int pixelInBlock = pixel - block * 25;

		/*
		 * If the pixel is the last in the block cheat a little and
		 * move one pixel backward to avoid reading between two blocks
		 * and having to deal with the padding bits.
		 */
		if (pixelInBlock == 24)
			pixelInBlock--;

		const uint8_t *in = static_cast<const uint8_t *>(input)
				  + block * 32 + (pixelInBlock / 4) * 5;

		uint16_t val1, val2, val3, val4;
		switch (pixelInBlock % 4) {
		case 0:
			val1 = (in[1] & 0x03) << 14 | (in[0] & 0xff) << 6;
			val2 = (in[2] & 0x0f) << 12 | (in[1] & 0xfc) << 4;
			val3 = (in[stride + 1] & 0x03) << 14 | (in[stride + 0] & 0xff) << 6;
			val4 = (in[stride + 2] & 0x0f) << 12 | (in[stride + 1] & 0xfc) << 4;
			break;
		case 1:
			val1 = (in[2] & 0x0f) << 12 | (in[1] & 0xfc) << 4;
			val2 = (in[3] & 0x3f) << 10 | (in[2] & 0xf0) << 2;
			val3 = (in[stride + 2] & 0x0f) << 12 | (in[stride + 1] & 0xfc) << 4;
			val4 = (in[stride + 3] & 0x3f) << 10 | (in[stride + 2] & 0xf0) << 2;
			break;
		case 2:
			val1 = (in[3] & 0x3f) << 10 | (in[2] & 0xf0) << 2;
			val2 = (in[4] & 0xff) <<  8 | (in[3] & 0xc0) << 0;
			val3 = (in[stride + 3] & 0x3f) << 10 | (in[stride + 2] & 0xf0) << 2;
			val4 = (in[stride + 4] & 0xff) <<  8 | (in[stride + 3] & 0xc0) << 0;
			break;
		case 3:
			val1 = (in[4] & 0xff) <<  8 | (in[3] & 0xc0) << 0;
			val2 = (in[6] & 0x03) << 14 | (in[5] & 0xff) << 6;
			val3 = (in[stride + 4] & 0xff) <<  8 | (in[stride + 3] & 0xc0) << 0;
			val4 = (in[stride + 6] & 0x03) << 14 | (in[stride + 5] & 0xff) << 6;
			break;
		}

		uint8_t value = (val1 + val2 + val3 + val4) >> 10;
		*out++ = value;
		*out++ = value;
		*out++ = value;
	}
}

static const std::map<PixelFormat, FormatInfo> formatInfo = {
	{ formats::SBGGR8, {
		.bitsPerSample = 8,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR8,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SGBRG8, {
		.bitsPerSample = 8,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR8,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SGRBG8, {
		.bitsPerSample = 8,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR8,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SRGGB8, {
		.bitsPerSample = 8,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR8,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SBGGR10_CSI2P, {
		.bitsPerSample = 10,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SGBRG10_CSI2P, {
		.bitsPerSample = 10,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SGRBG10_CSI2P, {
		.bitsPerSample = 10,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SRGGB10_CSI2P, {
		.bitsPerSample = 10,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SBGGR12_CSI2P, {
		.bitsPerSample = 12,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SGBRG12_CSI2P, {
		.bitsPerSample = 12,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SGRBG12_CSI2P, {
		.bitsPerSample = 12,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SRGGB12_CSI2P, {
		.bitsPerSample = 12,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ formats::SBGGR10_IPU3, {
		.bitsPerSample = 16,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineIPU3,
		.thumbScanline = thumbScanlineIPU3,
	} },
	{ formats::SGBRG10_IPU3, {
		.bitsPerSample = 16,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineIPU3,
		.thumbScanline = thumbScanlineIPU3,
	} },
	{ formats::SGRBG10_IPU3, {
		.bitsPerSample = 16,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineIPU3,
		.thumbScanline = thumbScanlineIPU3,
	} },
	{ formats::SRGGB10_IPU3, {
		.bitsPerSample = 16,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineIPU3,
		.thumbScanline = thumbScanlineIPU3,
	} },
};

int DNGWriter::write(const char *filename, const Camera *camera,
		     const StreamConfiguration &config,
		     const ControlList &metadata,
		     [[maybe_unused]] const FrameBuffer *buffer,
		     const void *data)
{
	const ControlList &cameraProperties = camera->properties();

	const auto it = formatInfo.find(config.pixelFormat);
	if (it == formatInfo.cend()) {
		std::cerr << "Unsupported pixel format" << std::endl;
		return -EINVAL;
	}
	const FormatInfo *info = &it->second;

	TIFF *tif = TIFFOpen(filename, "w");
	if (!tif) {
		std::cerr << "Failed to open tiff file" << std::endl;
		return -EINVAL;
	}

	/*
	 * Scanline buffer, has to be large enough to store both a RAW scanline
	 * or a thumbnail scanline. The latter will always be much smaller than
	 * the former as we downscale by 16 in both directions.
	 */
	uint8_t scanline[(config.size.width * info->bitsPerSample + 7) / 8];

	toff_t rawIFDOffset = 0;
	toff_t exifIFDOffset = 0;

	/*
	 * Start with a thumbnail in IFD 0 for compatibility with TIFF baseline
	 * readers, as required by the TIFF/EP specification. Tags that apply to
	 * the whole file are stored here.
	 */
	const uint8_t version[] = { 1, 2, 0, 0 };

	TIFFSetField(tif, TIFFTAG_DNGVERSION, version);
	TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, version);
	TIFFSetField(tif, TIFFTAG_FILLORDER, FILLORDER_MSB2LSB);
	TIFFSetField(tif, TIFFTAG_MAKE, "libcamera");

	const auto &model = cameraProperties.get(properties::Model);
	if (model) {
		TIFFSetField(tif, TIFFTAG_MODEL, model->c_str());
		/* \todo set TIFFTAG_UNIQUECAMERAMODEL. */
	}

	TIFFSetField(tif, TIFFTAG_SOFTWARE, "qcam");
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);

	/*
	 * Thumbnail-specific tags. The thumbnail is stored as an RGB image
	 * with 1/16 of the raw image resolution. Greyscale would save space,
	 * but doesn't seem well supported by RawTherapee.
	 */
	TIFFSetField(tif, TIFFTAG_SUBFILETYPE, FILETYPE_REDUCEDIMAGE);
	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, config.size.width / 16);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, config.size.height / 16);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
	TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);

	/*
	 * Fill in some reasonable colour information in the DNG. We supply
	 * the "neutral" colour values which determine the white balance, and the
	 * "ColorMatrix1" which converts XYZ to (un-white-balanced) camera RGB.
	 * Note that this is not a "proper" colour calibration for the DNG,
	 * nonetheless, many tools should be able to render the colours better.
	 */
	float neutral[3] = { 1, 1, 1 };
	Matrix3d wbGain = Matrix3d::identity();
	/* From http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html */
	const Matrix3d rgb2xyz(0.4124564, 0.3575761, 0.1804375,
			       0.2126729, 0.7151522, 0.0721750,
			       0.0193339, 0.1191920, 0.9503041);
	Matrix3d ccm = Matrix3d::identity();
	/*
	 * Pick a reasonable number eps to protect against singularities. It
	 * should be comfortably larger than the point at which we run into
	 * numerical trouble, yet smaller than any plausible gain that we might
	 * apply to a colour, either explicitly or as part of the colour matrix.
	 */
	const double eps = 1e-2;

	const auto &colourGains = metadata.get(controls::ColourGains);
	if (colourGains) {
		if ((*colourGains)[0] > eps && (*colourGains)[1] > eps) {
			wbGain = Matrix3d::diag((*colourGains)[0], 1, (*colourGains)[1]);
			neutral[0] = 1.0 / (*colourGains)[0]; /* red */
			neutral[2] = 1.0 / (*colourGains)[1]; /* blue */
		}
	}

	const auto &ccmControl = metadata.get(controls::ColourCorrectionMatrix);
	if (ccmControl) {
		Matrix3d ccmSupplied(*ccmControl);
		if (ccmSupplied.determinant() > eps)
			ccm = ccmSupplied;
	}

	/*
	 * rgb2xyz is known to be invertible, and we've ensured above that both
	 * the ccm and wbGain matrices are non-singular, so the product of all
	 * three is guaranteed to be invertible too.
	 */
	Matrix3d colorMatrix1 = (rgb2xyz * ccm * wbGain).inverse();

	TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, colorMatrix1.m);
	TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, neutral);

	/*
	 * Reserve space for the SubIFD and ExifIFD tags, pointing to the IFD
	 * for the raw image and EXIF data respectively. The real offsets will
	 * be set later.
	 */
	TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &rawIFDOffset);
	TIFFSetField(tif, TIFFTAG_EXIFIFD, exifIFDOffset);

	/* Write the thumbnail. */
	const uint8_t *row = static_cast<const uint8_t *>(data);
	for (unsigned int y = 0; y < config.size.height / 16; y++) {
		info->thumbScanline(*info, &scanline, row,
				    config.size.width / 16, config.stride);

		if (TIFFWriteScanline(tif, &scanline, y, 0) != 1) {
			std::cerr << "Failed to write thumbnail scanline"
				  << std::endl;
			TIFFClose(tif);
			return -EINVAL;
		}

		row += config.stride * 16;
	}

	TIFFWriteDirectory(tif);

	/* Create a new IFD for the RAW image. */
	const uint16_t cfaRepeatPatternDim[] = { 2, 2 };
	const uint8_t cfaPlaneColor[] = {
		CFAPatternRed,
		CFAPatternGreen,
		CFAPatternBlue
	};

	TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, config.size.width);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, config.size.height);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, info->bitsPerSample);
	TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
	TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfaRepeatPatternDim);
	if (TIFFLIB_VERSION < 20201219)
		TIFFSetField(tif, TIFFTAG_CFAPATTERN, info->pattern);
	else
		TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, info->pattern);
	TIFFSetField(tif, TIFFTAG_CFAPLANECOLOR, 3, cfaPlaneColor);
	TIFFSetField(tif, TIFFTAG_CFALAYOUT, 1);

	const uint16_t blackLevelRepeatDim[] = { 2, 2 };
	float blackLevel[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	uint32_t whiteLevel = (1 << info->bitsPerSample) - 1;

	const auto &blackLevels = metadata.get(controls::SensorBlackLevels);
	if (blackLevels) {
		Span<const int32_t, 4> levels = *blackLevels;

		/*
		 * The black levels control is specified in R, Gr, Gb, B order.
		 * Map it to the TIFF tag that is specified in CFA pattern
		 * order.
		 */
		unsigned int green = (info->pattern[0] == CFAPatternRed ||
				      info->pattern[1] == CFAPatternRed)
				   ? 0 : 1;

		for (unsigned int i = 0; i < 4; ++i) {
			unsigned int level;

			switch (info->pattern[i]) {
			case CFAPatternRed:
				level = levels[0];
				break;
			case CFAPatternGreen:
				level = levels[green + 1];
				green = (green + 1) % 2;
				break;
			case CFAPatternBlue:
			default:
				level = levels[3];
				break;
			}

			/* Map the 16-bit value to the bits per sample range. */
			blackLevel[i] = level >> (16 - info->bitsPerSample);
		}
	}

	TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &blackLevelRepeatDim);
	TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 4, &blackLevel);
	TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &whiteLevel);

	/* Write RAW content. */
	row = static_cast<const uint8_t *>(data);
	for (unsigned int y = 0; y < config.size.height; y++) {
		info->packScanline(&scanline, row, config.size.width);

		if (TIFFWriteScanline(tif, &scanline, y, 0) != 1) {
			std::cerr << "Failed to write RAW scanline"
				  << std::endl;
			TIFFClose(tif);
			return -EINVAL;
		}

		row += config.stride;
	}

	/* Checkpoint the IFD to retrieve its offset, and write it out. */
	TIFFCheckpointDirectory(tif);
	rawIFDOffset = TIFFCurrentDirOffset(tif);
	TIFFWriteDirectory(tif);

	/* Create a new IFD for the EXIF data and fill it. */
	TIFFCreateEXIFDirectory(tif);

	/* Store creation time. */
	time_t rawtime;
	struct tm *timeinfo;
	char strTime[20];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(strTime, 20, "%Y:%m:%d %H:%M:%S", timeinfo);

	/*
	 * \todo Handle timezone information by setting OffsetTimeOriginal and
	 * OffsetTimeDigitized once libtiff catches up to the specification and
	 * has EXIFTAG_ defines to handle them.
	 */
	TIFFSetField(tif, EXIFTAG_DATETIMEORIGINAL, strTime);
	TIFFSetField(tif, EXIFTAG_DATETIMEDIGITIZED, strTime);

	const auto &analogGain = metadata.get(controls::AnalogueGain);
	if (analogGain) {
		uint16_t iso = std::min(std::max(*analogGain * 100, 0.0f), 65535.0f);
		TIFFSetField(tif, EXIFTAG_ISOSPEEDRATINGS, 1, &iso);
	}

	const auto &exposureTime = metadata.get(controls::ExposureTime);
	if (exposureTime)
		TIFFSetField(tif, EXIFTAG_EXPOSURETIME, *exposureTime / 1e6);

	TIFFWriteCustomDirectory(tif, &exifIFDOffset);

	/* Update the IFD offsets and close the file. */
	TIFFSetDirectory(tif, 0);
	TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &rawIFDOffset);
	TIFFSetField(tif, TIFFTAG_EXIFIFD, exifIFDOffset);
	TIFFWriteDirectory(tif);

	TIFFClose(tif);

	return 0;
}
