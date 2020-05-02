/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * dng_writer.cpp - DNG writer
 */

#include "dng_writer.h"

#include <iostream>
#include <map>

#include <tiffio.h>

using namespace libcamera;

enum CFAPatternColour : uint8_t {
	CFAPatternRed = 0,
	CFAPatternGreen = 1,
	CFAPatternBlue = 2,
};

struct FormatInfo {
	uint8_t bitsPerSample;
	CFAPatternColour pattern[4];
	void (*packScanline)(void *, const void *, unsigned int);
};

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

	/* \todo: Can this be made more efficient? */
	for (unsigned int i = 0; i < width; i += 2) {
		*out++ = in[0];
		*out++ = (in[2] & 0x0f) << 4 | in[1] >> 4;
		*out++ = (in[1] & 0x0f) << 4 | in[2] >> 4;
		in += 3;
	}
}

static const std::map<PixelFormat, FormatInfo> formatInfo = {
	{ PixelFormat(DRM_FORMAT_SBGGR10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR10P,
	} },
	{ PixelFormat(DRM_FORMAT_SGBRG10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR10P,
	} },
	{ PixelFormat(DRM_FORMAT_SGRBG10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR10P,
	} },
	{ PixelFormat(DRM_FORMAT_SRGGB10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR10P,
	} },
	{ PixelFormat(DRM_FORMAT_SBGGR12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR12P,
	} },
	{ PixelFormat(DRM_FORMAT_SGBRG12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR12P,
	} },
	{ PixelFormat(DRM_FORMAT_SGRBG12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR12P,
	} },
	{ PixelFormat(DRM_FORMAT_SRGGB12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR12P,
	} },
};

int DNGWriter::write(const char *filename, const Camera *camera,
		     const StreamConfiguration &config,
		     const FrameBuffer *buffer, const void *data)
{
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

	const uint8_t version[] = { 1, 2, 0, 0 };
	const uint16_t cfa_repeatpatterndim[] = { 2, 2 };
	TIFFSetField(tif, TIFFTAG_DNGVERSION, version);
	TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, version);
	TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, config.size.width);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, config.size.height);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, info->bitsPerSample);
	TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
	TIFFSetField(tif, TIFFTAG_FILLORDER, FILLORDER_MSB2LSB);
	TIFFSetField(tif, TIFFTAG_MAKE, "libcamera");
	TIFFSetField(tif, TIFFTAG_MODEL, camera->name().c_str());
	TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, camera->name().c_str());
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_SOFTWARE, "qcam");
	TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);
	TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfa_repeatpatterndim);
	TIFFSetField(tif, TIFFTAG_CFAPATTERN, info->pattern);
	TIFFSetField(tif, TIFFTAG_CFAPLANECOLOR, 3, "\00\01\02");
	TIFFSetField(tif, TIFFTAG_CFALAYOUT, 1);

	/* \todo Add more EXIF fields to output. */

	/* Write RAW content. */
	const uint8_t *row = static_cast<const uint8_t *>(data);
	uint8_t scanline[(config.size.width * info->bitsPerSample + 7) / 8];
	for (unsigned int y = 0; y < config.size.height; y++) {
		info->packScanline(&scanline, row, config.size.width);

		if (TIFFWriteScanline(tif, &scanline, y, 0) != 1) {
			std::cerr << "Failed to write scanline" << std::endl;
			TIFFClose(tif);
			return -EINVAL;
		}

		row += config.stride;
	}

	TIFFWriteDirectory(tif);

	TIFFClose(tif);

	return 0;
}
