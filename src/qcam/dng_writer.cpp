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
	void (*packScanline)(void *output, const void *input,
			     unsigned int width);
	void (*thumbScanline)(const FormatInfo &info, void *output,
			      const void *input, unsigned int width,
			      unsigned int stride);
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

void thumbScanlineSBGGRxxP(const FormatInfo &info, void *output,
			   const void *input, unsigned int width,
			   unsigned int stride)
{
	const uint8_t *in = static_cast<const uint8_t *>(input);
	uint8_t *out = static_cast<uint8_t *>(output);

	/* Number of bytes corresponding to 16 pixels. */
	unsigned int skip = info.bitsPerSample * 16 / 8;

	for (unsigned int x = 0; x < width; x++) {
		*out++ = (in[0] + in[1] + in[stride] + in[stride + 1]) >> 2;
		in += skip;
	}
}

static const std::map<PixelFormat, FormatInfo> formatInfo = {
	{ PixelFormat(DRM_FORMAT_SBGGR10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SGBRG10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SGRBG10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SRGGB10, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 10,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR10P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SBGGR12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternBlue, CFAPatternGreen, CFAPatternGreen, CFAPatternRed },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SGBRG12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternGreen, CFAPatternBlue, CFAPatternRed, CFAPatternGreen },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SGRBG12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternGreen, CFAPatternRed, CFAPatternBlue, CFAPatternGreen },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
	{ PixelFormat(DRM_FORMAT_SRGGB12, MIPI_FORMAT_MOD_CSI2_PACKED), {
		.bitsPerSample = 12,
		.pattern = { CFAPatternRed, CFAPatternGreen, CFAPatternGreen, CFAPatternBlue },
		.packScanline = packScanlineSBGGR12P,
		.thumbScanline = thumbScanlineSBGGRxxP,
	} },
};

int DNGWriter::write(const char *filename, const Camera *camera,
		     const StreamConfiguration &config,
		     const ControlList &metadata,
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

	/*
	 * Scanline buffer, has to be large enough to store both a RAW scanline
	 * or a thumbnail scanline. The latter will always be much smaller than
	 * the former as we downscale by 16 in both directions.
	 */
	uint8_t scanline[(config.size.width * info->bitsPerSample + 7) / 8];

	toff_t rawIFDOffset = 0;

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
	TIFFSetField(tif, TIFFTAG_MODEL, camera->name().c_str());
	TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, camera->name().c_str());
	TIFFSetField(tif, TIFFTAG_SOFTWARE, "qcam");
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);

	/*
	 * Thumbnail-specific tags. The thumbnail is stored as a greyscale image
	 * with 1/16 of the raw image resolution.
	 */
	TIFFSetField(tif, TIFFTAG_SUBFILETYPE, FILETYPE_REDUCEDIMAGE);
	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, config.size.width / 16);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, config.size.height / 16);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
	TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_UINT);

	/*
	 * Reserve space for the SubIFD tag, pointing to the IFD for the raw
	 * image. The real offset will be set later.
	 */
	TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &rawIFDOffset);

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
	TIFFSetField(tif, TIFFTAG_CFAPATTERN, info->pattern);
	TIFFSetField(tif, TIFFTAG_CFAPLANECOLOR, 3, cfaPlaneColor);
	TIFFSetField(tif, TIFFTAG_CFALAYOUT, 1);

	/* \todo Add more EXIF fields to output. */

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

	/* Update the IFD offsets and close the file. */
	TIFFSetDirectory(tif, 0);
	TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &rawIFDOffset);
	TIFFWriteDirectory(tif);

	TIFFClose(tif);

	return 0;
}
