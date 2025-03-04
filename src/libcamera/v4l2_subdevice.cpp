/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * V4L2 Subdevice
 */

#include "libcamera/internal/v4l2_subdevice.h"

#include <fcntl.h>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#pragma GCC diagnostic push
#if defined __SANITIZE_ADDRESS__ && defined __OPTIMIZE__
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <regex>
#pragma GCC diagnostic pop

#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"

/**
 * \file v4l2_subdevice.h
 * \brief V4L2 Subdevice API
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(V4L2)

/**
 * \class MediaBusFormatInfo
 * \brief Information about media bus formats
 *
 * The MediaBusFormatInfo class groups together information describing a media
 * bus format. It facilitates handling of media bus formats by providing data
 * commonly used in pipeline handlers.
 *
 * \var MediaBusFormatInfo::name
 * \brief The format name as a human-readable string, used as the text
 * representation of the format
 *
 * \var MediaBusFormatInfo::code
 * \brief The media bus format code described by this instance (MEDIA_BUS_FMT_*)
 *
 * \var MediaBusFormatInfo::type
 * \brief The media bus format type
 *
 * \var MediaBusFormatInfo::bitsPerPixel
 * \brief The average number of bits per pixel
 *
 * The number of bits per pixel averages the total number of bits for all
 * colour components over the whole image, excluding any padding bits or
 * padding pixels.
 *
 * For formats that transmit multiple or fractional pixels per sample, the
 * value will differ from the bus width.
 *
 * Formats that don't have a fixed number of bits per pixel, such as compressed
 * formats, or device-specific embedded data formats, report 0 in this field.
 *
 * \var MediaBusFormatInfo::colourEncoding
 * \brief The colour encoding type
 *
 * This field is valid for Type::Image formats only.
 */

/**
 * \enum MediaBusFormatInfo::Type
 * \brief The format type
 *
 * \var MediaBusFormatInfo::Type::Image
 * \brief The format describes image data
 *
 * \var MediaBusFormatInfo::Type::Metadata
 * \brief The format describes generic metadata
 *
 * \var MediaBusFormatInfo::Type::EmbeddedData
 * \brief The format describes sensor embedded data
 */

namespace {

const std::map<uint32_t, MediaBusFormatInfo> mediaBusFormatInfo{
	/* This table is sorted to match the order in linux/media-bus-format.h */
	{ MEDIA_BUS_FMT_RGB444_2X8_PADHI_BE, {
		.name = "RGB444_2X8_PADHI_BE",
		.code = MEDIA_BUS_FMT_RGB444_2X8_PADHI_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE, {
		.name = "RGB444_2X8_PADHI_LE",
		.code = MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE, {
		.name = "RGB555_2X8_PADHI_BE",
		.code = MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE, {
		.name = "RGB555_2X8_PADHI_LE",
		.code = MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB565_1X16, {
		.name = "RGB565_1X16",
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_BGR565_2X8_BE, {
		.name = "BGR565_2X8_BE",
		.code = MEDIA_BUS_FMT_BGR565_2X8_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_BGR565_2X8_LE, {
		.name = "BGR565_2X8_LE",
		.code = MEDIA_BUS_FMT_BGR565_2X8_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB565_2X8_BE, {
		.name = "RGB565_2X8_BE",
		.code = MEDIA_BUS_FMT_RGB565_2X8_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, {
		.name = "RGB565_2X8_LE",
		.code = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB666_1X18, {
		.name = "RGB666_1X18",
		.code = MEDIA_BUS_FMT_RGB666_1X18,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 18,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_BGR888_1X24, {
		.name = "BGR888_1X24",
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB888_1X24, {
		.name = "RGB888_1X24",
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB888_2X12_BE, {
		.name = "RGB888_2X12_BE",
		.code = MEDIA_BUS_FMT_RGB888_2X12_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB888_2X12_LE, {
		.name = "RGB888_2X12_LE",
		.code = MEDIA_BUS_FMT_RGB888_2X12_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB121212_1X36, {
		.name = "RGB121212_1X36",
		.code = MEDIA_BUS_FMT_RGB121212_1X36,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 36,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB202020_1X60, {
		.name = "RGB202020_1X60",
		.code = MEDIA_BUS_FMT_RGB202020_1X60,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 60,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_ARGB8888_1X32, {
		.name = "ARGB8888_1X32",
		.code = MEDIA_BUS_FMT_ARGB8888_1X32,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 32,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_Y8_1X8, {
		.name = "Y8_1X8",
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UV8_1X8, {
		.name = "UV8_1X8",
		.code = MEDIA_BUS_FMT_UV8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, {
		.name = "UYVY8_1_5X8",
		.code = MEDIA_BUS_FMT_UYVY8_1_5X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, {
		.name = "VYUY8_1_5X8",
		.code = MEDIA_BUS_FMT_VYUY8_1_5X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, {
		.name = "YUYV8_1_5X8",
		.code = MEDIA_BUS_FMT_YUYV8_1_5X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, {
		.name = "YVYU8_1_5X8",
		.code = MEDIA_BUS_FMT_YVYU8_1_5X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY8_2X8, {
		.name = "UYVY8_2X8",
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY8_2X8, {
		.name = "VYUY8_2X8",
		.code = MEDIA_BUS_FMT_VYUY8_2X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV8_2X8, {
		.name = "YUYV8_2X8",
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU8_2X8, {
		.name = "YVYU8_2X8",
		.code = MEDIA_BUS_FMT_YVYU8_2X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_Y10_1X10, {
		.name = "Y10_1X10",
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY10_2X10, {
		.name = "UYVY10_2X10",
		.code = MEDIA_BUS_FMT_UYVY10_2X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY10_2X10, {
		.name = "VYUY10_2X10",
		.code = MEDIA_BUS_FMT_VYUY10_2X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV10_2X10, {
		.name = "YUYV10_2X10",
		.code = MEDIA_BUS_FMT_YUYV10_2X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU10_2X10, {
		.name = "YVYU10_2X10",
		.code = MEDIA_BUS_FMT_YVYU10_2X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_Y12_1X12, {
		.name = "Y12_1X12",
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_Y16_1X16, {
		.name = "Y16_1X16",
		.code = MEDIA_BUS_FMT_Y16_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY8_1X16, {
		.name = "UYVY8_1X16",
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY8_1X16, {
		.name = "VYUY8_1X16",
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV8_1X16, {
		.name = "YUYV8_1X16",
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU8_1X16, {
		.name = "YVYU8_1X16",
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YDYUYDYV8_1X16, {
		.name = "YDYUYDYV8_1X16",
		.code = MEDIA_BUS_FMT_YDYUYDYV8_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY10_1X20, {
		.name = "UYVY10_1X20",
		.code = MEDIA_BUS_FMT_UYVY10_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY10_1X20, {
		.name = "VYUY10_1X20",
		.code = MEDIA_BUS_FMT_VYUY10_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV10_1X20, {
		.name = "YUYV10_1X20",
		.code = MEDIA_BUS_FMT_YUYV10_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU10_1X20, {
		.name = "YVYU10_1X20",
		.code = MEDIA_BUS_FMT_YVYU10_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUV8_1X24, {
		.name = "YUV8_1X24",
		.code = MEDIA_BUS_FMT_YUV8_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUV10_1X30, {
		.name = "YUV10_1X30",
		.code = MEDIA_BUS_FMT_YUV10_1X30,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 30,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_AYUV8_1X32, {
		.name = "AYUV8_1X32",
		.code = MEDIA_BUS_FMT_AYUV8_1X32,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 32,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY12_2X12, {
		.name = "UYVY12_2X12",
		.code = MEDIA_BUS_FMT_UYVY12_2X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY12_2X12, {
		.name = "VYUY12_2X12",
		.code = MEDIA_BUS_FMT_VYUY12_2X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV12_2X12, {
		.name = "YUYV12_2X12",
		.code = MEDIA_BUS_FMT_YUYV12_2X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU12_2X12, {
		.name = "YVYU12_2X12",
		.code = MEDIA_BUS_FMT_YVYU12_2X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY12_1X24, {
		.name = "UYVY12_1X24",
		.code = MEDIA_BUS_FMT_UYVY12_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY12_1X24, {
		.name = "VYUY12_1X24",
		.code = MEDIA_BUS_FMT_VYUY12_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV12_1X24, {
		.name = "YUYV12_1X24",
		.code = MEDIA_BUS_FMT_YUYV12_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU12_1X24, {
		.name = "YVYU12_1X24",
		.code = MEDIA_BUS_FMT_YVYU12_1X24,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, {
		.name = "SBGGR8_1X8",
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, {
		.name = "SGBRG8_1X8",
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, {
		.name = "SGRBG8_1X8",
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, {
		.name = "SRGGB8_1X8",
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_ALAW8_1X8, {
		.name = "SBGGR10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SBGGR10_ALAW8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG10_ALAW8_1X8, {
		.name = "SGBRG10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SGBRG10_ALAW8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG10_ALAW8_1X8, {
		.name = "SGRBG10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SGRBG10_ALAW8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB10_ALAW8_1X8, {
		.name = "SRGGB10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SRGGB10_ALAW8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8, {
		.name = "SBGGR10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8, {
		.name = "SGBRG10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8, {
		.name = "SGRBG10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8, {
		.name = "SRGGB10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE, {
		.name = "SBGGR10_2X8_PADHI_BE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE, {
		.name = "SBGGR10_2X8_PADHI_LE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE, {
		.name = "SBGGR10_2X8_PADLO_BE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE, {
		.name = "SBGGR10_2X8_PADLO_LE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, {
		.name = "SBGGR10_1X10",
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, {
		.name = "SGBRG10_1X10",
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, {
		.name = "SGRBG10_1X10",
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, {
		.name = "SRGGB10_1X10",
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, {
		.name = "SBGGR12_1X12",
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, {
		.name = "SGBRG12_1X12",
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, {
		.name = "SGRBG12_1X12",
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, {
		.name = "SRGGB12_1X12",
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR14_1X14, {
		.name = "SBGGR14_1X14",
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 14,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG14_1X14, {
		.name = "SGBRG14_1X14",
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 14,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG14_1X14, {
		.name = "SGRBG14_1X14",
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 14,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB14_1X14, {
		.name = "SRGGB14_1X14",
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 14,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR16_1X16, {
		.name = "SBGGR16_1X16",
		.code = MEDIA_BUS_FMT_SBGGR16_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SGBRG16_1X16, {
		.name = "SGBRG16_1X16",
		.code = MEDIA_BUS_FMT_SGBRG16_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SGRBG16_1X16, {
		.name = "SGRBG16_1X16",
		.code = MEDIA_BUS_FMT_SGRBG16_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SRGGB16_1X16, {
		.name = "SRGGB16_1X16",
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SBGGR20_1X20, {
		.name = "SBGGR20_1X20",
		.code = MEDIA_BUS_FMT_SBGGR20_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SGBRG20_1X20, {
		.name = "SGBRG20_1X20",
		.code = MEDIA_BUS_FMT_SGBRG20_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SGRBG20_1X20, {
		.name = "SGRBG20_1X20",
		.code = MEDIA_BUS_FMT_SGRBG20_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	{ MEDIA_BUS_FMT_SRGGB20_1X20, {
		.name = "SRGGB20_1X20",
		.code = MEDIA_BUS_FMT_SRGGB20_1X20,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW
	} },
	/* \todo Clarify colour encoding for HSV formats */
	{ MEDIA_BUS_FMT_AHSV8888_1X32, {
		.name = "AHSV8888_1X32",
		.code = MEDIA_BUS_FMT_AHSV8888_1X32,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 32,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_JPEG_1X8, {
		.name = "JPEG_1X8",
		.code = MEDIA_BUS_FMT_JPEG_1X8,
		.type = MediaBusFormatInfo::Type::Image,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_METADATA_FIXED, {
		.name = "METADATA_FIXED",
		.code = MEDIA_BUS_FMT_METADATA_FIXED,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 0,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_8, {
		.name = "META_8",
		.code = MEDIA_BUS_FMT_META_8,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_10, {
		.name = "META_10",
		.code = MEDIA_BUS_FMT_META_10,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_12, {
		.name = "META_12",
		.code = MEDIA_BUS_FMT_META_12,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_14, {
		.name = "META_14",
		.code = MEDIA_BUS_FMT_META_14,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 14,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_16, {
		.name = "META_16",
		.code = MEDIA_BUS_FMT_META_16,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_20, {
		.name = "META_20",
		.code = MEDIA_BUS_FMT_META_20,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_META_24, {
		.name = "META_24",
		.code = MEDIA_BUS_FMT_META_24,
		.type = MediaBusFormatInfo::Type::Metadata,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_CCS_EMBEDDED, {
		.name = "CCS_EMBEDDED",
		.code = MEDIA_BUS_FMT_CCS_EMBEDDED,
		.type = MediaBusFormatInfo::Type::EmbeddedData,
		.bitsPerPixel = 0,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_OV2740_EMBEDDED, {
		.name = "OV2740_EMBEDDED",
		.code = MEDIA_BUS_FMT_CCS_EMBEDDED,
		.type = MediaBusFormatInfo::Type::EmbeddedData,
		.bitsPerPixel = 0,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
};

} /* namespace */

/**
 * \fn bool MediaBusFormatInfo::isValid() const
 * \brief Check if the media bus format info is valid
 * \return True if the media bus format info is valid, false otherwise
 */

/**
 * \brief Retrieve information about a media bus format
 * \param[in] code The media bus format code
 * \return The MediaBusFormatInfo describing the \a code if known, or an invalid
 * MediaBusFormatInfo otherwise
 */
const MediaBusFormatInfo &MediaBusFormatInfo::info(uint32_t code)
{
	static const MediaBusFormatInfo invalid{};

	const auto it = mediaBusFormatInfo.find(code);
	if (it == mediaBusFormatInfo.end()) {
		LOG(V4L2, Warning)
			<< "Unsupported media bus format "
			<< utils::hex(code, 4);
		return invalid;
	}

	return it->second;
}

/**
 * \struct V4L2SubdeviceCapability
 * \brief struct v4l2_subdev_capability object wrapper and helpers
 *
 * The V4L2SubdeviceCapability structure manages the information returned by the
 * VIDIOC_SUBDEV_QUERYCAP ioctl.
 */

/**
 * \fn V4L2SubdeviceCapability::isReadOnly()
 * \brief Retrieve if a subdevice is registered as read-only
 *
 * A V4L2 subdevice is registered as read-only if V4L2_SUBDEV_CAP_RO_SUBDEV
 * is listed as part of its capabilities.
 *
 * \return True if the subdevice is registered as read-only, false otherwise
 */

/**
 * \fn V4L2SubdeviceCapability::hasStreams()
 * \brief Retrieve if a subdevice supports the V4L2 streams API
 * \return True if the subdevice supports the streams API, false otherwise
 */

/**
 * \struct V4L2SubdeviceFormat
 * \brief The V4L2 sub-device image format and sizes
 *
 * This structure describes the format of images when transported between
 * separate components connected through a physical bus, such as image sensor
 * and image receiver or between components part of the same System-on-Chip that
 * realize an image transformation pipeline.
 *
 * The format of images when transported on physical interconnections is known
 * as the "media bus format", and it is identified by a resolution and a pixel
 * format identification code, known as the "media bus code", not to be confused
 * with the fourcc code that identify the format of images when stored in memory
 * (see V4L2VideoDevice::V4L2DeviceFormat).
 *
 * Media Bus formats supported by the V4L2 APIs are described in Section
 * 4.15.3.4.1 of the "Part I - Video for Linux API" chapter of the "Linux Media
 * Infrastructure userspace API", part of the Linux kernel documentation.
 *
 * Image media bus formats are properties of the subdev pads.  When images are
 * transported between two media pads identified by a 0-indexed number, the
 * image bus format configured on the two pads should match (according to the
 * underlying driver format matching criteria) in order to prepare for a
 * successful streaming operation. For a more detailed description of the image
 * format negotiation process when performed between V4L2 subdevices, refer to
 * Section 4.15.3.1 of the above mentioned Linux kernel documentation section.
 */

/**
 * \var V4L2SubdeviceFormat::code
 * \brief The image format bus code
 */

/**
 * \var V4L2SubdeviceFormat::size
 * \brief The image size in pixels
 */

/**
 * \var V4L2SubdeviceFormat::colorSpace
 * \brief The color space of the pixels
 *
 * The color space of the image. When setting the format this may be
 * unset, in which case the driver gets to use its default color space.
 * After being set, this value should contain the color space that
 * was actually used. If this value is unset, then the color space chosen
 * by the driver could not be represented by the ColorSpace class (and
 * should probably be added).
 *
 * It is up to the pipeline handler or application to check if the
 * resulting color space is acceptable.
 */

/**
 * \brief Assemble and return a string describing the format
 * \return A string describing the V4L2SubdeviceFormat
 */
const std::string V4L2SubdeviceFormat::toString() const
{
	std::stringstream ss;
	ss << *this;

	return ss.str();
}

/**
 * \brief Insert a text representation of a V4L2SubdeviceFormat into an output
 * stream
 * \param[in] out The output stream
 * \param[in] f The V4L2SubdeviceFormat
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const V4L2SubdeviceFormat &f)
{
	out << f.size << "-";

	const auto it = mediaBusFormatInfo.find(f.code);

	if (it == mediaBusFormatInfo.end())
		out << utils::hex(f.code, 4);
	else
		out << it->second.name;

	return out;
}

/**
 * \class V4L2Subdevice
 * \brief A V4L2 subdevice as exposed by the Linux kernel
 *
 * The V4L2Subdevice class provides an API to the "Sub-device interface" as
 * described in section 4.15 of the "Linux Media Infrastructure userspace API"
 * chapter of the Linux Kernel documentation.
 *
 * A V4L2Subdevice is constructed from a MediaEntity instance, using the system
 * path of the entity's device node. No API call other than open(), isOpen()
 * and close() shall be called on an unopened device instance. Upon destruction
 * any device left open will be closed, and any resources released.
 */

/**
 * \typedef V4L2Subdevice::Formats
 * \brief A map of supported media bus formats to frame sizes
 */

/**
 * \enum V4L2Subdevice::Whence
 * \brief Specify the type of format for getFormat() and setFormat() operations
 * \var V4L2Subdevice::ActiveFormat
 * \brief The format operation applies to ACTIVE formats
 * \var V4L2Subdevice::TryFormat
 * \brief The format operation applies to TRY formats
 */

/**
 * \class V4L2Subdevice::Stream
 * \brief V4L2 subdevice stream
 *
 * This class identifies a subdev stream, by bundling the pad number with the
 * stream number. It is used in all stream-aware functions of the V4L2Subdevice
 * class to identify the stream the functions operate on.
 *
 * \var V4L2Subdevice::Stream::pad
 * \brief The 0-indexed pad number
 *
 * \var V4L2Subdevice::Stream::stream
 * \brief The stream number
 */

/**
 * \fn V4L2Subdevice::Stream::Stream()
 * \brief Construct a Stream with pad and stream set to 0
 */

/**
 * \fn V4L2Subdevice::Stream::Stream(unsigned int pad, unsigned int stream)
 * \brief Construct a Stream with a given \a pad and \a stream number
 * \param[in] pad The indexed pad number
 * \param[in] stream The stream number
 */

/**
 * \brief Compare streams for equality
 * \return True if the two streams are equal, false otherwise
 */
bool operator==(const V4L2Subdevice::Stream &lhs, const V4L2Subdevice::Stream &rhs)
{
	return lhs.pad == rhs.pad && lhs.stream == rhs.stream;
}

/**
 * \fn bool operator!=(const V4L2Subdevice::Stream &lhs, const V4L2Subdevice::Stream &rhs)
 * \brief Compare streams for inequality
 * \return True if the two streams are not equal, false otherwise
 */

/**
 * \brief Insert a text representation of a V4L2Subdevice::Stream into an
 * output stream
 * \param[in] out The output stream
 * \param[in] stream The V4L2Subdevice::Stream
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const V4L2Subdevice::Stream &stream)
{
	out << stream.pad << "/" << stream.stream;

	return out;
}

/**
 * \class V4L2Subdevice::Route
 * \brief V4L2 subdevice routing table entry
 *
 * This class models a route in the subdevice routing table. It is similar to
 * the v4l2_subdev_route structure, but uses the V4L2Subdevice::Stream class
 * for easier usage with the V4L2Subdevice stream-aware functions.
 *
 * \var V4L2Subdevice::Route::sink
 * \brief The sink stream of the route
 *
 * \var V4L2Subdevice::Route::source
 * \brief The source stream of the route
 *
 * \var V4L2Subdevice::Route::flags
 * \brief The route flags (V4L2_SUBDEV_ROUTE_FL_*)
 */

/**
 * \fn V4L2Subdevice::Route::Route()
 * \brief Construct a Route with default streams
 */

/**
 * \fn V4L2Subdevice::Route::Route(const Stream &sink, const Stream &source,
 * uint32_t flags)
 * \brief Construct a Route from \a sink to \a source
 * \param[in] sink The sink stream
 * \param[in] source The source stream
 * \param[in] flags The route flags
 */

/**
 * \brief Insert a text representation of a V4L2Subdevice::Route into an
 * output stream
 * \param[in] out The output stream
 * \param[in] route The V4L2Subdevice::Route
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const V4L2Subdevice::Route &route)
{
	out << route.sink << " -> " << route.source
	    << " (" << utils::hex(route.flags) << ")";

	return out;
}

/**
 * \typedef V4L2Subdevice::Routing
 * \brief V4L2 subdevice routing table
 *
 * This class stores a subdevice routing table as a vector of routes.
 */

/**
 * \brief Insert a text representation of a V4L2Subdevice::Routing into an
 * output stream
 * \param[in] out The output stream
 * \param[in] routing The V4L2Subdevice::Routing
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const V4L2Subdevice::Routing &routing)
{
	for (const auto &[i, route] : utils::enumerate(routing)) {
		out << "[" << i << "] " << route;
		if (i != routing.size() - 1)
			out << ", ";
	}

	return out;
}

/**
 * \brief Create a V4L2 subdevice from a MediaEntity using its device node
 * path
 */
V4L2Subdevice::V4L2Subdevice(const MediaEntity *entity)
	: V4L2Device(entity->deviceNode()), entity_(entity)
{
}

V4L2Subdevice::~V4L2Subdevice()
{
	close();
}

/**
 * \brief Open a V4L2 subdevice
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::open()
{
	int ret = V4L2Device::open(O_RDWR);
	if (ret)
		return ret;

	/*
	 * Try to query the subdev capabilities. The VIDIOC_SUBDEV_QUERYCAP API
	 * was introduced in kernel v5.8, ENOTTY errors must be ignored to
	 * support older kernels.
	 */
	caps_ = {};
	ret = ioctl(VIDIOC_SUBDEV_QUERYCAP, &caps_);
	if (ret < 0 && errno != ENOTTY) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Unable to query capabilities: " << strerror(-ret);
		return ret;
	}

	/* If the subdev supports streams, enable the streams API. */
	if (caps_.hasStreams()) {
		struct v4l2_subdev_client_capability clientCaps{};
		clientCaps.capabilities = V4L2_SUBDEV_CLIENT_CAP_STREAMS;

		ret = ioctl(VIDIOC_SUBDEV_S_CLIENT_CAP, &clientCaps);
		if (ret < 0) {
			ret = -errno;
			LOG(V4L2, Error)
				<< "Unable to set client capabilities: "
				<< strerror(-ret);
			return ret;
		}
	}

	return 0;
}

/**
 * \fn V4L2Subdevice::entity()
 * \brief Retrieve the media entity associated with the subdevice
 * \return The subdevice's associated media entity.
 */

/**
 * \brief Get selection rectangle \a rect for \a target
 * \param[in] stream The stream the rectangle is retrieved from
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[out] rect The retrieved selection rectangle
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getSelection(const Stream &stream, unsigned int target,
				Rectangle *rect)
{
	struct v4l2_subdev_selection sel = {};

	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = stream.pad;
	sel.stream = stream.stream;
	sel.target = target;
	sel.flags = 0;

	int ret = ioctl(VIDIOC_SUBDEV_G_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to get rectangle " << target << " on pad "
			<< stream << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}

/**
 * \fn V4L2Subdevice::getSelection(unsigned int pad, unsigned int target,
 * Rectangle *rect)
 * \brief Get selection rectangle \a rect for \a target
 * \param[in] pad The 0-indexed pad number the rectangle is retrieved from
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[out] rect The retrieved selection rectangle
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \brief Set selection rectangle \a rect for \a target
 * \param[in] stream The stream the rectangle is to be applied to
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[inout] rect The selection rectangle to be applied
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setSelection(const Stream &stream, unsigned int target,
				Rectangle *rect)
{
	struct v4l2_subdev_selection sel = {};

	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = stream.pad;
	sel.stream = stream.stream;
	sel.target = target;
	sel.flags = 0;

	sel.r.left = rect->x;
	sel.r.top = rect->y;
	sel.r.width = rect->width;
	sel.r.height = rect->height;

	int ret = ioctl(VIDIOC_SUBDEV_S_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to set rectangle " << target << " on pad "
			<< stream << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}

/**
 * \fn V4L2Subdevice::setSelection(unsigned int pad, unsigned int target,
 * Rectangle *rect)
 * \brief Set selection rectangle \a rect for \a target
 * \param[in] pad The 0-indexed pad number the rectangle is to be applied to
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[inout] rect The selection rectangle to be applied
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \brief Enumerate all media bus codes and frame sizes on a \a stream
 * \param[in] stream The stream to enumerate formats for
 *
 * Enumerate all media bus codes and frame sizes supported by the subdevice on
 * a \a stream.
 *
 * \return A list of the supported device formats
 */
V4L2Subdevice::Formats V4L2Subdevice::formats(const Stream &stream)
{
	Formats formats;

	if (stream.pad >= entity_->pads().size()) {
		LOG(V4L2, Error) << "Invalid pad: " << stream.pad;
		return {};
	}

	for (unsigned int code : enumPadCodes(stream)) {
		std::vector<SizeRange> sizes = enumPadSizes(stream, code);
		if (sizes.empty())
			return {};

		const auto inserted = formats.insert({ code, sizes });
		if (!inserted.second) {
			LOG(V4L2, Error)
				<< "Could not add sizes for media bus code "
				<< code << " on pad " << stream.pad;
			return {};
		}
	}

	return formats;
}

/**
 * \fn V4L2Subdevice::formats(unsigned int pad)
 * \brief Enumerate all media bus codes and frame sizes on a \a pad
 * \param[in] pad The 0-indexed pad number to enumerate formats on
 *
 * Enumerate all media bus codes and frame sizes supported by the subdevice on
 * a \a pad
 *
 * \return A list of the supported device formats
 */

std::optional<ColorSpace> V4L2Subdevice::toColorSpace(const v4l2_mbus_framefmt &format) const
{
	/*
	 * Only image formats have a color space, for other formats (such as
	 * metadata formats) the color space concept isn't applicable. V4L2
	 * subdev drivers return a colorspace set to V4L2_COLORSPACE_DEFAULT in
	 * that case (as well as for image formats when the driver hasn't
	 * bothered implementing color space support). Check the colorspace
	 * field here and return std::nullopt directly to avoid logging a
	 * warning.
	 */
	if (format.colorspace == V4L2_COLORSPACE_DEFAULT)
		return std::nullopt;

	PixelFormatInfo::ColourEncoding colourEncoding;
	const MediaBusFormatInfo &info = MediaBusFormatInfo::info(format.code);
	if (info.isValid()) {
		colourEncoding = info.colourEncoding;
	} else {
		LOG(V4L2, Warning)
			<< "Unknown subdev format "
			<< utils::hex(format.code, 4)
			<< ", defaulting to RGB encoding";

		colourEncoding = PixelFormatInfo::ColourEncodingRGB;
	}

	return V4L2Device::toColorSpace(format, colourEncoding);
}

/**
 * \brief Retrieve the image format set on one of the V4L2 subdevice streams
 * \param[in] stream The stream the format is to be retrieved from
 * \param[out] format The image bus format
 * \param[in] whence The format to get, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getFormat(const Stream &stream, V4L2SubdeviceFormat *format,
			     Whence whence)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = whence;
	subdevFmt.pad = stream.pad;
	subdevFmt.stream = stream.stream;

	int ret = ioctl(VIDIOC_SUBDEV_G_FMT, &subdevFmt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to get format on pad " << stream << ": "
			<< strerror(-ret);
		return ret;
	}

	format->size.width = subdevFmt.format.width;
	format->size.height = subdevFmt.format.height;
	format->code = subdevFmt.format.code;
	format->colorSpace = toColorSpace(subdevFmt.format);

	return 0;
}

/**
 * \fn V4L2Subdevice::getFormat(unsigned int pad, V4L2SubdeviceFormat *format,
 * Whence whence)
 * \brief Retrieve the image format set on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be retrieved from
 * \param[out] format The image bus format
 * \param[in] whence The format to get, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 * \return 0 on success or a negative error code otherwise
 */

/**
 * \brief Set an image format on one of the V4L2 subdevice pads
 * \param[in] stream The stream the format is to be applied to
 * \param[inout] format The image bus format to apply to the stream
 * \param[in] whence The format to set, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * Apply the requested image format to the desired stream and return the
 * actually applied format parameters, as getFormat() would do.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setFormat(const Stream &stream, V4L2SubdeviceFormat *format,
			     Whence whence)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = whence;
	subdevFmt.pad = stream.pad;
	subdevFmt.stream = stream.stream;
	subdevFmt.format.width = format->size.width;
	subdevFmt.format.height = format->size.height;
	subdevFmt.format.code = format->code;
	subdevFmt.format.field = V4L2_FIELD_NONE;
	if (format->colorSpace) {
		fromColorSpace(format->colorSpace, subdevFmt.format);

		/* The CSC flag is only applicable to source pads. */
		if (entity_->pads()[stream.pad]->flags() & MEDIA_PAD_FL_SOURCE)
			subdevFmt.format.flags |= V4L2_MBUS_FRAMEFMT_SET_CSC;
	}

	int ret = ioctl(VIDIOC_SUBDEV_S_FMT, &subdevFmt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to set format on pad " << stream << ": "
			<< strerror(-ret);
		return ret;
	}

	format->size.width = subdevFmt.format.width;
	format->size.height = subdevFmt.format.height;
	format->code = subdevFmt.format.code;
	format->colorSpace = toColorSpace(subdevFmt.format);

	return 0;
}

/**
 * \fn V4L2Subdevice::setFormat(unsigned int pad, V4L2SubdeviceFormat *format,
 * Whence whence)
 * \brief Set an image format on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be applied to
 * \param[inout] format The image bus format to apply to the subdevice's pad
 * \param[in] whence The format to set, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * Apply the requested image format to the desired media pad and return the
 * actually applied format parameters, as getFormat() would do.
 *
 * \return 0 on success or a negative error code otherwise
 */

namespace {

void routeFromKernel(V4L2Subdevice::Route &route,
		     const struct v4l2_subdev_route &kroute)
{
	route.sink.pad = kroute.sink_pad;
	route.sink.stream = kroute.sink_stream;
	route.source.pad = kroute.source_pad;
	route.source.stream = kroute.source_stream;
	route.flags = kroute.flags;
}

void routeToKernel(const V4L2Subdevice::Route &route,
		   struct v4l2_subdev_route &kroute)
{
	kroute.sink_pad = route.sink.pad;
	kroute.sink_stream = route.sink.stream;
	kroute.source_pad = route.source.pad;
	kroute.source_stream = route.source.stream;
	kroute.flags = route.flags;
}

/*
 * Legacy routing support for pre-v6.10-rc1 kernels. Drop when v6.12-rc1 gets
 * released.
 */
struct v4l2_subdev_routing_legacy {
	__u32 which;
	__u32 num_routes;
	__u64 routes;
	__u32 reserved[6];
};

#define VIDIOC_SUBDEV_G_ROUTING_LEGACY	_IOWR('V', 38, struct v4l2_subdev_routing_legacy)
#define VIDIOC_SUBDEV_S_ROUTING_LEGACY	_IOWR('V', 39, struct v4l2_subdev_routing_legacy)

} /* namespace */

int V4L2Subdevice::getRoutingLegacy(Routing *routing, Whence whence)
{
	struct v4l2_subdev_routing_legacy rt = {};

	rt.which = whence;

	int ret = ioctl(VIDIOC_SUBDEV_G_ROUTING_LEGACY, &rt);
	if (ret == 0 || ret == -ENOTTY)
		return ret;

	if (ret != -ENOSPC) {
		LOG(V4L2, Error)
			<< "Failed to retrieve number of routes: "
			<< strerror(-ret);
		return ret;
	}

	std::vector<struct v4l2_subdev_route> routes{ rt.num_routes };
	rt.routes = reinterpret_cast<uintptr_t>(routes.data());

	ret = ioctl(VIDIOC_SUBDEV_G_ROUTING_LEGACY, &rt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Failed to retrieve routes: " << strerror(-ret);
		return ret;
	}

	if (rt.num_routes != routes.size()) {
		LOG(V4L2, Error) << "Invalid number of routes";
		return -EINVAL;
	}

	routing->resize(rt.num_routes);

	for (const auto &[i, route] : utils::enumerate(routes))
		routeFromKernel((*routing)[i], route);

	return 0;
}

/**
 * \brief Retrieve the subdevice's internal routing table
 * \param[out] routing The routing table
 * \param[in] whence The routing table to get, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getRouting(Routing *routing, Whence whence)
{
	routing->clear();

	if (!caps_.hasStreams())
		return 0;

	struct v4l2_subdev_routing rt = {};

	rt.which = whence;

	int ret = ioctl(VIDIOC_SUBDEV_G_ROUTING, &rt);
	if (ret == -ENOTTY)
		return V4L2Subdevice::getRoutingLegacy(routing, whence);

	if (ret) {
		LOG(V4L2, Error)
			<< "Failed to retrieve number of routes: "
			<< strerror(-ret);
		return ret;
	}

	if (!rt.num_routes)
		return 0;

	std::vector<struct v4l2_subdev_route> routes{ rt.num_routes };
	rt.routes = reinterpret_cast<uintptr_t>(routes.data());

	rt.len_routes = rt.num_routes;
	rt.num_routes = 0;

	ret = ioctl(VIDIOC_SUBDEV_G_ROUTING, &rt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Failed to retrieve routes: " << strerror(-ret);
		return ret;
	}

	if (rt.num_routes != routes.size()) {
		LOG(V4L2, Error) << "Invalid number of routes";
		return -EINVAL;
	}

	routing->resize(rt.num_routes);

	for (const auto &[i, route] : utils::enumerate(routes))
		routeFromKernel((*routing)[i], route);

	return 0;
}

int V4L2Subdevice::setRoutingLegacy(Routing *routing, Whence whence)
{
	std::vector<struct v4l2_subdev_route> routes{ routing->size() };

	for (const auto &[i, route] : utils::enumerate(*routing))
		routeToKernel(route, routes[i]);

	struct v4l2_subdev_routing_legacy rt = {};
	rt.which = whence;
	rt.num_routes = routes.size();
	rt.routes = reinterpret_cast<uintptr_t>(routes.data());

	int ret = ioctl(VIDIOC_SUBDEV_S_ROUTING_LEGACY, &rt);
	if (ret) {
		LOG(V4L2, Error) << "Failed to set routes: " << strerror(-ret);
		return ret;
	}

	routes.resize(rt.num_routes);
	routing->resize(rt.num_routes);

	for (const auto &[i, route] : utils::enumerate(routes))
		routeFromKernel((*routing)[i], route);

	return 0;
}

/**
 * \brief Set a routing table on the V4L2 subdevice
 * \param[inout] routing The routing table
 * \param[in] whence The routing table to set, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * Apply to the V4L2 subdevice the routing table \a routing and update its
 * content to reflect the actually applied routing table as getRouting() would
 * do.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setRouting(Routing *routing, Whence whence)
{
	if (!caps_.hasStreams()) {
		routing->clear();
		return 0;
	}

	std::vector<struct v4l2_subdev_route> routes{ routing->size() };

	for (const auto &[i, route] : utils::enumerate(*routing))
		routeToKernel(route, routes[i]);

	struct v4l2_subdev_routing rt = {};
	rt.which = whence;
	rt.len_routes = routes.size();
	rt.num_routes = routes.size();
	rt.routes = reinterpret_cast<uintptr_t>(routes.data());

	int ret = ioctl(VIDIOC_SUBDEV_S_ROUTING, &rt);
	if (ret == -ENOTTY)
		return setRoutingLegacy(routing, whence);

	if (ret) {
		LOG(V4L2, Error) << "Failed to set routes: " << strerror(-ret);
		return ret;
	}

	/*
	 * The kernel may want to return more routes than we have space for. In
	 * that event, we must issue a VIDIOC_SUBDEV_G_ROUTING call to retrieve
	 * the additional routes.
	 */
	if (rt.num_routes > routes.size()) {
		routes.resize(rt.num_routes);

		rt.len_routes = rt.num_routes;
		rt.num_routes = 0;

		ret = ioctl(VIDIOC_SUBDEV_G_ROUTING, &rt);
		if (ret) {
			LOG(V4L2, Error)
				<< "Failed to retrieve routes: " << strerror(-ret);
			return ret;
		}
	}

	if (rt.num_routes != routes.size()) {
		LOG(V4L2, Error) << "Invalid number of routes";
		return -EINVAL;
	}

	routing->resize(rt.num_routes);

	for (const auto &[i, route] : utils::enumerate(routes))
		routeFromKernel((*routing)[i], route);

	return 0;
}

/**
 * \brief Retrieve the model name of the device
 *
 * The model name allows identification of the specific device model. This can
 * be used to infer device characteristics, for instance to determine the
 * analogue gain model of a camera sensor based on the sensor model name.
 *
 * Neither the V4L2 API nor the Media Controller API expose an explicit model
 * name. This function implements a heuristics to extract the model name from
 * the subdevice's entity name. This should produce accurate results for
 * I2C-based devices. If the heuristics can't match a known naming pattern,
 * the function returns the full entity name.
 *
 * \return The model name of the device
 */
const std::string &V4L2Subdevice::model()
{
	if (!model_.empty())
		return model_;

	/*
	 * Extract model name from the media entity name.
	 *
	 * There is no standardized naming scheme for sensor or other entities
	 * in the Linux kernel at the moment.
	 *
	 * - The most common rule, used by I2C sensors, associates the model
	 *   name with the I2C bus number and address (e.g. 'imx219 0-0010').
	 *
	 * - When the sensor exposes multiple subdevs, the model name is
	 *   usually followed by a function name, as in the smiapp driver (e.g.
	 *   'jt8ew9 pixel_array 0-0010').
	 *
	 * - The vimc driver names its sensors 'Sensor A' and 'Sensor B'.
	 *
	 * Other schemes probably exist. As a best effort heuristic, use the
	 * part of the entity name before the first space if the name contains
	 * an I2C address, and use the full entity name otherwise.
	 */
	std::string entityName = entity_->name();
	std::regex i2cRegex{ " [0-9]+-[0-9a-f]{4}" };
	std::smatch match;

	std::string model;
	if (std::regex_search(entityName, match, i2cRegex))
		model_ = entityName.substr(0, entityName.find(' '));
	else
		model_ = entityName;

	return model_;
}

/**
 * \fn V4L2Subdevice::caps()
 * \brief Retrieve the subdevice V4L2 capabilities
 * \return The subdevice V4L2 capabilities
 */

/**
 * \brief Create a new video subdevice instance from \a entity in media device
 * \a media
 * \param[in] media The media device where the entity is registered
 * \param[in] entity The media entity name
 *
 * \return A newly created V4L2Subdevice on success, nullptr otherwise
 */
std::unique_ptr<V4L2Subdevice>
V4L2Subdevice::fromEntityName(const MediaDevice *media,
			      const std::string &entity)
{
	MediaEntity *mediaEntity = media->getEntityByName(entity);
	if (!mediaEntity)
		return nullptr;

	return std::make_unique<V4L2Subdevice>(mediaEntity);
}

std::string V4L2Subdevice::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

std::vector<unsigned int> V4L2Subdevice::enumPadCodes(const Stream &stream)
{
	std::vector<unsigned int> codes;
	int ret;

	for (unsigned int index = 0; ; index++) {
		struct v4l2_subdev_mbus_code_enum mbusEnum = {};
		mbusEnum.pad = stream.pad;
		mbusEnum.stream = stream.stream;
		mbusEnum.index = index;
		mbusEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = ioctl(VIDIOC_SUBDEV_ENUM_MBUS_CODE, &mbusEnum);
		if (ret)
			break;

		codes.push_back(mbusEnum.code);
	}

	if (ret < 0 && ret != -EINVAL) {
		LOG(V4L2, Error)
			<< "Unable to enumerate formats on pad " << stream
			<< ": " << strerror(-ret);
		return {};
	}

	return codes;
}

std::vector<SizeRange> V4L2Subdevice::enumPadSizes(const Stream &stream,
						   unsigned int code)
{
	std::vector<SizeRange> sizes;
	int ret;

	for (unsigned int index = 0;; index++) {
		struct v4l2_subdev_frame_size_enum sizeEnum = {};
		sizeEnum.index = index;
		sizeEnum.pad = stream.pad;
		sizeEnum.stream = stream.stream;
		sizeEnum.code = code;
		sizeEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = ioctl(VIDIOC_SUBDEV_ENUM_FRAME_SIZE, &sizeEnum);
		if (ret)
			break;

		sizes.emplace_back(Size{ sizeEnum.min_width, sizeEnum.min_height },
				   Size{ sizeEnum.max_width, sizeEnum.max_height });
	}

	if (ret < 0 && ret != -EINVAL && ret != -ENOTTY) {
		LOG(V4L2, Error)
			<< "Unable to enumerate sizes on pad " << stream
			<< ": " << strerror(-ret);
		return {};
	}

	return sizes;
}

} /* namespace libcamera */
