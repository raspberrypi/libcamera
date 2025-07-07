/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * pisp.cpp - Pipeline handler for PiSP based Raspberry Pi devices
 */

#include <algorithm>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <unordered_map>
#include <vector>

#include <linux/dma-buf.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <libcamera/base/shared_fd.h>
#include <libcamera/formats.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/shared_mem_object.h"

#include "libpisp/backend/backend.hpp"
#include "libpisp/common/logging.hpp"
#include "libpisp/common/utils.hpp"
#include "libpisp/common/version.hpp"
#include "libpisp/frontend/frontend.hpp"
#include "libpisp/variants/variant.hpp"

#include "../common/pipeline_base.h"
#include "../common/rpi_stream.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(RPI)

using StreamFlag = RPi::Stream::StreamFlag;
using StreamParams = RPi::RPiCameraConfiguration::StreamParams;

namespace {

enum class Cfe : unsigned int { Output0, Embedded, Stats, Config };
enum class Isp : unsigned int { Input, Output0, Output1, TdnInput, TdnOutput,
				StitchInput, StitchOutput, Config };

/* Offset for all compressed buffers; mode for TDN and Stitch. */
constexpr unsigned int DefaultCompressionOffset = 2048;
constexpr unsigned int DefaultCompressionMode = 1;

const std::vector<std::pair<BayerFormat, unsigned int>> BayerToMbusCodeMap{
	{ { BayerFormat::BGGR, 8, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SBGGR8_1X8, },
	{ { BayerFormat::GBRG, 8, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGBRG8_1X8, },
	{ { BayerFormat::GRBG, 8, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGRBG8_1X8, },
	{ { BayerFormat::RGGB, 8, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SRGGB8_1X8, },
	{ { BayerFormat::BGGR, 10, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SBGGR10_1X10, },
	{ { BayerFormat::GBRG, 10, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGBRG10_1X10, },
	{ { BayerFormat::GRBG, 10, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGRBG10_1X10, },
	{ { BayerFormat::RGGB, 10, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SRGGB10_1X10, },
	{ { BayerFormat::BGGR, 12, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SBGGR12_1X12, },
	{ { BayerFormat::GBRG, 12, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGBRG12_1X12, },
	{ { BayerFormat::GRBG, 12, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGRBG12_1X12, },
	{ { BayerFormat::RGGB, 12, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SRGGB12_1X12, },
	{ { BayerFormat::BGGR, 14, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SBGGR14_1X14, },
	{ { BayerFormat::GBRG, 14, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGBRG14_1X14, },
	{ { BayerFormat::GRBG, 14, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGRBG14_1X14, },
	{ { BayerFormat::RGGB, 14, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SRGGB14_1X14, },
	{ { BayerFormat::BGGR, 16, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SBGGR16_1X16, },
	{ { BayerFormat::GBRG, 16, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGBRG16_1X16, },
	{ { BayerFormat::GRBG, 16, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SGRBG16_1X16, },
	{ { BayerFormat::RGGB, 16, BayerFormat::Packing::None }, MEDIA_BUS_FMT_SRGGB16_1X16, },
	{ { BayerFormat::BGGR, 16, BayerFormat::Packing::PISP1 }, MEDIA_BUS_FMT_SBGGR16_1X16, },
	{ { BayerFormat::GBRG, 16, BayerFormat::Packing::PISP1 }, MEDIA_BUS_FMT_SGBRG16_1X16, },
	{ { BayerFormat::GRBG, 16, BayerFormat::Packing::PISP1 }, MEDIA_BUS_FMT_SGRBG16_1X16, },
	{ { BayerFormat::RGGB, 16, BayerFormat::Packing::PISP1 }, MEDIA_BUS_FMT_SRGGB16_1X16, },
	{ { BayerFormat::RGGB, 16, BayerFormat::Packing::PISP1 }, MEDIA_BUS_FMT_SRGGB16_1X16, },
	{ { BayerFormat::MONO, 16, BayerFormat::Packing::None }, MEDIA_BUS_FMT_Y16_1X16, },
	{ { BayerFormat::MONO, 16, BayerFormat::Packing::PISP1 }, MEDIA_BUS_FMT_Y16_1X16, },
};

unsigned int bayerToMbusCode(const BayerFormat &bayer)
{
	const auto it = std::find_if(BayerToMbusCodeMap.begin(), BayerToMbusCodeMap.end(),
				     [bayer](const std::pair<BayerFormat, unsigned int> &match) {
						return bayer == match.first;
				     });

	if (it != BayerToMbusCodeMap.end())
		return it->second;

	return 0;
}

uint32_t mbusCodeUnpacked16(unsigned int code)
{
	BayerFormat bayer = BayerFormat::fromMbusCode(code);
	BayerFormat bayer16(bayer.order, 16, BayerFormat::Packing::None);

	return bayerToMbusCode(bayer16);
}

uint8_t toPiSPBayerOrder(V4L2PixelFormat format)
{
	BayerFormat bayer = BayerFormat::fromV4L2PixelFormat(format);

	switch (bayer.order) {
	case BayerFormat::Order::BGGR:
		return PISP_BAYER_ORDER_BGGR;
	case BayerFormat::Order::GBRG:
		return PISP_BAYER_ORDER_GBRG;
	case BayerFormat::Order::GRBG:
		return PISP_BAYER_ORDER_GRBG;
	case BayerFormat::Order::RGGB:
		return PISP_BAYER_ORDER_RGGB;
	case BayerFormat::Order::MONO:
		return PISP_BAYER_ORDER_GREYSCALE;
	default:
		ASSERT(0);
		return -1;
	}
}

pisp_image_format_config toPiSPImageFormat(V4L2DeviceFormat &format)
{
	pisp_image_format_config image = {};

	image.width = format.size.width;
	image.height = format.size.height;
	image.stride = format.planes[0].bpl;

	PixelFormat pix = format.fourcc.toPixelFormat();

	if (RPi::PipelineHandlerBase::isRaw(pix)) {
		BayerFormat bayer = BayerFormat::fromPixelFormat(pix);
		switch (bayer.packing) {
		case BayerFormat::Packing::None:
			image.format = PISP_IMAGE_FORMAT_BPS_16 +
				       PISP_IMAGE_FORMAT_UNCOMPRESSED;
			break;
		case BayerFormat::Packing::PISP1:
			image.format = PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
			break;
		case BayerFormat::Packing::PISP2:
			image.format = PISP_IMAGE_FORMAT_COMPRESSION_MODE_2;
			break;
		default:
			ASSERT(0);
		}
		return image;
	}

	switch (pix) {
	case formats::YUV420:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_420 +
			       PISP_IMAGE_FORMAT_PLANARITY_PLANAR;
		image.stride2 = image.stride / 2;
		break;
	case formats::NV12:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_420 +
			       PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR;
		image.stride2 = image.stride;
		break;
	case formats::NV21:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_420 +
			       PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR +
			       PISP_IMAGE_FORMAT_ORDER_SWAPPED;
		image.stride2 = image.stride;
		break;
	case formats::YUYV:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_422 +
			       PISP_IMAGE_FORMAT_PLANARITY_INTERLEAVED;
		break;
	case formats::UYVY:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_422 +
			       PISP_IMAGE_FORMAT_PLANARITY_INTERLEAVED +
			       PISP_IMAGE_FORMAT_ORDER_SWAPPED;
		break;
	case formats::NV16:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_422 +
			       PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR;
		image.stride2 = image.stride;
		break;
	case formats::NV61:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL +
			       PISP_IMAGE_FORMAT_BPS_8 +
			       PISP_IMAGE_FORMAT_SAMPLING_422 +
			       PISP_IMAGE_FORMAT_PLANARITY_SEMI_PLANAR +
			       PISP_IMAGE_FORMAT_ORDER_SWAPPED;
		image.stride2 = image.stride;
		break;
	case formats::RGB888:
	case formats::BGR888:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL;
		break;
	case formats::XRGB8888:
	case formats::XBGR8888:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL + PISP_IMAGE_FORMAT_BPP_32;
		break;
	case formats::RGBX8888:
	case formats::BGRX8888:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL + PISP_IMAGE_FORMAT_BPP_32 +
			       PISP_IMAGE_FORMAT_ORDER_SWAPPED;
		break;
	case formats::RGB161616:
	case formats::BGR161616:
		image.format = PISP_IMAGE_FORMAT_THREE_CHANNEL + PISP_IMAGE_FORMAT_BPS_16;
		break;
	default:
		LOG(RPI, Error) << "Pixel format " << pix << " unsupported";
		ASSERT(0);
	}

	return image;
}

void computeOptimalStride(V4L2DeviceFormat &format)
{
	pisp_image_format_config fmt = toPiSPImageFormat(format);

	libpisp::compute_optimal_stride(fmt);

	uint32_t fourcc = format.fourcc.fourcc();

	/*
	 * For YUV420/422 non-multiplanar formats, double the U/V stride for the
	 * Y-plane to ensure we get the optimal alignment on all three planes.
	 */
	if (fourcc == V4L2_PIX_FMT_YUV420 || fourcc == V4L2_PIX_FMT_YUV422P ||
	    fourcc == V4L2_PIX_FMT_YVU420)
		fmt.stride = fmt.stride2 * 2;

	format.planes[0].bpl = fmt.stride;
	format.planes[1].bpl = fmt.stride2;
	format.planes[2].bpl = fmt.stride2;

	/*
	 * Need to set planesCount correctly so that V4L2VideoDevice::trySetFormatMultiplane()
	 * copies the bpl fields correctly.
	 */
	const PixelFormat &pixFormat = format.fourcc.toPixelFormat();
	const PixelFormatInfo &info = PixelFormatInfo::info(pixFormat);
	format.planesCount = info.numPlanes();
}

void setupOutputClipping(const V4L2DeviceFormat &v4l2Format,
			 pisp_be_output_format_config &outputFormat)
{
	const PixelFormat &pixFormat = v4l2Format.fourcc.toPixelFormat();
	const PixelFormatInfo &info = PixelFormatInfo::info(pixFormat);

	if (info.colourEncoding != PixelFormatInfo::ColourEncodingYUV)
		return;

	if (v4l2Format.colorSpace == ColorSpace::Sycc) {
		outputFormat.lo = 0;
		outputFormat.hi = 65535;
		outputFormat.lo2 = 0;
		outputFormat.hi2 = 65535;
	} else if (v4l2Format.colorSpace == ColorSpace::Smpte170m ||
			v4l2Format.colorSpace == ColorSpace::Rec709) {
		outputFormat.lo = 16 << 8;
		outputFormat.hi = 235 << 8;
		outputFormat.lo2 = 16 << 8;
		outputFormat.hi2 = 240 << 8;
	} else {
		LOG(RPI, Warning)
			<< "Unrecognised colour space "
			<< ColorSpace::toString(v4l2Format.colorSpace)
			<< ", using full range";
		outputFormat.lo = 0;
		outputFormat.hi = 65535;
		outputFormat.lo2 = 0;
		outputFormat.hi2 = 65535;
	}
}

int dmabufSyncStart(const SharedFD &fd)
{
	struct dma_buf_sync dma_sync {};
	dma_sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;

	int ret = ::ioctl(fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);
	if (ret)
		LOG(RPI, Error) << "failed to lock-sync-write dma buf";

	return ret;
}

int dmabufSyncEnd(const SharedFD &fd)
{
	struct dma_buf_sync dma_sync {};
	dma_sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;

	int ret = ::ioctl(fd.get(), DMA_BUF_IOCTL_SYNC, &dma_sync);

	if (ret)
		LOG(RPI, Error) << "failed to unlock-sync-write dma buf";

	return ret;
}

void do32BitConversion(void *mem, unsigned int width, unsigned int height,
		       unsigned int stride)
{
	/*
	 * The arm64 version is actually not that much quicker because the
	 * vast bulk of the time is spent waiting for memory.
	 */
#if __aarch64__
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;
		uint64_t count = (width + 15) / 16;
		uint8_t *dest = ptr + count * 64;
		uint8_t *src = ptr + count * 48;

		/* Pre-decrement would have been nice. */
		asm volatile("movi v3.16b, #255 \n"
				"1: \n"
				"sub %[src], %[src], #48 \n"
				"sub %[dest], %[dest], #64 \n"
				"subs %[count], %[count], #1 \n"
				"ld3 {v0.16b, v1.16b, v2.16b}, [%[src]] \n"
				"st4 {v0.16b, v1.16b, v2.16b, v3.16b}, [%[dest]] \n"
				"b.gt 1b \n"
				: [count]"+r" (count)
				: [src]"r" (src), [dest]"r" (dest)
				: "cc", "v1", "v2", "v3", "v4", "memory"
				);
	}
#else
	std::vector<uint8_t> incache(3 * width);
	std::vector<uint8_t> outcache(4 * width);

	memcpy(incache.data(), mem, 3 * width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *ptr3 = incache.data();
		uint8_t *ptr4 = outcache.data();
		for (unsigned int i = 0; i < width; i++) {
			*(ptr4++) = *(ptr3++);
			*(ptr4++) = *(ptr3++);
			*(ptr4++) = *(ptr3++);
			*(ptr4++) = 255;
		}

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, 3 * width);
		memcpy(ptr, outcache.data(), 4 * width);
	}
#endif
}

void do16BitEndianSwap([[maybe_unused]] void *mem, [[maybe_unused]] unsigned int width,
		       [[maybe_unused]] unsigned int height, [[maybe_unused]] unsigned int stride)
{
#if __aarch64__
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;
		uint64_t count = (width + 7) / 8;

		asm volatile("1: \n"
				"ld1 {v1.16b}, [%[ptr]] \n"
				"rev16 v1.16b, v1.16b \n"
				"st1 {v1.16b}, [%[ptr]], #16 \n"
				"subs %[count], %[count], #1 \n"
				"b.gt 1b \n"
				: [count]"+r" (count), [ptr]"+r" (ptr)
				:
				: "cc", "v1", "memory"
				);
	}
#endif
}

void do14bitUnpack(void *mem, unsigned int width, unsigned int height,
		   unsigned int stride)
{
	std::vector<uint8_t> cache(stride);

	for (unsigned int j = 0; j < height; j++) {
		const uint8_t *in = ((uint8_t *)mem) + j * stride;
		uint8_t *out = ((uint8_t *)mem) + j * stride;
		uint8_t *p = cache.data();

		std::memcpy(p, in, stride);
		for (unsigned int i = 0; i < width; i += 4, p += 7) {
			uint16_t p0 = (p[0] << 8) | ((p[4] & 0x3f) << 2);
			uint16_t p1 = (p[1] << 8) | ((p[4] & 0xc0) >> 4) | ((p[5] & 0x0f) << 4);
			uint16_t p2 = (p[2] << 8) | ((p[5] & 0xf0) >> 2) | ((p[6] & 0x03) << 6);
			uint16_t p3 = (p[3] << 8) | (p[6] & 0xfc);

			*(uint16_t *)(out + i * 2 + 0) = p0;
			*(uint16_t *)(out + i * 2 + 2) = p1;
			*(uint16_t *)(out + i * 2 + 4) = p2;
			*(uint16_t *)(out + i * 2 + 6) = p3;
		}
	}
}

void downscaleInterleaved3(void *mem, unsigned int height, unsigned int src_width,
			   unsigned int stride)
{
	std::vector<uint8_t> incache(3 * src_width);
	unsigned int dst_width = src_width / 2;
	std::vector<uint8_t> outcache(3 * dst_width);

	memcpy(incache.data(), mem, 3 * src_width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *src = incache.data(), *dst = outcache.data();
		for (unsigned int i = 0; i < dst_width; i++, src += 6, dst += 3) {
			dst[0] = ((int)src[0] + (int)src[3] + 1) >> 1;
			dst[1] = ((int)src[1] + (int)src[4] + 1) >> 1;
			dst[2] = ((int)src[2] + (int)src[5] + 1) >> 1;
		}

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, 3 * src_width);
		memcpy(ptr, outcache.data(), 3 * dst_width);
	}
}

void downscaleInterleaved4(void *mem, unsigned int height, unsigned int src_width,
			   unsigned int stride)
{
	std::vector<uint8_t> incache(4 * src_width);
	unsigned int dst_width = src_width / 2;
	std::vector<uint8_t> outcache(4 * dst_width);

	memcpy(incache.data(), mem, 4 * src_width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *src = incache.data(), *dst = outcache.data();
		for (unsigned int i = 0; i < dst_width; i++, src += 8, dst += 4) {
			dst[0] = ((int)src[0] + (int)src[4] + 1) >> 1;
			dst[1] = ((int)src[1] + (int)src[5] + 1) >> 1;
			dst[2] = ((int)src[2] + (int)src[6] + 1) >> 1;
			dst[3] = ((int)src[3] + (int)src[7] + 1) >> 1;
		}

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, 4 * src_width);
		memcpy(ptr, outcache.data(), 4 * dst_width);
	}
}

void downscalePlaneInternal(void *mem, unsigned int height, unsigned int src_width,
			    unsigned int stride, std::vector<uint8_t> &incache,
			    std::vector<uint8_t> &outcache)
{
	unsigned int dst_width = src_width / 2;
	memcpy(incache.data(), mem, src_width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *src = incache.data(), *dst = outcache.data();
		for (unsigned int i = 0; i < dst_width; i++, src += 2, dst++)
			*dst = ((int)src[0] + (int)src[1] + 1) >> 1;

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, src_width);
		memcpy(ptr, outcache.data(), dst_width);
	}
}

void downscalePlanar420(void *memY, void *memU, void *memV, unsigned int height,
			unsigned int src_width, unsigned int stride)
{
	std::vector<uint8_t> incache(src_width);
	std::vector<uint8_t> outcache(src_width / 2);

	downscalePlaneInternal(memY, height, src_width, stride, incache, outcache);
	downscalePlaneInternal(memU, height / 2, src_width / 2, stride / 2, incache, outcache);
	downscalePlaneInternal(memV, height / 2, src_width / 2, stride / 2, incache, outcache);
}

void downscalePlanar422(void *memY, void *memU, void *memV,
			unsigned int height, unsigned int src_width, unsigned int stride)
{
	std::vector<uint8_t> incache(src_width);
	std::vector<uint8_t> outcache(src_width / 2);

	downscalePlaneInternal(memY, height, src_width, stride, incache, outcache);
	downscalePlaneInternal(memU, height, src_width / 2, stride / 2, incache, outcache);
	downscalePlaneInternal(memV, height, src_width / 2, stride / 2, incache, outcache);
}

void downscaleInterleavedYuyv(void *mem, unsigned int height, unsigned int src_width,
			      unsigned int stride)
{
	std::vector<uint8_t> incache(2 * src_width);
	unsigned int dst_width = src_width / 2;
	std::vector<uint8_t> outcache(2 * dst_width);

	memcpy(incache.data(), mem, 2 * src_width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *src = incache.data(), *dst = outcache.data();
		for (unsigned int i = 0; i < dst_width; i++, src += 8, dst += 4) {
			dst[0] = ((int)src[0] + (int)src[2] + 1) >> 1;
			dst[1] = ((int)src[1] + (int)src[5] + 1) >> 1;
			dst[2] = ((int)src[4] + (int)src[6] + 1) >> 1;
			dst[3] = ((int)src[3] + (int)src[7] + 1) >> 1;
		}

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, 4 * src_width);
		memcpy(ptr, outcache.data(), 2 * dst_width);
	}
}

void downscaleInterleavedUyvy(void *mem, unsigned int height, unsigned int src_width,
			      unsigned int stride)
{
	std::vector<uint8_t> incache(2 * src_width);
	unsigned int dst_width = src_width / 2;
	std::vector<uint8_t> outcache(2 * dst_width);

	memcpy(incache.data(), mem, 2 * src_width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *src = incache.data(), *dst = outcache.data();
		for (unsigned int i = 0; i < dst_width; i++, src += 8, dst += 4) {
			dst[0] = ((int)src[0] + (int)src[4] + 1) >> 1;
			dst[1] = ((int)src[1] + (int)src[3] + 1) >> 1;
			dst[2] = ((int)src[2] + (int)src[6] + 1) >> 1;
			dst[3] = ((int)src[5] + (int)src[7] + 1) >> 1;
		}

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, 4 * src_width);
		memcpy(ptr, outcache.data(), 2 * dst_width);
	}
}

void downscaleInterleaved2Internal(void *mem, unsigned int height, unsigned int src_width,
				   unsigned int stride, std::vector<uint8_t> &incache,
				   std::vector<uint8_t> &outcache)
{
	unsigned int dst_width = src_width / 2;
	memcpy(incache.data(), mem, 2 * src_width);
	for (unsigned int j = 0; j < height; j++) {
		uint8_t *ptr = (uint8_t *)mem + j * stride;

		uint8_t *src = incache.data(), *dst = outcache.data();
		for (unsigned int i = 0; i < dst_width; i++, src += 4, dst += 2) {
			dst[0] = ((int)src[0] + (int)src[2] + 1) >> 1;
			dst[1] = ((int)src[1] + (int)src[3] + 1) >> 1;
		}

		if (j < height - 1)
			memcpy(incache.data(), ptr + stride, 2 * src_width);
		memcpy(ptr, outcache.data(), 2 * dst_width);
	}
}

void downscaleSemiPlanar420(void *memY, void *memUV, unsigned int height,
			    unsigned int src_width, unsigned int stride)
{
	std::vector<uint8_t> incache(src_width);
	std::vector<uint8_t> outcache(src_width / 2);

	downscalePlaneInternal(memY, height, src_width, stride, incache, outcache);
	downscaleInterleaved2Internal(memUV, height / 2, src_width / 2, stride,
				      incache, outcache);
}

void downscaleStreamBuffer(RPi::Stream *stream, int index)
{
	unsigned int downscale = stream->swDownscale();
	/* Must be a power of 2. */
	ASSERT((downscale & (downscale - 1)) == 0);

	unsigned int stride = stream->configuration().stride;
	unsigned int dst_width = stream->configuration().size.width;
	unsigned int height = stream->configuration().size.height;
	const PixelFormat &pixFormat = stream->configuration().pixelFormat;
	const RPi::BufferObject &b = stream->getBuffer(index);
	void *mem = b.mapped->planes()[0].data();
	ASSERT(b.mapped);

	/* Do repeated downscale-by-2 in place until we're done. */
	for (; downscale > 1; downscale >>= 1) {
		unsigned int src_width = downscale * dst_width;

		if (pixFormat == formats::RGB888 || pixFormat == formats::BGR888) {
			downscaleInterleaved3(mem, height, src_width, stride);
		} else if (pixFormat == formats::XRGB8888 || pixFormat == formats::XBGR8888) {
			/* On some devices these may actually be 24bpp at this point. */
			if (stream->getFlags() & StreamFlag::Needs32bitConv)
				downscaleInterleaved3(mem, height, src_width, stride);
			else
				downscaleInterleaved4(mem, height, src_width, stride);
		} else if (pixFormat == formats::YUV420 || pixFormat == formats::YVU420) {
			/* These may look like either single or multi-planar buffers. */
			void *mem1;
			void *mem2;
			if (b.mapped->planes().size() == 3) {
				mem1 = b.mapped->planes()[1].data();
				mem2 = b.mapped->planes()[2].data();
			} else {
				unsigned int ySize = height * stride;
				mem1 = static_cast<uint8_t *>(mem) + ySize;
				mem2 = static_cast<uint8_t *>(mem1) + ySize / 4;
			}
			downscalePlanar420(mem, mem1, mem2, height, src_width, stride);
		} else if (pixFormat == formats::YUV422 || pixFormat == formats::YVU422) {
			/* These may look like either single or multi-planar buffers. */
			void *mem1;
			void *mem2;
			if (b.mapped->planes().size() == 3) {
				mem1 = b.mapped->planes()[1].data();
				mem2 = b.mapped->planes()[2].data();
			} else {
				unsigned int ySize = height * stride;
				mem1 = static_cast<uint8_t *>(mem) + ySize;
				mem2 = static_cast<uint8_t *>(mem1) + ySize / 2;
			}
			downscalePlanar422(mem, mem1, mem2, height, src_width, stride);
		} else if (pixFormat == formats::YUYV || pixFormat == formats::YVYU) {
			downscaleInterleavedYuyv(mem, height, src_width, stride);
		} else if (pixFormat == formats::UYVY || pixFormat == formats::VYUY) {
			downscaleInterleavedUyvy(mem, height, src_width, stride);
		} else if (pixFormat == formats::NV12 || pixFormat == formats::NV21) {
			/* These may look like either single or multi-planar buffers. */
			void *mem1;
			if (b.mapped->planes().size() == 2)
				mem1 = b.mapped->planes()[1].data();
			else
				mem1 = static_cast<uint8_t *>(mem) + height * stride;
			downscaleSemiPlanar420(mem, mem1, height, src_width, stride);
		} else {
			LOG(RPI, Error) << "Sw downscale unsupported for " << pixFormat;
			ASSERT(0);
		}
	}
}

/* Return largest width of any of these streams (or of the camera input). */
unsigned int getLargestWidth(const V4L2SubdeviceFormat &sensorFormat,
			     const std::vector<StreamParams> &outStreams)
{
	unsigned int largestWidth = sensorFormat.size.width;

	for (const auto &stream : outStreams)
		largestWidth = std::max(largestWidth, stream.cfg->size.width);

	return largestWidth;
}

/* Return the minimum number of pixels required to write out multiples of 16 bytes. */
unsigned int getFormatAlignment(const V4L2PixelFormat &fourcc)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(fourcc);
	unsigned int formatAlignment = 0;
	for (const auto &plane : info.planes) {
		if (plane.bytesPerGroup) {
			/* How many pixels we need in this plane for a multiple of 16 bytes (??). */
			unsigned int align = 16 * info.pixelsPerGroup /
						std::gcd(16u, plane.bytesPerGroup);
			formatAlignment = std::max(formatAlignment, align);
		}
	}

	return formatAlignment;
}

/* Calculate the amount of software downscale required (which is a power of 2). */
unsigned int calculateSwDownscale(const V4L2DeviceFormat &format, unsigned int largestWidth,
				  unsigned int platformMaxDownscale)
{
	unsigned int formatAlignment = getFormatAlignment(format.fourcc);
	unsigned int maxDownscale = platformMaxDownscale * 16 / formatAlignment;
	unsigned int limitWidth = largestWidth / maxDownscale;

	unsigned int hwWidth = format.size.width;
	unsigned int swDownscale = 1;
	for (; hwWidth < limitWidth; hwWidth *= 2, swDownscale *= 2);

	return swDownscale;
}

} /* namespace */

using ::libpisp::BackEnd;
using ::libpisp::FrontEnd;

class PiSPCameraData final : public RPi::CameraData
{
public:
	PiSPCameraData(PipelineHandler *pipe, const libpisp::PiSPVariant &variant)
		: RPi::CameraData(pipe), pispVariant_(variant)
	{
		/* Initialise internal libpisp logging. */
		::libpisp::logging_init();
		LOG(RPI, Info) << "libpisp version " << ::libpisp::version();
	}

	~PiSPCameraData()
	{
		freeBuffers();
	}

	V4L2VideoDevice::Formats ispFormats() const override
	{
		return isp_[Isp::Output0].dev()->formats();
	}

	V4L2VideoDevice::Formats rawFormats() const override
	{
		return cfe_[Cfe::Output0].dev()->formats();
	}

	V4L2VideoDevice *frontendDevice() override
	{
		return cfe_[Cfe::Output0].dev();
	}

	CameraConfiguration::Status
	platformValidate(RPi::RPiCameraConfiguration *rpiConfig) const override;

	int platformPipelineConfigure(const std::unique_ptr<YamlObject> &root) override;

	void platformStart() override;
	void platformStop() override;
	void platformFreeBuffers() override;

	void cfeBufferDequeue(FrameBuffer *buffer);
	void beInputDequeue(FrameBuffer *buffer);
	void beOutputDequeue(FrameBuffer *buffer);

	void processStatsComplete(const ipa::RPi::BufferIds &buffers);
	void prepareIspComplete(const ipa::RPi::BufferIds &buffers, bool stitchSwapBuffers);
	void setCameraTimeout(uint32_t maxFrameLengthMs);

	/* Array of CFE and ISP device streams and associated buffers/streams. */
	RPi::Device<Cfe, 4> cfe_;
	RPi::Device<Isp, 8> isp_;

	const libpisp::PiSPVariant &pispVariant_;

	/* Frontend/Backend objects shared with the IPA. */
	SharedMemObject<FrontEnd> fe_;
	SharedMemObject<BackEnd> be_;
	bool beEnabled_;

	std::unique_ptr<V4L2Subdevice> csi2Subdev_;
	std::unique_ptr<V4L2Subdevice> feSubdev_;

	std::vector<FrameBuffer *> tdnBuffers_;
	std::vector<FrameBuffer *> stitchBuffers_;
	unsigned int tdnInputIndex_;
	unsigned int stitchInputIndex_;

	struct Config {
		/*
		 * Number of CFE config and stats buffers to allocate and use. A
		 * larger number minimises the possibility of dropping frames,
		 * but increases the latency for updating the HW configuration.
		 */
		unsigned int numCfeConfigStatsBuffers;
		/*
		 * Number of jobs to queue ahead to the CFE on startup.
		 * A larger number will increase latency for 3A changes.
		 */
		unsigned int numCfeConfigQueue;
		/* Don't use BE temporal denoise and free some memory resources. */
		bool disableTdn;
		/* Don't use BE HDR and free some memory resources. */
		bool disableHdr;
	};

	Config config_;

	bool adjustDeviceFormat(V4L2DeviceFormat &format) const;

private:
	int platformConfigure(const RPi::RPiCameraConfiguration *rpiConfig) override;

	int platformConfigureIpa([[maybe_unused]] ipa::RPi::ConfigParams &params) override
	{
		return 0;
	}

	int platformInitIpa(ipa::RPi::InitParams &params) override;

	int configureEntities(V4L2SubdeviceFormat sensorFormat,
			      V4L2SubdeviceFormat &embeddedFormat);
	int configureCfe();
	bool calculateCscConfiguration(const V4L2DeviceFormat &v4l2Format, pisp_be_ccm_config &csc);
	int configureBe(const std::optional<ColorSpace> &yuvColorSpace);

	void platformSetIspCrop(unsigned int index, const Rectangle &ispCrop) override;

	void prepareCfe();
	void prepareBe(uint32_t bufferId, bool stitchSwapBuffers);

	void tryRunPipeline() override;

	struct CfeJob {
		ControlList sensorControls;
		unsigned int delayContext;
		std::unordered_map<const RPi::Stream *, FrameBuffer *> buffers;
	};

	std::queue<CfeJob> cfeJobQueue_;

	bool cfeJobComplete() const
	{
		if (cfeJobQueue_.empty())
			return false;

		const CfeJob &job = cfeJobQueue_.back();
		return job.buffers.count(&cfe_[Cfe::Output0]) &&
		       job.buffers.count(&cfe_[Cfe::Stats]) &&
		       (!sensorMetadata_ ||
				job.buffers.count(&cfe_[Cfe::Embedded]));
	}

	std::string last_dump_file_;
};

class PipelineHandlerPiSP : public RPi::PipelineHandlerBase
{
public:
	PipelineHandlerPiSP(CameraManager *manager)
		: RPi::PipelineHandlerBase(manager)
	{
	}

	~PipelineHandlerPiSP()
	{
	}

	bool match(DeviceEnumerator *enumerator) override;

private:
	PiSPCameraData *cameraData(Camera *camera)
	{
		return static_cast<PiSPCameraData *>(camera->_d());
	}

	int prepareBuffers(Camera *camera) override;
	int platformRegister(std::unique_ptr<RPi::CameraData> &cameraData,
			     MediaDevice *cfe, MediaDevice *isp) override;
};

bool PipelineHandlerPiSP::match(DeviceEnumerator *enumerator)
{
	constexpr unsigned int numCfeDevices = 2;

	/*
	 * Loop over all CFE instances, but return out once a match is found.
	 * This is to ensure we correctly enumerate the camera when an instance
	 * of the CFE has registered with media controller, but has not registered
	 * device nodes due to a sensor subdevice failure.
	 */
	for (unsigned int i = 0; i < numCfeDevices; i++) {
		DeviceMatch cfe("rp1-cfe");
		cfe.add("rp1-cfe-fe_image0");
		cfe.add("rp1-cfe-fe_stats");
		cfe.add("rp1-cfe-fe_config");
		MediaDevice *cfeDevice = acquireMediaDevice(enumerator, cfe);

		if (!cfeDevice) {
			LOG(RPI, Debug) << "Unable to acquire a CFE instance";
			break;
		}

		DeviceMatch isp("pispbe");
		isp.add("pispbe-input");
		isp.add("pispbe-config");
		isp.add("pispbe-output0");
		isp.add("pispbe-output1");
		isp.add("pispbe-tdn_output");
		isp.add("pispbe-tdn_input");
		isp.add("pispbe-stitch_output");
		isp.add("pispbe-stitch_input");
		MediaDevice *ispDevice = acquireMediaDevice(enumerator, isp);

		if (!ispDevice) {
			LOG(RPI, Debug) << "Unable to acquire ISP instance";
			break;
		}

		/*
		 * The loop below is used to register multiple cameras behind
		 * one or more video mux devices that are attached to a
		 * particular CFE instance. Obviously these cameras cannot be
		 * used simultaneously.
		 */
		unsigned int numCameras = 0;
		for (MediaEntity *entity : cfeDevice->entities()) {
			if (entity->function() != MEDIA_ENT_F_CAM_SENSOR)
				continue;

			const libpisp::PiSPVariant &variant =
				libpisp::get_variant(cfeDevice->hwRevision(),
						     ispDevice->hwRevision());
			if (!variant.NumFrontEnds() || !variant.NumBackEnds()) {
				LOG(RPI, Error) << "Unsupported PiSP variant";
				break;
			}

			std::unique_ptr<RPi::CameraData> cameraData =
				std::make_unique<PiSPCameraData>(this, variant);
			PiSPCameraData *pisp =
				static_cast<PiSPCameraData *>(cameraData.get());

			pisp->fe_ = SharedMemObject<FrontEnd>
					("pisp_frontend", true, pisp->pispVariant_);
			pisp->be_ = SharedMemObject<BackEnd>
					("pisp_backend", BackEnd::Config({}), pisp->pispVariant_);

			if (!pisp->fe_.fd().isValid() || !pisp->be_.fd().isValid()) {
				LOG(RPI, Error) << "Failed to create ISP shared objects";
				break;
			}

			int ret = registerCamera(cameraData, cfeDevice, "csi2",
						 ispDevice, entity);
			if (ret)
				LOG(RPI, Error) << "Failed to register camera "
						<< entity->name() << ": " << ret;
			else
				numCameras++;
		}

		if (numCameras)
			return true;
	}

	return false;
}

int PipelineHandlerPiSP::prepareBuffers(Camera *camera)
{
	PiSPCameraData *data = cameraData(camera);
	unsigned int numRawBuffers = 0;
	int ret;

	for (Stream *s : camera->streams()) {
		if (PipelineHandlerBase::isRaw(s->configuration().pixelFormat)) {
			numRawBuffers = s->configuration().bufferCount;
			break;
		}
	}

	/* Decide how many internal buffers to allocate. */
	for (auto const stream : data->streams_) {
		unsigned int numBuffers;
		/*
		 * For CFE, allocate a minimum of 4 buffers as we want
		 * to avoid any frame drops.
		 */
		constexpr unsigned int minBuffers = 4;
		if (stream == &data->cfe_[Cfe::Output0]) {
			/*
			 * If an application has configured a RAW stream, allocate
			 * additional buffers to make up the minimum, but ensure
			 * we have at least 2 sets of internal buffers to use to
			 * minimise frame drops.
			 */
			numBuffers = std::max<int>(2, minBuffers - numRawBuffers);
		} else if (stream == &data->isp_[Isp::Input]) {
			/*
			 * ISP input buffers are imported from the CFE, so follow
			 * similar logic as above to count all the RAW buffers
			 * available.
			 */
			numBuffers = numRawBuffers +
					std::max<int>(2, minBuffers - numRawBuffers);
		} else if (stream == &data->cfe_[Cfe::Embedded]) {
			/*
			 * Embedded data buffers are (currently) for internal use,
			 * so allocate a reasonably large amount.
			 */
			numBuffers = 12;
		} else if (stream == &data->cfe_[Cfe::Stats] ||
			   stream == &data->cfe_[Cfe::Config]) {
			numBuffers = data->config_.numCfeConfigStatsBuffers;
		} else if (!data->beEnabled_) {
			/* Backend not enabled, we don't need to allocate buffers. */
			numBuffers = 0;
		} else if (stream == &data->isp_[Isp::TdnOutput] && data->config_.disableTdn) {
			/* TDN is explicitly disabled. */
			continue;
		} else if (stream == &data->isp_[Isp::StitchOutput] && data->config_.disableHdr) {
			/* Stitch/HDR is explicitly disabled. */
			continue;
		} else {
			/* Allocate 2 sets of all other Backend buffers */
			numBuffers = 2;
		}

		LOG(RPI, Debug) << "Preparing " << numBuffers
				<< " buffers for stream " << stream->name();

		ret = stream->prepareBuffers(numBuffers);
		if (ret < 0)
			return ret;
	}

	/*
	 * Store the Framebuffer pointers for convenience as we will ping-pong
	 * these buffers between the input and output nodes for TDN and Stitch.
	 *
	 * The buffer size needs to be setup here as well. Conveniently this is
	 * the same for both TDN and stitch.
	 */
	pisp_image_format_config tdn;
	data->be_->GetTdnOutputFormat(tdn);
	unsigned int size = tdn.stride * tdn.height;
	for (auto const &buffer : data->isp_[Isp::TdnOutput].getBuffers()) {
		FrameBuffer *b = buffer.second.buffer;
		b->_d()->metadata().planes()[0].bytesused = size;
		data->tdnBuffers_.push_back(b);
	}
	for (auto const &buffer : data->isp_[Isp::StitchOutput].getBuffers()) {
		FrameBuffer *b = buffer.second.buffer;
		b->_d()->metadata().planes()[0].bytesused = size;
		data->stitchBuffers_.push_back(b);
	}

	/* Size up the config buffers as well. */
	for (auto &b : data->isp_[Isp::Config].getBuffers()) {
		FrameMetadata::Plane &plane = b.second.buffer->_d()->metadata().planes()[0];
		plane.bytesused = sizeof(pisp_be_tiles_config);
	}

	/*
	 * Pass the stats and embedded data buffers to the IPA. No other
	 * buffers need to be passed.
	 */
	mapBuffers(camera, data->cfe_[Cfe::Stats].getBuffers(), RPi::MaskStats);
	if (data->sensorMetadata_)
		mapBuffers(camera, data->cfe_[Cfe::Embedded].getBuffers(),
			   RPi::MaskEmbeddedData);

	return 0;
}

int PipelineHandlerPiSP::platformRegister(std::unique_ptr<RPi::CameraData> &cameraData,
					  MediaDevice *cfe, MediaDevice *isp)
{
	PiSPCameraData *data = static_cast<PiSPCameraData *>(cameraData.get());
	int ret;

	MediaEntity *cfeImage = cfe->getEntityByName("rp1-cfe-fe_image0");
	MediaEntity *cfeEmbedded = cfe->getEntityByName("rp1-cfe-embedded");
	MediaEntity *cfeStats = cfe->getEntityByName("rp1-cfe-fe_stats");
	MediaEntity *cfeConfig = cfe->getEntityByName("rp1-cfe-fe_config");
	MediaEntity *ispInput = isp->getEntityByName("pispbe-input");
	MediaEntity *IpaPrepare = isp->getEntityByName("pispbe-config");
	MediaEntity *ispOutput0 = isp->getEntityByName("pispbe-output0");
	MediaEntity *ispOutput1 = isp->getEntityByName("pispbe-output1");
	MediaEntity *ispTdnOutput = isp->getEntityByName("pispbe-tdn_output");
	MediaEntity *ispTdnInput = isp->getEntityByName("pispbe-tdn_input");
	MediaEntity *ispStitchOutput = isp->getEntityByName("pispbe-stitch_output");
	MediaEntity *ispStitchInput = isp->getEntityByName("pispbe-stitch_input");

	/* Locate and open the cfe video streams. */
	data->cfe_[Cfe::Output0] = RPi::Stream("CFE Image", cfeImage, StreamFlag::RequiresMmap);
	data->cfe_[Cfe::Embedded] = RPi::Stream("CFE Embedded", cfeEmbedded);
	data->cfe_[Cfe::Stats] = RPi::Stream("CFE Stats", cfeStats);
	data->cfe_[Cfe::Config] = RPi::Stream("CFE Config", cfeConfig,
					      StreamFlag::Recurrent | StreamFlag::RequiresMmap);

	/* Tag the ISP input stream as an import stream. */
	data->isp_[Isp::Input] =
		RPi::Stream("ISP Input", ispInput, StreamFlag::ImportOnly);
	data->isp_[Isp::Config] =
		RPi::Stream("ISP Config", IpaPrepare, StreamFlag::Recurrent |
						      StreamFlag::RequiresMmap);
	data->isp_[Isp::Output0] =
		RPi::Stream("ISP Output0", ispOutput0, StreamFlag::RequiresMmap);
	data->isp_[Isp::Output1] =
		RPi::Stream("ISP Output1", ispOutput1, StreamFlag::RequiresMmap);
	data->isp_[Isp::TdnOutput] =
		RPi::Stream("ISP TDN Output", ispTdnOutput, StreamFlag::Recurrent);
	data->isp_[Isp::TdnInput] =
		RPi::Stream("ISP TDN Input", ispTdnInput, StreamFlag::ImportOnly |
							  StreamFlag::Recurrent);
	data->isp_[Isp::StitchOutput] =
		RPi::Stream("ISP Stitch Output", ispStitchOutput, StreamFlag::Recurrent);
	data->isp_[Isp::StitchInput] =
		RPi::Stream("ISP Stitch Input", ispStitchInput, StreamFlag::ImportOnly |
								StreamFlag::Recurrent);

	/* Wire up all the buffer connections. */
	data->cfe_[Cfe::Output0].dev()->bufferReady.connect(data, &PiSPCameraData::cfeBufferDequeue);
	data->cfe_[Cfe::Stats].dev()->bufferReady.connect(data, &PiSPCameraData::cfeBufferDequeue);
	data->cfe_[Cfe::Config].dev()->bufferReady.connect(data, &PiSPCameraData::cfeBufferDequeue);
	data->isp_[Isp::Input].dev()->bufferReady.connect(data, &PiSPCameraData::beInputDequeue);
	data->isp_[Isp::Config].dev()->bufferReady.connect(data, &PiSPCameraData::beOutputDequeue);
	data->isp_[Isp::Output0].dev()->bufferReady.connect(data, &PiSPCameraData::beOutputDequeue);
	data->isp_[Isp::Output1].dev()->bufferReady.connect(data, &PiSPCameraData::beOutputDequeue);
	data->cfe_[Cfe::Embedded].dev()->bufferReady.connect(data, &PiSPCameraData::cfeBufferDequeue);

	data->csi2Subdev_ = std::make_unique<V4L2Subdevice>(cfe->getEntityByName("csi2"));
	data->feSubdev_ = std::make_unique<V4L2Subdevice>(cfe->getEntityByName("pisp-fe"));
	data->csi2Subdev_->open();
	data->feSubdev_->open();

	/*
	 * Open all CFE and ISP streams. The exception is the embedded data
	 * stream, which only gets opened below if the IPA reports that the sensor
	 * supports embedded data.
	 *
	 * The below grouping is just for convenience so that we can easily
	 * iterate over all streams in one go.
	 */
	data->streams_.push_back(&data->cfe_[Cfe::Output0]);
	data->streams_.push_back(&data->cfe_[Cfe::Config]);
	data->streams_.push_back(&data->cfe_[Cfe::Stats]);
	if (data->sensorMetadata_)
		data->streams_.push_back(&data->cfe_[Cfe::Embedded]);

	data->streams_.push_back(&data->isp_[Isp::Input]);
	data->streams_.push_back(&data->isp_[Isp::Output0]);
	data->streams_.push_back(&data->isp_[Isp::Output1]);
	data->streams_.push_back(&data->isp_[Isp::Config]);
	data->streams_.push_back(&data->isp_[Isp::TdnInput]);
	data->streams_.push_back(&data->isp_[Isp::TdnOutput]);
	data->streams_.push_back(&data->isp_[Isp::StitchInput]);
	data->streams_.push_back(&data->isp_[Isp::StitchOutput]);

	for (auto stream : data->streams_) {
		ret = stream->dev()->open();
		if (ret)
			return ret;
	}

	/* Write up all the IPA connections. */
	data->ipa_->prepareIspComplete.connect(data, &PiSPCameraData::prepareIspComplete);
	data->ipa_->processStatsComplete.connect(data, &PiSPCameraData::processStatsComplete);
	data->ipa_->setCameraTimeout.connect(data, &PiSPCameraData::setCameraTimeout);

	/*
	 * List the available streams an application may request. At present, we
	 * do not advertise CFE Embedded and ISP Statistics streams, as there
	 * is no mechanism for the application to request non-image buffer formats.
	 */
	std::set<Stream *> streams;
	streams.insert(&data->cfe_[Cfe::Output0]);
	streams.insert(&data->isp_[Isp::Output0]);
	streams.insert(&data->isp_[Isp::Output1]);

	/* Create and register the camera. */
	const std::string &id = data->sensor_->id();
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(cameraData), id, streams);
	PipelineHandler::registerCamera(std::move(camera));

	LOG(RPI, Info) << "Registered camera " << id
		       << " to CFE device " << cfe->deviceNode()
		       << " and ISP device " << isp->deviceNode()
		       << " using PiSP variant " << data->pispVariant_.Name();

	return 0;
}

CameraConfiguration::Status
PiSPCameraData::platformValidate(RPi::RPiCameraConfiguration *rpiConfig) const
{
	std::vector<StreamParams> &rawStreams = rpiConfig->rawStreams_;
	std::vector<StreamParams> &outStreams = rpiConfig->outStreams_;

	CameraConfiguration::Status status = CameraConfiguration::Status::Valid;

	/* Can only output 1 RAW stream and/or 2 YUV/RGB streams for now. */
	if (rawStreams.size() > 1 || outStreams.size() > 2) {
		LOG(RPI, Error) << "Invalid number of streams requested";
		return CameraConfiguration::Status::Invalid;
	}

	if (!rawStreams.empty()) {
		rawStreams[0].dev = cfe_[Cfe::Output0].dev();

		StreamConfiguration *rawStream = rawStreams[0].cfg;
		BayerFormat bayer = BayerFormat::fromPixelFormat(rawStream->pixelFormat);
		/*
		 * We cannot output CSI2 packed or non 16-bit output from the frontend,
		 * so signal the output as unpacked 16-bits in these cases.
		 */
		if (bayer.packing == BayerFormat::Packing::CSI2 || bayer.bitDepth != 16) {
			bayer.packing = (bayer.packing == BayerFormat::Packing::CSI2) ?
				BayerFormat::Packing::PISP1 : BayerFormat::Packing::None;
			bayer.bitDepth = 16;
		}

		/* The RAW stream size cannot exceed the sensor frame output - for now. */
		if (rawStream->size != rpiConfig->sensorFormat_.size ||
		    rawStream->pixelFormat != bayer.toPixelFormat()) {
			rawStream->size = rpiConfig->sensorFormat_.size;
			rawStream->pixelFormat = bayer.toPixelFormat();
			status = CameraConfiguration::Adjusted;
		}

		rawStreams[0].format =
			RPi::PipelineHandlerBase::toV4L2DeviceFormat(cfe_[Cfe::Output0].dev(), rawStream);

		computeOptimalStride(rawStreams[0].format);
	}

	/*
	 * For the two ISP outputs, the lower resolution must be routed from
	 * Output 1
	 *
	 * Index 0 contains the largest requested resolution.
	 */
	unsigned int largestWidth = getLargestWidth(rpiConfig->sensorFormat_,
						    rpiConfig->outStreams_);

	for (unsigned int i = 0; i < outStreams.size(); i++) {
		StreamConfiguration *cfg = outStreams[i].cfg;

		/*
		 * Output 1 must be for the smallest resolution. We will
		 * have that fixed up in the code above.
		 */
		auto ispOutput = i == 1 || outStreams.size() == 1 ? Isp::Output1
								  : Isp::Output0;
		outStreams[i].dev = isp_[ispOutput].dev();

		/*
		 * Don't let The output streams downscale by more than 64x when
		 * a downscaler block is available, or 16x when there's only the
		 * resampler.
		 */
		Size rawSize = rpiConfig->sensorFormat_.size.boundedToAspectRatio(cfg->size);
		unsigned int outputIndex = ispOutput == Isp::Output0 ? 0 : 1;
		Size minSize;
		if (pispVariant_.BackEndDownscalerAvailable(0, outputIndex)) {
			/*
			 * Downscaler available. Allow up to 64x downscale. If not a multiple of
			 * 64, round up to the next integer, but also ensure the result is even.
			 */
			const unsigned int downscale = 64;
			minSize.width = (rawSize.width + downscale - 1) / downscale;
			minSize.width = (minSize.width + 1) & ~1; /* ensure even */
			minSize.height = (rawSize.height + downscale - 1) / downscale;
			minSize.height = (minSize.height + 1) & ~1; /* ensure even */
		} else {
			/* No downscale. Resampler requires: (output_dim - 1) * 16 <= input_dim - 1 */
			const unsigned int downscale = 16;
			minSize.width = (rawSize.width - 1 + downscale - 1) / downscale + 1;
			minSize.width = (minSize.width + 1) & ~1; /* ensure even */
			minSize.height = (rawSize.height - 1 + downscale - 1) / downscale + 1;
			minSize.height = (minSize.height + 1) & ~1; /* ensure even */
		}
		LOG(RPI, Debug) << "minSize: width " << minSize.width << " height " << minSize.height;

		/* Bound the output size to minSize, preserve aspect ratio, and ensure even numbers. */
		if (cfg->size.width < minSize.width) {
			cfg->size.height = (cfg->size.height * minSize.width / cfg->size.width + 1) & ~1;
			cfg->size.width = minSize.width;
			status = CameraConfiguration::Status::Adjusted;
		}

		if (cfg->size.height < minSize.height) {
			cfg->size.width = (cfg->size.width * minSize.height / cfg->size.height + 1) & ~1;
			cfg->size.height = minSize.height;
			status = CameraConfiguration::Status::Adjusted;
		}

		/* Make sure output1 is no larger than output 0. */
		Size size = cfg->size.boundedTo(outStreams[0].cfg->size);

		/* \todo Warn if upscaling: reduces image quality. */

		if (cfg->size != size) {
			cfg->size = size;
			status = CameraConfiguration::Status::Adjusted;
		}

		outStreams[i].format =
			RPi::PipelineHandlerBase::toV4L2DeviceFormat(outStreams[i].dev, outStreams[i].cfg);

		/* Compute the optimal stride for the BE output buffers. */
		computeOptimalStride(outStreams[i].format);

		/*
		 * We need to check for software downscaling. This must happen
		 * after adjusting the device format so that we can choose the
		 * largest stride - which might have been the original
		 * unadjusted format, or the adjusted one (if software
		 * downscaling means it's larger).
		 */
		V4L2DeviceFormat adjustedFormat = outStreams[i].format;
		adjustDeviceFormat(adjustedFormat);

		unsigned int swDownscale =
			calculateSwDownscale(adjustedFormat, largestWidth,
					     be_->GetMaxDownscale());
		LOG(RPI, Debug) << "For stream " << adjustedFormat
				<< " swDownscale is " << swDownscale;
		if (swDownscale > 1) {
			adjustedFormat.size.width *= swDownscale;
			computeOptimalStride(adjustedFormat);
			for (unsigned int p = 0; p < outStreams[i].format.planesCount; p++)
				outStreams[i].format.planes[p].bpl =
					std::max(outStreams[i].format.planes[p].bpl, adjustedFormat.planes[p].bpl);
		}
	}

	return status;
}

int PiSPCameraData::platformPipelineConfigure(const std::unique_ptr<YamlObject> &root)
{
	config_ = {
		.numCfeConfigStatsBuffers = 12,
		.numCfeConfigQueue = 2,
		.disableTdn = false,
		.disableHdr = false,
	};

	if (!root)
		return 0;

	std::optional<double> ver = (*root)["version"].get<double>();
	if (!ver || *ver != 1.0) {
		LOG(RPI, Error) << "Unexpected configuration file version reported";
		return -EINVAL;
	}

	std::optional<std::string> target = (*root)["target"].get<std::string>();
	if (!target || *target != "pisp") {
		LOG(RPI, Error) << "Unexpected target reported: expected \"pisp\", got "
				<< *target;
		return -EINVAL;
	}

	const YamlObject &phConfig = (*root)["pipeline_handler"];
	config_.numCfeConfigStatsBuffers =
		phConfig["num_cfe_config_stats_buffers"].get<unsigned int>(config_.numCfeConfigStatsBuffers);
	config_.numCfeConfigQueue =
		phConfig["num_cfe_config_queue"].get<unsigned int>(config_.numCfeConfigQueue);
	config_.disableTdn = phConfig["disable_tdn"].get<bool>(config_.disableTdn);
	config_.disableHdr = phConfig["disable_hdr"].get<bool>(config_.disableHdr);

	if (config_.disableTdn) {
		LOG(RPI, Info) << "TDN disabled by user config";
		streams_.erase(std::remove_if(streams_.begin(), streams_.end(),
			       [this] (const RPi::Stream *s) { return s == &isp_[Isp::TdnInput] ||
								      s == &isp_[Isp::TdnInput]; }),
			       streams_.end());
	}

	if (config_.disableHdr) {
		LOG(RPI, Info) << "HDR disabled by user config";
		streams_.erase(std::remove_if(streams_.begin(), streams_.end(),
			       [this] (const RPi::Stream *s) { return s == &isp_[Isp::StitchInput] ||
								      s == &isp_[Isp::StitchOutput]; }),
			       streams_.end());
	}

	if (config_.numCfeConfigStatsBuffers < 1) {
		LOG(RPI, Error)
			<< "Invalid configuration: num_cfe_config_stats_buffers must be >= 1";
		return -EINVAL;
	}

	if (config_.numCfeConfigQueue < 1) {
		LOG(RPI, Error)
			<< "Invalid configuration: numCfeConfigQueue must be >= 1";
		return -EINVAL;
	}

	return 0;
}

std::unordered_map<uint32_t, uint32_t> deviceAdjustTable = {
	{ V4L2_PIX_FMT_RGBX32, V4L2_PIX_FMT_RGB24 },
	{ V4L2_PIX_FMT_XBGR32, V4L2_PIX_FMT_BGR24 }
};

bool PiSPCameraData::adjustDeviceFormat(V4L2DeviceFormat &format) const
{
	auto it = deviceAdjustTable.find(format.fourcc.fourcc());

	if (pispVariant_.BackendRGB32Supported(0))
		return false;

	if (it != deviceAdjustTable.end()) {
		LOG(RPI, Debug) << "Swapping 32-bit for 24-bit format";
		format.fourcc = V4L2PixelFormat(it->second);
		return true;
	}

	return false;
}

int PiSPCameraData::platformConfigure(const RPi::RPiCameraConfiguration *rpiConfig)
{
	const std::vector<RPi::RPiCameraConfiguration::StreamParams> &rawStreams = rpiConfig->rawStreams_;
	const std::vector<RPi::RPiCameraConfiguration::StreamParams> &outStreams = rpiConfig->outStreams_;
	int ret;
	V4L2VideoDevice *cfe = cfe_[Cfe::Output0].dev();
	V4L2DeviceFormat cfeFormat;

	/*
	 * See which streams are requested, and route the user
	 * StreamConfiguration appropriately.
	 */
	if (rawStreams.empty()) {
		/*
		 * The CFE Frontend output will always be 16-bits unpacked, so adjust the
		 * mbus code right at the start.
		 */
		V4L2SubdeviceFormat sensorFormatMod = rpiConfig->sensorFormat_;
		sensorFormatMod.code = mbusCodeUnpacked16(sensorFormatMod.code);
		cfeFormat = RPi::PipelineHandlerBase::toV4L2DeviceFormat(cfe,
									 sensorFormatMod,
									 BayerFormat::Packing::PISP1);
		computeOptimalStride(cfeFormat);
	} else {
		rawStreams[0].cfg->setStream(&cfe_[Cfe::Output0]);
		cfe_[Cfe::Output0].setFlags(StreamFlag::External);
		cfeFormat = rawStreams[0].format;
	}

	/*
	 * If the sensor output is 16-bits, we must endian swap the buffer
	 * contents to account for the HW missing this feature.
	 */
	cfe_[Cfe::Output0].clearFlags(StreamFlag::Needs16bitEndianSwap);
	if (MediaBusFormatInfo::info(rpiConfig->sensorFormat_.code).bitsPerPixel == 16) {
		cfe_[Cfe::Output0].setFlags(StreamFlag::Needs16bitEndianSwap);
		LOG(RPI, Warning)
			<< "The sensor is configured for a 16-bit output, statistics"
			<< "  will not be correct. You must use manual camera settings.";
	}

	/* Ditto for the 14-bit unpacking. */
	cfe_[Cfe::Output0].clearFlags(StreamFlag::Needs14bitUnpack);
	if (MediaBusFormatInfo::info(rpiConfig->sensorFormat_.code).bitsPerPixel == 14) {
		cfe_[Cfe::Output0].setFlags(StreamFlag::Needs14bitUnpack);
		LOG(RPI, Warning)
			<< "The sensor is configured for a 14-bit output, statistics"
			<< "  will not be correct. You must use manual camera settings.";
	}

	ret = cfe->setFormat(&cfeFormat);
	if (ret)
		return ret;

	/* Set the TDN and Stitch node formats in case they are turned on. */
	isp_[Isp::TdnOutput].dev()->setFormat(&cfeFormat);
	isp_[Isp::TdnInput].dev()->setFormat(&cfeFormat);
	isp_[Isp::StitchOutput].dev()->setFormat(&cfeFormat);
	isp_[Isp::StitchInput].dev()->setFormat(&cfeFormat);

	ret = isp_[Isp::Input].dev()->setFormat(&cfeFormat);
	if (ret)
		return ret;

	LOG(RPI, Info) << "Sensor: " << sensor_->id()
		       << " - Selected sensor format: " << rpiConfig->sensorFormat_
		       << " - Selected CFE format: " << cfeFormat;

	/*
	 * Find the largest width of any stream; we'll use it later to check for
	 * excessive downscaling.
	 */
	unsigned int largestWidth = getLargestWidth(rpiConfig->sensorFormat_, outStreams);

	unsigned int beEnables = 0;
	V4L2DeviceFormat format;

	/*
	 * First thing is to remove Isp::Output0 and Isp::Output1 from streams_
	 * as they may be unused depending on the configuration. Add them back
	 * only if needed.
	 */
	streams_.erase(std::remove_if(streams_.begin(), streams_.end(),
		       [this] (const RPi::Stream *s) { return s == &isp_[Isp::Output0] ||
							      s == &isp_[Isp::Output1]; }),
		       streams_.end());

	cropParams_.clear();
	for (unsigned int i = 0; i < outStreams.size(); i++) {
		StreamConfiguration *cfg = outStreams[i].cfg;
		unsigned int ispIndex;

		/*
		 * Output 1 must be for the smallest resolution. We will
		 * have that fixed up in the code above.
		 */
		RPi::Stream *stream;
		if (i == 1 || outStreams.size() == 1) {
			stream = &isp_[Isp::Output1];
			beEnables |= PISP_BE_RGB_ENABLE_OUTPUT1;
			ispIndex = 1;
		} else {
			stream = &isp_[Isp::Output0];
			beEnables |= PISP_BE_RGB_ENABLE_OUTPUT0;
			ispIndex = 0;
		}

		format = outStreams[i].format;
		bool needs32BitConversion = adjustDeviceFormat(format);

		/*
		 * This pixel format may not be the same as the configured
		 * pixel format if adjustDeviceFormat() above has reqused a change.
		 */
		PixelFormat pixFmt = format.fourcc.toPixelFormat();

		/* If there's excessive downscaling we'll do some of it in software. */
		unsigned int swDownscale = calculateSwDownscale(format, largestWidth,
								be_->GetMaxDownscale());
		unsigned int hwWidth = format.size.width * swDownscale;
		format.size.width = hwWidth;

		LOG(RPI, Debug) << "Setting " << stream->name() << " to "
				<< format << " (sw downscale " << swDownscale << ")";

		ret = stream->dev()->setFormat(&format);
		if (ret)
			return -EINVAL;
		LOG(RPI, Debug) << "After setFormat, stride " << format.planes[0].bpl;

		if (format.size.height != cfg->size.height ||
		    format.size.width != hwWidth || format.fourcc.toPixelFormat() != pixFmt) {
			LOG(RPI, Error)
				<< "Failed to set requested format on " << stream->name()
				<< ", returned " << format;
			return -EINVAL;
		}

		LOG(RPI, Debug)
			<< "Stream " << stream->name() << " has color space "
			<< ColorSpace::toString(cfg->colorSpace);

		libcamera::RPi::Stream::StreamFlags flags = StreamFlag::External;

		stream->clearFlags(StreamFlag::Needs32bitConv);
		if (needs32BitConversion)
			flags |= StreamFlag::Needs32bitConv;

		/* Set smallest selection the ISP will allow. */
		Size minCrop{ 32, 32 };

		/* Adjust aspect ratio by providing crops on the input image. */
		Size size = cfeFormat.size.boundedToAspectRatio(outStreams[i].cfg->size);
		Rectangle ispCrop = size.centeredTo(Rectangle(cfeFormat.size).center());

		/*
		 * Calculate the minimum crop. The rule is that (output_dim - 1) / (input_dim - 1)
		 * must be strictly < 16. We add 2 after dividing because +1
		 * comes from the division that rounds down, and +1 because we
		 * had (input_dim - 1).
		 */
		Size scalingMinSize = outStreams[i].cfg->size.shrunkBy({ 1, 1 }) / 16;
		scalingMinSize.growBy({ 2, 2 });
		minCrop.expandTo(scalingMinSize);

		platformSetIspCrop(ispIndex, ispCrop);
		/*
		 * Set the scaler crop to the value we are using (scaled to native sensor
		 * coordinates).
		 */
		cropParams_.emplace(std::piecewise_construct,
				    std::forward_as_tuple(outStreams[i].index),
				    std::forward_as_tuple(ispCrop, minCrop, ispIndex));

		cfg->setStream(stream);
		stream->setFlags(flags);
		stream->setSwDownscale(swDownscale);
		streams_.push_back(stream);
	}

	pisp_be_global_config global;
	be_->GetGlobal(global);
	global.rgb_enables &= ~(PISP_BE_RGB_ENABLE_OUTPUT0 + PISP_BE_RGB_ENABLE_OUTPUT1);
	global.rgb_enables |= beEnables;
	be_->SetGlobal(global);

	beEnabled_ = beEnables & (PISP_BE_RGB_ENABLE_OUTPUT0 | PISP_BE_RGB_ENABLE_OUTPUT1);

	/* CFE statistics output format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_RPI_FE_STATS);
	ret = cfe_[Cfe::Stats].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on CFE stats stream: "
				<< format.toString();
		return ret;
	}

	/* CFE config format. */
	format = {};
	format.fourcc = V4L2PixelFormat(V4L2_META_FMT_RPI_FE_CFG);
	ret = cfe_[Cfe::Config].dev()->setFormat(&format);
	if (ret) {
		LOG(RPI, Error) << "Failed to set format on CFE config stream: "
				<< format.toString();
		return ret;
	}

	/*
	 * Configure the CFE embedded data output format only if the sensor
	 * supports it.
	 */
	V4L2SubdeviceFormat embeddedFormat;
	if (sensorMetadata_) {
		sensor_->device()->getFormat(1, &embeddedFormat);
		format = {};
		format.fourcc = V4L2PixelFormat(V4L2_META_FMT_SENSOR_DATA);
		format.planes[0].size = embeddedFormat.size.width * embeddedFormat.size.height;

		LOG(RPI, Debug) << "Setting embedded data format " << format.toString();
		ret = cfe_[Cfe::Embedded].dev()->setFormat(&format);
		if (ret) {
			LOG(RPI, Error) << "Failed to set format on CFE embedded: "
					<< format;
			return ret;
		}
	}

	configureEntities(rpiConfig->sensorFormat_, embeddedFormat);
	configureCfe();

	if (beEnabled_)
		configureBe(rpiConfig->yuvColorSpace_);

	return 0;
}

void PiSPCameraData::platformStart()
{
	/*
	 * We don't need to worry about dequeue events for the TDN and Stitch
	 * nodes as the buffers are simply ping-ponged every frame. But we do
	 * want to track the currently used input index.
	 */
	tdnInputIndex_ = 0;
	stitchInputIndex_ = 0;

	cfeJobQueue_ = {};

	for (unsigned int i = 0; i < config_.numCfeConfigQueue; i++)
		prepareCfe();

	/* Clear the debug dump file history. */
	last_dump_file_.clear();
}

void PiSPCameraData::platformStop()
{
	cfeJobQueue_ = {};
}

void PiSPCameraData::platformFreeBuffers()
{
	tdnBuffers_.clear();
	stitchBuffers_.clear();
}

void PiSPCameraData::cfeBufferDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	if (!isRunning())
		return;

	for (RPi::Stream &s : cfe_) {
		index = s.getBufferId(buffer);
		if (index) {
			stream = &s;
			break;
		}
	}

	/* If the last CFE job has completed, we need a new job entry in the queue. */
	if (cfeJobQueue_.empty() || cfeJobComplete())
		cfeJobQueue_.push({});

	CfeJob &job = cfeJobQueue_.back();

	/* The buffer must belong to one of our streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer dequeue"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	job.buffers[stream] = buffer;

	if (stream == &cfe_[Cfe::Output0]) {
		/* Do an endian swap or 14-bit unpacking if needed. */
		if (stream->getFlags() & StreamFlag::Needs16bitEndianSwap ||
		    stream->getFlags() & StreamFlag::Needs14bitUnpack) {
			const unsigned int stride = stream->configuration().stride;
			const unsigned int width = stream->configuration().size.width;
			const unsigned int height = stream->configuration().size.height;
			const RPi::BufferObject &b = stream->getBuffer(index);

			ASSERT(b.mapped);
			void *mem = b.mapped->planes()[0].data();

			dmabufSyncStart(buffer->planes()[0].fd);
			if (stream->getFlags() & StreamFlag::Needs16bitEndianSwap)
				do16BitEndianSwap(mem, width, height, stride);
			else
				do14bitUnpack(mem, width, height, stride);
			dmabufSyncEnd(buffer->planes()[0].fd);
		}

		/*
		 * Lookup the sensor controls used for this frame sequence from
		 * DelayedControl and queue them along with the frame buffer.
		 */
		auto [ctrl, delayContext] = delayedCtrls_->get(buffer->metadata().sequence);
		/*
		 * Add the frame timestamp to the ControlList for the IPA to use
		 * as it does not receive the FrameBuffer object. Also derive a
		 * corresponding wallclock value.
		 */
		wallClockRecovery_.addSample();
		uint64_t sensorTimestamp = buffer->metadata().timestamp;
		uint64_t wallClockTimestamp = wallClockRecovery_.getOutput(sensorTimestamp / 1000);

		ctrl.set(controls::SensorTimestamp, sensorTimestamp);
		ctrl.set(controls::FrameWallClock, wallClockTimestamp);
		job.sensorControls = std::move(ctrl);
		job.delayContext = delayContext;
	} else if (stream == &cfe_[Cfe::Config]) {
		/* The config buffer can be re-queued back straight away. */
		handleStreamBuffer(buffer, &cfe_[Cfe::Config]);
		prepareCfe();
	}

	handleState();
}

void PiSPCameraData::beInputDequeue(FrameBuffer *buffer)
{
	if (!isRunning())
		return;

	LOG(RPI, Debug) << "Stream ISP Input buffer complete"
			<< ", buffer id " << cfe_[Cfe::Output0].getBufferId(buffer)
			<< ", timestamp: " << buffer->metadata().timestamp;

	/* The ISP input buffer gets re-queued into CFE. */
	handleStreamBuffer(buffer, &cfe_[Cfe::Output0]);
	handleState();
}

void PiSPCameraData::beOutputDequeue(FrameBuffer *buffer)
{
	RPi::Stream *stream = nullptr;
	int index;

	if (!isRunning())
		return;

	for (RPi::Stream &s : isp_) {
		index = s.getBufferId(buffer);
		if (index) {
			stream = &s;
			break;
		}
	}

	/* The buffer must belong to one of our ISP output streams. */
	ASSERT(stream);

	LOG(RPI, Debug) << "Stream " << stream->name() << " buffer complete"
			<< ", buffer id " << index
			<< ", timestamp: " << buffer->metadata().timestamp;

	bool downscale = stream->swDownscale() > 1;
	bool needs32bitConv = !!(stream->getFlags() & StreamFlag::Needs32bitConv);

	if (downscale || needs32bitConv)
		dmabufSyncStart(buffer->planes()[0].fd);

	if (downscale) {
		/* Further software downscaling must be applied. */
		downscaleStreamBuffer(stream, index);
	}

	/* Convert 24bpp outputs to 32bpp outputs where necessary. */
	if (needs32bitConv) {
		unsigned int stride = stream->configuration().stride;
		unsigned int width = stream->configuration().size.width;
		unsigned int height = stream->configuration().size.height;

		const RPi::BufferObject &b = stream->getBuffer(index);

		ASSERT(b.mapped);
		void *mem = b.mapped->planes()[0].data();
		do32BitConversion(mem, width, height, stride);
	}

	if (downscale || needs32bitConv)
		dmabufSyncEnd(buffer->planes()[0].fd);

	handleStreamBuffer(buffer, stream);
	handleState();
}

void PiSPCameraData::processStatsComplete(const ipa::RPi::BufferIds &buffers)
{
	if (!isRunning())
		return;

	handleStreamBuffer(cfe_[Cfe::Stats].getBuffers().at(buffers.stats & RPi::MaskID).buffer,
			   &cfe_[Cfe::Stats]);
}

void PiSPCameraData::setCameraTimeout(uint32_t maxFrameLengthMs)
{
	/*
	 * Set the dequeue timeout to the larger of 5x the maximum reported
	 * frame length advertised by the IPA over a number of frames. Allow
	 * a minimum timeout value of 1s.
	 */
	utils::Duration timeout =
		std::max<utils::Duration>(1s, 5 * maxFrameLengthMs * 1ms);

	LOG(RPI, Debug) << "Setting CFE timeout to " << timeout;
	cfe_[Cfe::Output0].dev()->setDequeueTimeout(timeout);
}

void PiSPCameraData::prepareIspComplete(const ipa::RPi::BufferIds &buffers, bool stitchSwapBuffers)
{
	unsigned int embeddedId = buffers.embedded & RPi::MaskID;
	unsigned int bayerId = buffers.bayer & RPi::MaskID;
	FrameBuffer *buffer;

	if (!isRunning())
		return;

	if (sensorMetadata_ && embeddedId) {
		buffer = cfe_[Cfe::Embedded].getBuffers().at(embeddedId).buffer;
		handleStreamBuffer(buffer, &cfe_[Cfe::Embedded]);
	}

	if (!beEnabled_) {
		/*
		 * If there is no need to run the Backend, just signal that the
		 * input buffer is completed and all Backend outputs are ready.
		 */
		buffer = cfe_[Cfe::Output0].getBuffers().at(bayerId).buffer;
		handleStreamBuffer(buffer, &cfe_[Cfe::Output0]);
	} else
		prepareBe(bayerId, stitchSwapBuffers);

	state_ = State::IpaComplete;
	handleState();
}

int PiSPCameraData::configureCfe()
{
	V4L2DeviceFormat cfeFormat;
	cfe_[Cfe::Output0].dev()->getFormat(&cfeFormat);

	std::scoped_lock<FrontEnd> l(*fe_);

	pisp_fe_global_config global;
	fe_->GetGlobal(global);
	global.enables &= ~(PISP_FE_ENABLE_COMPRESS0 | PISP_FE_ENABLE_DECIMATE |
			    PISP_FE_ENABLE_STATS_CROP);

	global.enables |= PISP_FE_ENABLE_OUTPUT0;
	global.bayer_order = toPiSPBayerOrder(cfeFormat.fourcc);

	pisp_image_format_config image = toPiSPImageFormat(cfeFormat);
	pisp_fe_input_config input = {};

	/* 14-bit bodge */
	if (cfe_[Cfe::Output0].getFlags() & StreamFlag::Needs14bitUnpack)
		image.width = image.width * 14 / 16;

	input.streaming = 1;
	input.format = image;
	input.format.format = PISP_IMAGE_FORMAT_BPS_16;

	if (PISP_IMAGE_FORMAT_COMPRESSED(image.format)) {
		pisp_compress_config compress;
		compress.offset = DefaultCompressionOffset;
		compress.mode =	(image.format & PISP_IMAGE_FORMAT_COMPRESSION_MASK) /
					PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
		global.enables |= PISP_FE_ENABLE_COMPRESS0;
		fe_->SetCompress(0, compress);
	}

	const unsigned int maxStatsWidth = pispVariant_.FrontEndDownscalerMaxWidth(0, 0);
	if (input.format.width > maxStatsWidth) {
		/* The line is too wide for the stats generation, so try to decimate. */
		global.enables |= PISP_FE_ENABLE_DECIMATE;
		if ((input.format.width >> 1) > maxStatsWidth) {
			/* Still too wide, crop a window before the decimation. */
			pisp_fe_crop_config statsCrop{};
			statsCrop.width = maxStatsWidth << 1;
			statsCrop.height = input.format.height;
			statsCrop.offset_x =
				(((input.format.width - statsCrop.width) >> 1) & ~1);
			statsCrop.offset_y = 0;
			fe_->SetStatsCrop(statsCrop);
			global.enables |= PISP_FE_ENABLE_STATS_CROP;
			LOG(RPI, Warning) << "Cropping FE stats window";
		}
	}

	fe_->SetGlobal(global);
	fe_->SetInput(input);
	fe_->SetOutputFormat(0, image);

	return 0;
}

bool PiSPCameraData::calculateCscConfiguration(const V4L2DeviceFormat &v4l2Format, pisp_be_ccm_config &csc)
{
	const PixelFormat &pixFormat = v4l2Format.fourcc.toPixelFormat();
	const PixelFormatInfo &info = PixelFormatInfo::info(pixFormat);
	memset(&csc, 0, sizeof(csc));

	if (info.colourEncoding == PixelFormatInfo::ColourEncodingYUV) {
		/* Look up the correct YCbCr conversion matrix for this colour space. */
		if (v4l2Format.colorSpace == ColorSpace::Sycc)
			be_->InitialiseYcbcr(csc, "jpeg");
		else if (v4l2Format.colorSpace == ColorSpace::Smpte170m)
			be_->InitialiseYcbcr(csc, "smpte170m");
		else if (v4l2Format.colorSpace == ColorSpace::Rec709)
			be_->InitialiseYcbcr(csc, "rec709");
		else {
			LOG(RPI, Warning)
				<< "Unrecognised colour space "
				<< ColorSpace::toString(v4l2Format.colorSpace)
				<< ", defaulting to sYCC";
			be_->InitialiseYcbcr(csc, "jpeg");
		}
		return true;
	}
	/* There will be more formats to check for in due course. */
	else if (pixFormat == formats::RGB888 || pixFormat == formats::RGBX8888 ||
		 pixFormat == formats::XRGB8888 || pixFormat == formats::RGB161616) {
		/* Identity matrix but with RB colour swap. */
		csc.coeffs[2] = csc.coeffs[4] = csc.coeffs[6] = 1 << 10;
		return true;
	}

	return false;
}

int PiSPCameraData::configureBe(const std::optional<ColorSpace> &yuvColorSpace)
{
	pisp_image_format_config inputFormat;
	V4L2DeviceFormat cfeFormat;

	isp_[Isp::Input].dev()->getFormat(&cfeFormat);
	inputFormat = toPiSPImageFormat(cfeFormat);

	pisp_be_global_config global;
	be_->GetGlobal(global);
	global.bayer_enables &= ~(PISP_BE_BAYER_ENABLE_DECOMPRESS +
				  PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS +
				  PISP_BE_BAYER_ENABLE_TDN_COMPRESS +
				  PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS +
				  PISP_BE_BAYER_ENABLE_STITCH_COMPRESS);
	global.rgb_enables &= ~(PISP_BE_RGB_ENABLE_RESAMPLE0 +
				PISP_BE_RGB_ENABLE_RESAMPLE1 +
				PISP_BE_RGB_ENABLE_DOWNSCALE0 +
				PISP_BE_RGB_ENABLE_DOWNSCALE1 +
				PISP_BE_RGB_ENABLE_CSC0 +
				PISP_BE_RGB_ENABLE_CSC1);

	global.bayer_enables |= PISP_BE_BAYER_ENABLE_INPUT;
	global.bayer_order = toPiSPBayerOrder(cfeFormat.fourcc);

	if (PISP_IMAGE_FORMAT_COMPRESSED(inputFormat.format)) {
		pisp_decompress_config decompress;
		decompress.offset = DefaultCompressionOffset;
		decompress.mode = (inputFormat.format & PISP_IMAGE_FORMAT_COMPRESSION_MASK)
					/ PISP_IMAGE_FORMAT_COMPRESSION_MODE_1;
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_DECOMPRESS;
		be_->SetDecompress(decompress);
	}

	if (global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT0) {
		pisp_be_output_format_config outputFormat0 = {};
		V4L2DeviceFormat ispFormat0 = {};

		isp_[Isp::Output0].dev()->getFormat(&ispFormat0);
		outputFormat0.image = toPiSPImageFormat(ispFormat0);

		pisp_be_ccm_config csc;
		if (calculateCscConfiguration(ispFormat0, csc)) {
			global.rgb_enables |= PISP_BE_RGB_ENABLE_CSC0;
			be_->SetCsc(0, csc);
		}

		BackEnd::SmartResize resize = {};
		resize.width = ispFormat0.size.width;
		resize.height = ispFormat0.size.height;
		be_->SetSmartResize(0, resize);

		setupOutputClipping(ispFormat0, outputFormat0);

		be_->SetOutputFormat(0, outputFormat0);
	}

	if (global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1) {
		pisp_be_output_format_config outputFormat1 = {};
		V4L2DeviceFormat ispFormat1 = {};

		isp_[Isp::Output1].dev()->getFormat(&ispFormat1);
		outputFormat1.image = toPiSPImageFormat(ispFormat1);

		pisp_be_ccm_config csc;
		if (calculateCscConfiguration(ispFormat1, csc)) {
			global.rgb_enables |= PISP_BE_RGB_ENABLE_CSC1;
			be_->SetCsc(1, csc);
		}

		BackEnd::SmartResize resize = {};
		resize.width = ispFormat1.size.width;
		resize.height = ispFormat1.size.height;
		be_->SetSmartResize(1, resize);

		setupOutputClipping(ispFormat1, outputFormat1);

		be_->SetOutputFormat(1, outputFormat1);
	}

	/* Setup the TDN I/O blocks in case TDN gets turned on later. */
	V4L2DeviceFormat tdnV4L2Format;
	isp_[Isp::TdnOutput].dev()->getFormat(&tdnV4L2Format);
	pisp_image_format_config tdnFormat = toPiSPImageFormat(tdnV4L2Format);
	be_->SetTdnOutputFormat(tdnFormat);
	be_->SetTdnInputFormat(tdnFormat);

	if (PISP_IMAGE_FORMAT_COMPRESSED(tdnFormat.format)) {
		pisp_decompress_config tdnDecompress;
		pisp_compress_config tdnCompress;

		tdnDecompress.offset = tdnCompress.offset = DefaultCompressionOffset;
		tdnDecompress.mode = tdnCompress.mode = DefaultCompressionMode;
		be_->SetTdnDecompress(tdnDecompress);
		be_->SetTdnCompress(tdnCompress);
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS +
					PISP_BE_BAYER_ENABLE_TDN_COMPRESS;
	}

	/* Likewise for the Stitch block. */
	V4L2DeviceFormat stitchV4L2Format;
	isp_[Isp::StitchOutput].dev()->getFormat(&stitchV4L2Format);
	pisp_image_format_config stitchFormat = toPiSPImageFormat(stitchV4L2Format);
	be_->SetStitchOutputFormat(stitchFormat);
	be_->SetStitchInputFormat(stitchFormat);

	if (PISP_IMAGE_FORMAT_COMPRESSED(stitchFormat.format)) {
		pisp_decompress_config stitchDecompress;
		pisp_compress_config stitchCompress;

		/* Stitch block is after BLC, so compression offset should be 0. */
		stitchDecompress.offset = stitchCompress.offset = 0;
		stitchDecompress.mode = stitchCompress.mode = DefaultCompressionMode;
		be_->SetStitchDecompress(stitchDecompress);
		be_->SetStitchCompress(stitchCompress);
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS +
					PISP_BE_BAYER_ENABLE_STITCH_COMPRESS;
	}

	/*
	 * For the bit of the pipeline where we go temporarily into YCbCr, we'll use the
	 * same flavour of YCbCr as dictated by the headline colour space. But there's
	 * no benefit from compressing and shifting the range, so we'll stick with the
	 * full range version of whatever that colour space is.
	 */
	if (yuvColorSpace) {
		pisp_be_ccm_config ccm;
		if (yuvColorSpace == ColorSpace::Sycc) {
			be_->InitialiseYcbcr(ccm, "jpeg");
			be_->SetYcbcr(ccm);
			be_->InitialiseYcbcrInverse(ccm, "jpeg");
			be_->SetYcbcrInverse(ccm);
		} else if (yuvColorSpace == ColorSpace::Smpte170m) {
			/* We want the full range version of smpte170m, aka. jpeg */
			be_->InitialiseYcbcr(ccm, "jpeg");
			be_->SetYcbcr(ccm);
			be_->InitialiseYcbcrInverse(ccm, "jpeg");
			be_->SetYcbcrInverse(ccm);
		} else if (yuvColorSpace == ColorSpace::Rec709) {
			be_->InitialiseYcbcr(ccm, "rec709_full");
			be_->SetYcbcr(ccm);
			be_->InitialiseYcbcrInverse(ccm, "rec709_full");
			be_->SetYcbcrInverse(ccm);
		} else {
			/* Validation should have ensured this can't happen. */
			LOG(RPI, Error)
				<< "Invalid colour space "
				<< ColorSpace::toString(yuvColorSpace);
			ASSERT(0);
		}
	} else {
		/* Again, validation should have prevented this. */
		LOG(RPI, Error) << "No YUV colour space";
		ASSERT(0);
	}

	be_->SetGlobal(global);
	be_->SetInputFormat(inputFormat);

	return 0;
}

void PiSPCameraData::platformSetIspCrop(unsigned int index, const Rectangle &ispCrop)
{
	pisp_be_crop_config beCrop = {
		static_cast<uint16_t>(ispCrop.x),
		static_cast<uint16_t>(ispCrop.y),
		static_cast<uint16_t>(ispCrop.width),
		static_cast<uint16_t>(ispCrop.height)
	};

	LOG(RPI, Debug) << "Output " << index << " " << ispCrop.toString();
	be_->SetCrop(index, beCrop);
}

int PiSPCameraData::platformInitIpa(ipa::RPi::InitParams &params)
{
	params.fe = fe_.fd();
	params.be = be_.fd();
	return 0;
}

int PiSPCameraData::configureEntities(V4L2SubdeviceFormat sensorFormat,
				      V4L2SubdeviceFormat &embeddedFormat)
{
	int ret = 0;

	constexpr unsigned int csiVideoSinkPad = 0;
	constexpr unsigned int csiMetaSinkPad = 1;
	constexpr unsigned int csiVideoSourcePad = 4;
	constexpr unsigned int csiMetaSourcePad = 5;

	constexpr unsigned int feVideoSinkPad = 0;
	constexpr unsigned int feConfigSinkPad = 1;
	constexpr unsigned int feVideo0SourcePad = 2;
	constexpr unsigned int feVideo1SourcePad = 3;
	constexpr unsigned int feStatsSourcePad = 4;

	const MediaEntity *csi2 = csi2Subdev_->entity();
	const MediaEntity *fe = feSubdev_->entity();

	for (MediaLink *link : csi2->pads()[csiVideoSourcePad]->links()) {
		if (link->sink()->entity()->name() == "rp1-cfe-csi2_ch0")
			link->setEnabled(false);
		else if (link->sink()->entity()->name() == "pisp-fe")
			link->setEnabled(true);
	}

	csi2->pads()[csiMetaSourcePad]->links()[0]->setEnabled(sensorMetadata_);

	fe->pads()[feConfigSinkPad]->links()[0]->setEnabled(true);
	fe->pads()[feVideo0SourcePad]->links()[0]->setEnabled(true);
	fe->pads()[feVideo1SourcePad]->links()[0]->setEnabled(false);
	fe->pads()[feStatsSourcePad]->links()[0]->setEnabled(true);

	ret = csi2Subdev_->setFormat(csiVideoSinkPad, &sensorFormat);
	if (ret)
		return ret;

	if (sensorMetadata_) {
		ret = csi2Subdev_->setFormat(csiMetaSinkPad, &embeddedFormat);
		if (ret)
			return ret;

		ret = csi2Subdev_->setFormat(csiMetaSourcePad, &embeddedFormat);
		if (ret)
			return ret;
	}

	V4L2SubdeviceFormat feFormat = sensorFormat;
	feFormat.code = mbusCodeUnpacked16(sensorFormat.code);
	ret = feSubdev_->setFormat(feVideoSinkPad, &feFormat);
	if (ret)
		return ret;

	ret = csi2Subdev_->setFormat(csiVideoSourcePad, &feFormat);
	if (ret)
		return ret;

	V4L2DeviceFormat feOutputFormat;
	cfe_[Cfe::Output0].dev()->getFormat(&feOutputFormat);
	BayerFormat feOutputBayer = BayerFormat::fromV4L2PixelFormat(feOutputFormat.fourcc);

	feFormat.code = bayerToMbusCode(feOutputBayer);
	ret = feSubdev_->setFormat(feVideo0SourcePad, &feFormat);

	return ret;
}

void PiSPCameraData::prepareCfe()
{
	/* Fetch an unused config buffer from the stream .*/
	const RPi::BufferObject &config = cfe_[Cfe::Config].acquireBuffer();
	ASSERT(config.mapped);

	{
		std::scoped_lock<FrontEnd> l(*fe_);
		Span<uint8_t> configBuffer = config.mapped->planes()[0];
		fe_->Prepare(reinterpret_cast<pisp_fe_config *>(configBuffer.data()));
	}

	config.buffer->_d()->metadata().planes()[0].bytesused = sizeof(pisp_fe_config);
	cfe_[Cfe::Config].queueBuffer(config.buffer);
}

void PiSPCameraData::prepareBe(uint32_t bufferId, bool stitchSwapBuffers)
{
	FrameBuffer *buffer = cfe_[Cfe::Output0].getBuffers().at(bufferId).buffer;

	LOG(RPI, Debug) << "Input re-queue to ISP, buffer id " << bufferId
			<< ", timestamp: " << buffer->metadata().timestamp;

	isp_[Isp::Input].queueBuffer(buffer);

	/* Ping-pong between input/output buffers for the TDN and Stitch nodes. */
	if (!config_.disableTdn) {
		isp_[Isp::TdnInput].queueBuffer(tdnBuffers_[tdnInputIndex_]);
		isp_[Isp::TdnOutput].queueBuffer(tdnBuffers_[tdnInputIndex_ ^ 1]);
		tdnInputIndex_ ^= 1;
	}

	if (!config_.disableHdr) {
		if (stitchSwapBuffers)
			stitchInputIndex_ ^= 1;
		isp_[Isp::StitchInput].queueBuffer(stitchBuffers_[stitchInputIndex_]);
		isp_[Isp::StitchOutput].queueBuffer(stitchBuffers_[stitchInputIndex_ ^ 1]);
	}

	/* Fetch an unused config buffer from the stream .*/
	const RPi::BufferObject &config = isp_[Isp::Config].acquireBuffer();
	ASSERT(config.mapped);

	Span<uint8_t> configBufferSpan = config.mapped->planes()[0];
	pisp_be_tiles_config *configBuffer = reinterpret_cast<pisp_be_tiles_config *>(configBufferSpan.data());
	be_->Prepare(configBuffer);

	/*
	 * If the LIBCAMERA_RPI_PISP_CONFIG_DUMP environment variable is set,
	 * dump the Backend config to the given file. This is a one-shot
	 * operation, so log the filename that was provided and allow the
	 * application to change the filename for multiple dumps in a single
	 * run.
	 *
	 * \todo Using an environment variable is only a temporary solution
	 * until we have support for vendor specific controls in libcamera.
	 */
	const char *config_dump = utils::secure_getenv("LIBCAMERA_RPI_PISP_CONFIG_DUMP");
	if (config_dump && last_dump_file_ != config_dump) {
		std::ofstream of(config_dump);
		if (of.is_open()) {
			of << be_->GetJsonConfig(configBuffer);
			last_dump_file_ = config_dump;
		}
	}

	isp_[Isp::Config].queueBuffer(config.buffer);
}

void PiSPCameraData::tryRunPipeline()
{
	/* If any of our request or buffer queues are empty, we cannot proceed. */
	if (state_ != State::Idle || requestQueue_.empty() || !cfeJobComplete())
		return;

	CfeJob &job = cfeJobQueue_.front();

	/* Take the first request from the queue and action the IPA. */
	Request *request = requestQueue_.front();

	/* See if a new ScalerCrop value needs to be applied. */
	applyScalerCrop(request->controls());

	/*
	 * Clear the request metadata and fill it with some initial non-IPA
	 * related controls. We clear it first because the request metadata
	 * may have been populated if we have dropped the previous frame.
	 */
	request->metadata().clear();
	fillRequestMetadata(job.sensorControls, request);

	/* Set our state to say the pipeline is active. */
	state_ = State::Busy;

	unsigned int bayerId = cfe_[Cfe::Output0].getBufferId(job.buffers[&cfe_[Cfe::Output0]]);
	unsigned int statsId = cfe_[Cfe::Stats].getBufferId(job.buffers[&cfe_[Cfe::Stats]]);
	ASSERT(bayerId && statsId);

	std::stringstream ss;
	ss << "Signalling IPA processStats and prepareIsp:"
	   << " Bayer buffer id: " << bayerId
	   << " Stats buffer id: " << statsId;

	ipa::RPi::PrepareParams params;
	params.buffers.bayer = RPi::MaskBayerData | bayerId;
	params.buffers.stats = RPi::MaskStats | statsId;
	params.buffers.embedded = 0;
	params.ipaContext = requestQueue_.front()->sequence();
	params.delayContext = job.delayContext;
	params.sensorControls = std::move(job.sensorControls);
	params.requestControls = request->controls();

	if (sensorMetadata_) {
		unsigned int embeddedId =
			cfe_[Cfe::Embedded].getBufferId(job.buffers[&cfe_[Cfe::Embedded]]);

		ASSERT(embeddedId);
		params.buffers.embedded = RPi::MaskEmbeddedData | embeddedId;
		ss << " Embedded buffer id: " << embeddedId;
	}

	LOG(RPI, Debug) << ss.str();

	cfeJobQueue_.pop();
	ipa_->prepareIsp(params);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerPiSP, "rpi/pisp")

} /* namespace libcamera */
