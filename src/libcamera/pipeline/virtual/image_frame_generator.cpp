/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Derived class of FrameGenerator for generating frames from images
 */

#include "image_frame_generator.h"

#include <string>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>

#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "libyuv/convert.h"
#include "libyuv/scale.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Virtual)

/*
 * Factory function to create an ImageFrameGenerator object.
 * Read the images and convert them to buffers in NV12 format.
 * Store the pointers to the buffers to a list (imageFrameDatas)
 */
std::unique_ptr<ImageFrameGenerator>
ImageFrameGenerator::create(ImageFrames &imageFrames)
{
	std::unique_ptr<ImageFrameGenerator> imageFrameGenerator =
		std::make_unique<ImageFrameGenerator>();
	imageFrameGenerator->imageFrames_ = &imageFrames;

	/*
	 * For each file in the directory, load the image,
	 * convert it to NV12, and store the pointer.
	 */
	for (const auto &path : imageFrames.files) {
		File file(path);
		if (!file.open(File::OpenModeFlag::ReadOnly)) {
			LOG(Virtual, Error) << "Failed to open image file " << file.fileName()
					    << ": " << strerror(file.error());
			return nullptr;
		}

		/* Read the image file to data */
		auto fileSize = file.size();
		auto buffer = std::make_unique<uint8_t[]>(fileSize);
		if (file.read({ buffer.get(), static_cast<size_t>(fileSize) }) != fileSize) {
			LOG(Virtual, Error) << "Failed to read file " << file.fileName()
					    << ": " << strerror(file.error());
			return nullptr;
		}

		/* Get the width and height of the image */
		int width, height;
		if (libyuv::MJPGSize(buffer.get(), fileSize, &width, &height)) {
			LOG(Virtual, Error) << "Failed to get the size of the image file: "
					    << file.fileName();
			return nullptr;
		}

		std::unique_ptr<uint8_t[]> dstY =
			std::make_unique<uint8_t[]>(width * height);
		std::unique_ptr<uint8_t[]> dstUV =
			std::make_unique<uint8_t[]>(width * height / 2);
		int ret = libyuv::MJPGToNV12(buffer.get(), fileSize,
					     dstY.get(), width, dstUV.get(),
					     width, width, height, width, height);
		if (ret != 0)
			LOG(Virtual, Error) << "MJPGToNV12() failed with " << ret;

		imageFrameGenerator->imageFrameDatas_.emplace_back(
			ImageFrameData{ std::move(dstY), std::move(dstUV),
					Size(width, height) });
	}

	ASSERT(!imageFrameGenerator->imageFrameDatas_.empty());

	return imageFrameGenerator;
}

/*
 * \var ImageFrameGenerator::frameRepeat
 * \brief Number of frames to repeat before proceeding to the next frame
 */

/* Scale the buffers for image frames. */
void ImageFrameGenerator::configure(const Size &size)
{
	/* Reset the source images to prevent multiple configuration calls */
	scaledFrameDatas_.clear();
	frameIndex_ = 0;
	parameter_ = 0;

	for (unsigned int i = 0; i < imageFrameDatas_.size(); i++) {
		/* Scale the imageFrameDatas_ to scaledY and scaledUV */
		unsigned int halfSizeWidth = (size.width + 1) / 2;
		unsigned int halfSizeHeight = (size.height + 1) / 2;
		std::unique_ptr<uint8_t[]> scaledY =
			std::make_unique<uint8_t[]>(size.width * size.height);
		std::unique_ptr<uint8_t[]> scaledUV =
			std::make_unique<uint8_t[]>(halfSizeWidth * halfSizeHeight * 2);
		auto &src = imageFrameDatas_[i];

		/*
		 * \todo Some platforms might enforce stride due to GPU.
                 * The width needs to be a multiple of the stride to work
                 * properly for now.
		 */
		libyuv::NV12Scale(src.y.get(), src.size.width,
				  src.uv.get(), src.size.width,
				  src.size.width, src.size.height,
				  scaledY.get(), size.width, scaledUV.get(), size.width,
				  size.width, size.height, libyuv::FilterMode::kFilterBilinear);

		scaledFrameDatas_.emplace_back(
			ImageFrameData{ std::move(scaledY), std::move(scaledUV), size });
	}
}

int ImageFrameGenerator::generateFrame(const StreamConfiguration &cfg, const FrameBuffer *buffer)
{
	ASSERT(!scaledFrameDatas_.empty());

	/*
	 * \todo Assume all planes are contiguous and mapped at different
	 * offsets on the same fd. This holds true as long as the implementation
	 * of DmaBufAllocator only allocates contiguous planes. If this will
	 * change or a different allocator will be used then we need to create
	 * one dma syncer for each non-contiguous plane.
	 */
	DmaSyncer outDmaSyncer(buffer->planes()[0].fd,
			       DmaSyncer::SyncType::Write);
	MappedFrameBuffer mappedFrameBuffer(buffer,
					    MappedFrameBuffer::MapFlag::Write);

	const auto &planes = mappedFrameBuffer.planes();
	const Size &size = cfg.size;
	const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);

	/* Loop only around the number of images available */
	frameIndex_ %= imageFrameDatas_.size();

	int ret;
	auto y = scaledFrameDatas_[frameIndex_].y.get();
	auto uv = scaledFrameDatas_[frameIndex_].uv.get();

	/* Write the scaledY and scaledUV to the mapped frame buffer */
	switch (cfg.pixelFormat) {
	case formats::NV12:
		ret = libyuv::NV12Copy(y, size.width, uv, size.width,
				       planes[0].data(), info.stride(size.width, 0),
				       planes[1].data(), info.stride(size.width, 1),
				       size.width, size.height);
		break;

	case formats::YUV420:
		ret = libyuv::NV12ToI420(y, size.width, uv, size.width,
					 planes[0].data(), info.stride(size.width, 0),
					 planes[1].data(), info.stride(size.width, 1),
					 planes[2].data(), info.stride(size.width, 2),
					 size.width, size.height);
		break;

	case formats::ABGR8888:
	case formats::XBGR8888:
		ret = libyuv::NV12ToABGR(y, size.width, uv, size.width,
					 planes[0].data(), info.stride(size.width, 0),
					 size.width, size.height);
		break;

	default:
		LOG(Virtual, Error) << "Unsupported format";
		return -1;
	}

	if (ret != 0)
		LOG(Virtual, Error) << "Conversion failed with " << ret;

	/* Proceed to the next image every 4 frames */
	/* \todo Consider setting the frameRepeat in the config file  */
	parameter_++;
	if (parameter_ % frameRepeat == 0)
		frameIndex_++;

	return ret;
}

/*
 * \var ImageFrameGenerator::imageFrameDatas_
 * \brief List of pointers to the not scaled image buffers
 */

/*
 * \var ImageFrameGenerator::scaledFrameDatas_
 * \brief List of pointers to the scaled image buffers
 */

/*
 * \var ImageFrameGenerator::imageFrames_
 * \brief Pointer to the imageFrames_ in VirtualCameraData
 */

/*
 * \var ImageFrameGenerator::parameter_
 * \brief Speed parameter. Change to the next image every parameter_ frames
 */

} /* namespace libcamera */
