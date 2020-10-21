/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * post_processor_jpeg.cpp - JPEG Post Processor
 */

#include "post_processor_jpeg.h"

#include "../camera_device.h"
#include "../camera_metadata.h"
#include "encoder_libjpeg.h"
#include "exif.h"

#include <libcamera/formats.h>

#include "libcamera/internal/log.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(JPEG)

PostProcessorJpeg::PostProcessorJpeg(CameraDevice *device)
	: cameraDevice_(device)
{
}

int PostProcessorJpeg::configure(const StreamConfiguration &inCfg,
				 const StreamConfiguration &outCfg)
{
	if (inCfg.size != outCfg.size) {
		LOG(JPEG, Error) << "Mismatch of input and output stream sizes";
		return -EINVAL;
	}

	if (outCfg.pixelFormat != formats::MJPEG) {
		LOG(JPEG, Error) << "Output stream pixel format is not JPEG";
		return -EINVAL;
	}

	streamSize_ = outCfg.size;
	encoder_ = std::make_unique<EncoderLibJpeg>();

	return encoder_->configure(inCfg);
}

int PostProcessorJpeg::process(const libcamera::FrameBuffer &source,
			       libcamera::Span<uint8_t> destination,
			       CameraMetadata *metadata)
{
	if (!encoder_)
		return 0;

	/* Set EXIF metadata for various tags. */
	Exif exif;
	/* \todo Set Make and Model from external vendor tags. */
	exif.setMake("libcamera");
	exif.setModel("cameraModel");
	exif.setOrientation(cameraDevice_->orientation());
	exif.setSize(streamSize_);
	/*
	 * We set the frame's EXIF timestamp as the time of encode.
	 * Since the precision we need for EXIF timestamp is only one
	 * second, it is good enough.
	 */
	exif.setTimestamp(std::time(nullptr));
	if (exif.generate() != 0)
		LOG(JPEG, Error) << "Failed to generate valid EXIF data";

	int jpeg_size = encoder_->encode(&source, destination, exif.data());
	if (jpeg_size < 0) {
		LOG(JPEG, Error) << "Failed to encode stream image";
		return jpeg_size;
	}

	/*
	 * Fill in the JPEG blob header.
	 *
	 * The mapped size of the buffer is being returned as
	 * substantially larger than the requested JPEG_MAX_SIZE
	 * (which is referenced from maxJpegBufferSize_). Utilise
	 * this static size to ensure the correct offset of the blob is
	 * determined.
	 *
	 * \todo Investigate if the buffer size mismatch is an issue or
	 * expected behaviour.
	 */
	uint8_t *resultPtr = destination.data() +
			     cameraDevice_->maxJpegBufferSize() -
			     sizeof(struct camera3_jpeg_blob);
	auto *blob = reinterpret_cast<struct camera3_jpeg_blob *>(resultPtr);
	blob->jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
	blob->jpeg_size = jpeg_size;

	/* Update the JPEG result Metadata. */
	metadata->addEntry(ANDROID_JPEG_SIZE, &jpeg_size, 1);

	const uint32_t jpeg_quality = 95;
	metadata->addEntry(ANDROID_JPEG_QUALITY, &jpeg_quality, 1);

	const uint32_t jpeg_orientation = 0;
	metadata->addEntry(ANDROID_JPEG_ORIENTATION, &jpeg_orientation, 1);

	return 0;
}
