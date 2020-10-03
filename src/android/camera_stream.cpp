/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * camera_stream.cpp - Camera HAL stream
 */

#include "camera_stream.h"

#include "camera_device.h"
#include "camera_metadata.h"
#include "jpeg/encoder.h"
#include "jpeg/encoder_libjpeg.h"
#include "jpeg/exif.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(HAL);

CameraStream::CameraStream(CameraDevice *cameraDevice,
			   camera3_stream_t *camera3Stream,
			   const libcamera::StreamConfiguration &cfg,
			   Type type, unsigned int index)
	: cameraDevice_(cameraDevice), camera3Stream_(camera3Stream),
	  type_(type), index_(index)
{
	config_ = cameraDevice_->cameraConfiguration();

	format_ = cfg.pixelFormat;
	size_ = cfg.size;

	if (type_ == Type::Internal || type_ == Type::Mapped)
		encoder_ = std::make_unique<EncoderLibJpeg>();
}

const StreamConfiguration &CameraStream::configuration() const
{
	return config_->at(index_);
}

Stream *CameraStream::stream() const
{
	return configuration().stream();
}

int CameraStream::configure(const libcamera::StreamConfiguration &cfg)
{
	if (encoder_)
		return encoder_->configure(cfg);

	return 0;
}

int CameraStream::process(const libcamera::FrameBuffer &source,
			  MappedCamera3Buffer *dest, CameraMetadata *metadata)
{
	if (!encoder_)
		return 0;

	/* Set EXIF metadata for various tags. */
	Exif exif;
	/* \todo Set Make and Model from external vendor tags. */
	exif.setMake("libcamera");
	exif.setModel("cameraModel");
	exif.setOrientation(cameraDevice_->orientation());
	exif.setSize(size_);
	/*
	 * We set the frame's EXIF timestamp as the time of encode.
	 * Since the precision we need for EXIF timestamp is only one
	 * second, it is good enough.
	 */
	exif.setTimestamp(std::time(nullptr));
	if (exif.generate() != 0)
		LOG(HAL, Error) << "Failed to generate valid EXIF data";

	int jpeg_size = encoder_->encode(&source, dest->maps()[0], exif.data());
	if (jpeg_size < 0) {
		LOG(HAL, Error) << "Failed to encode stream image";
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
	uint8_t *resultPtr = dest->maps()[0].data() +
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
