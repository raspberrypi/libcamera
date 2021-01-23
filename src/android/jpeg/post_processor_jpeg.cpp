/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * post_processor_jpeg.cpp - JPEG Post Processor
 */

#include "post_processor_jpeg.h"

#include <chrono>

#include "../camera_device.h"
#include "../camera_metadata.h"
#include "encoder_libjpeg.h"
#include "exif.h"

#include <libcamera/formats.h>

#include "libcamera/internal/log.h"

using namespace libcamera;
using namespace std::chrono_literals;

LOG_DEFINE_CATEGORY(JPEG)

PostProcessorJpeg::PostProcessorJpeg(CameraDevice *const device)
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

	thumbnailer_.configure(inCfg.size, inCfg.pixelFormat);
	StreamConfiguration thCfg = inCfg;
	thCfg.size = thumbnailer_.size();
	if (thumbnailEncoder_.configure(thCfg) != 0) {
		LOG(JPEG, Error) << "Failed to configure thumbnail encoder";
		return -EINVAL;
	}

	encoder_ = std::make_unique<EncoderLibJpeg>();

	return encoder_->configure(inCfg);
}

void PostProcessorJpeg::generateThumbnail(const FrameBuffer &source,
					  std::vector<unsigned char> *thumbnail)
{
	/* Stores the raw scaled-down thumbnail bytes. */
	std::vector<unsigned char> rawThumbnail;

	thumbnailer_.createThumbnail(source, &rawThumbnail);

	if (!rawThumbnail.empty()) {
		/*
		 * \todo Avoid value-initialization of all elements of the
		 * vector.
		 */
		thumbnail->resize(rawThumbnail.size());

		int jpeg_size = thumbnailEncoder_.encode(rawThumbnail,
							 *thumbnail, {});
		thumbnail->resize(jpeg_size);

		LOG(JPEG, Debug)
			<< "Thumbnail compress returned "
			<< jpeg_size << " bytes";
	}
}

int PostProcessorJpeg::process(const FrameBuffer &source,
			       Span<uint8_t> destination,
			       const CameraMetadata &requestMetadata,
			       CameraMetadata *resultMetadata)
{
	if (!encoder_)
		return 0;

	camera_metadata_ro_entry_t entry;
	int ret;

	/* Set EXIF metadata for various tags. */
	Exif exif;
	exif.setMake(cameraDevice_->maker());
	exif.setModel(cameraDevice_->model());

	ret = requestMetadata.getEntry(ANDROID_JPEG_ORIENTATION, &entry);

	const uint32_t jpegOrientation = ret ? *entry.data.i32 : 0;
	resultMetadata->addEntry(ANDROID_JPEG_ORIENTATION, &jpegOrientation, 1);
	exif.setOrientation(jpegOrientation);

	exif.setSize(streamSize_);
	/*
	 * We set the frame's EXIF timestamp as the time of encode.
	 * Since the precision we need for EXIF timestamp is only one
	 * second, it is good enough.
	 */
	exif.setTimestamp(std::time(nullptr), 0ms);

	/* \todo Get this information from libcamera::Request::metadata */
	exif.setExposureTime(0);
	ret = requestMetadata.getEntry(ANDROID_LENS_APERTURE, &entry);
	if (ret)
		exif.setAperture(*entry.data.f);
	exif.setISO(100);
	exif.setFlash(Exif::Flash::FlashNotPresent);
	exif.setWhiteBalance(Exif::WhiteBalance::Auto);

	exif.setFocalLength(1.0);

	ret = requestMetadata.getEntry(ANDROID_JPEG_GPS_TIMESTAMP, &entry);
	if (ret) {
		exif.setGPSDateTimestamp(*entry.data.i64);
		resultMetadata->addEntry(ANDROID_JPEG_GPS_TIMESTAMP,
					 entry.data.i64, 1);
	}

	ret = requestMetadata.getEntry(ANDROID_JPEG_GPS_COORDINATES, &entry);
	if (ret) {
		exif.setGPSLocation(entry.data.d);
		resultMetadata->addEntry(ANDROID_JPEG_GPS_COORDINATES,
					 entry.data.d, 3);
	}

	ret = requestMetadata.getEntry(ANDROID_JPEG_GPS_PROCESSING_METHOD, &entry);
	if (ret) {
		std::string method(entry.data.u8, entry.data.u8 + entry.count);
		exif.setGPSMethod(method);
		resultMetadata->addEntry(ANDROID_JPEG_GPS_PROCESSING_METHOD,
					 entry.data.u8, entry.count);
	}

	std::vector<unsigned char> thumbnail;
	generateThumbnail(source, &thumbnail);
	if (!thumbnail.empty())
		exif.setThumbnail(thumbnail, Exif::Compression::JPEG);

	if (exif.generate() != 0)
		LOG(JPEG, Error) << "Failed to generate valid EXIF data";

	int jpeg_size = encoder_->encode(source, destination, exif.data());
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
	resultMetadata->addEntry(ANDROID_JPEG_SIZE, &jpeg_size, 1);

	/* \todo Configure JPEG encoder with this */
	ret = requestMetadata.getEntry(ANDROID_JPEG_QUALITY, &entry);
	const uint8_t jpegQuality = ret ? *entry.data.u8 : 95;
	resultMetadata->addEntry(ANDROID_JPEG_QUALITY, &jpegQuality, 1);

	return 0;
}
