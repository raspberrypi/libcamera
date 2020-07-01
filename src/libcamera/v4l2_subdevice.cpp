/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.cpp - V4L2 Subdevice
 */

#include "libcamera/internal/v4l2_subdevice.h"

#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"
#include "libcamera/internal/utils.h"

/**
 * \file v4l2_subdevice.h
 * \brief V4L2 Subdevice API
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(V4L2)

namespace {

/*
 * \struct V4L2SubdeviceFormatInfo
 * \brief Information about media bus formats
 * \param bitsPerPixel Bits per pixel
 * \param name Name of MBUS format
 */
struct V4L2SubdeviceFormatInfo {
	unsigned int bitsPerPixel;
	const char *name;
};

/*
 * \var formatInfoMap
 * \brief A map that associates V4L2SubdeviceFormatInfo struct to V4L2 media
 * bus codes
 */
const std::map<uint32_t, V4L2SubdeviceFormatInfo> formatInfoMap = {
	{ V4L2_MBUS_FMT_RGB444_2X8_PADHI_BE, { 16, "RGB444_2X8_PADHI_BE" } },
	{ V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE, { 16, "RGB444_2X8_PADHI_LE" } },
	{ V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE, { 16, "RGB555_2X8_PADHI_BE" } },
	{ V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE, { 16, "RGB555_2X8_PADHI_LE" } },
	{ V4L2_MBUS_FMT_BGR565_2X8_BE, { 16, "BGR565_2X8_BE" } },
	{ V4L2_MBUS_FMT_BGR565_2X8_LE, { 16, "BGR565_2X8_LE" } },
	{ V4L2_MBUS_FMT_RGB565_2X8_BE, { 16, "RGB565_2X8_BE" } },
	{ V4L2_MBUS_FMT_RGB565_2X8_LE, { 16, "RGB565_2X8_LE" } },
	{ V4L2_MBUS_FMT_RGB666_1X18, { 18, "RGB666_1X18" } },
	{ V4L2_MBUS_FMT_RGB888_1X24, { 24, "RGB888_1X24" } },
	{ V4L2_MBUS_FMT_RGB888_2X12_BE, { 24, "RGB888_2X12_BE" } },
	{ V4L2_MBUS_FMT_RGB888_2X12_LE, { 24, "RGB888_2X12_LE" } },
	{ V4L2_MBUS_FMT_ARGB8888_1X32, { 32, "ARGB8888_1X32" } },
	{ V4L2_MBUS_FMT_Y8_1X8, { 8, "Y8_1X8" } },
	{ V4L2_MBUS_FMT_UV8_1X8, { 8, "UV8_1X8" } },
	{ V4L2_MBUS_FMT_UYVY8_1_5X8, { 12, "UYVY8_1_5X8" } },
	{ V4L2_MBUS_FMT_VYUY8_1_5X8, { 12, "VYUY8_1_5X8" } },
	{ V4L2_MBUS_FMT_YUYV8_1_5X8, { 12, "YUYV8_1_5X8" } },
	{ V4L2_MBUS_FMT_YVYU8_1_5X8, { 12, "YVYU8_1_5X8" } },
	{ V4L2_MBUS_FMT_UYVY8_2X8, { 16, "UYVY8_2X8" } },
	{ V4L2_MBUS_FMT_VYUY8_2X8, { 16, "VYUY8_2X8" } },
	{ V4L2_MBUS_FMT_YUYV8_2X8, { 16, "YUYV8_2X8" } },
	{ V4L2_MBUS_FMT_YVYU8_2X8, { 16, "YVYU8_2X8" } },
	{ V4L2_MBUS_FMT_Y10_1X10, { 10, "Y10_1X10" } },
	{ V4L2_MBUS_FMT_UYVY10_2X10, { 20, "UYVY10_2X10" } },
	{ V4L2_MBUS_FMT_VYUY10_2X10, { 20, "VYUY10_2X10" } },
	{ V4L2_MBUS_FMT_YUYV10_2X10, { 20, "YUYV10_2X10" } },
	{ V4L2_MBUS_FMT_YVYU10_2X10, { 20, "YVYU10_2X10" } },
	{ V4L2_MBUS_FMT_Y12_1X12, { 12, "Y12_1X12" } },
	{ V4L2_MBUS_FMT_UYVY8_1X16, { 16, "UYVY8_1X16" } },
	{ V4L2_MBUS_FMT_VYUY8_1X16, { 16, "VYUY8_1X16" } },
	{ V4L2_MBUS_FMT_YUYV8_1X16, { 16, "YUYV8_1X16" } },
	{ V4L2_MBUS_FMT_YVYU8_1X16, { 16, "YVYU8_1X16" } },
	{ V4L2_MBUS_FMT_YDYUYDYV8_1X16, { 16, "YDYUYDYV8_1X16" } },
	{ V4L2_MBUS_FMT_UYVY10_1X20, { 20, "UYVY10_1X20" } },
	{ V4L2_MBUS_FMT_VYUY10_1X20, { 20, "VYUY10_1X20" } },
	{ V4L2_MBUS_FMT_YUYV10_1X20, { 20, "YUYV10_1X20" } },
	{ V4L2_MBUS_FMT_YVYU10_1X20, { 20, "YVYU10_1X20" } },
	{ V4L2_MBUS_FMT_YUV10_1X30, { 30, "YUV10_1X30" } },
	{ V4L2_MBUS_FMT_AYUV8_1X32, { 32, "AYUV8_1X32" } },
	{ V4L2_MBUS_FMT_UYVY12_2X12, { 24, "UYVY12_2X12" } },
	{ V4L2_MBUS_FMT_VYUY12_2X12, { 24, "VYUY12_2X12" } },
	{ V4L2_MBUS_FMT_YUYV12_2X12, { 24, "YUYV12_2X12" } },
	{ V4L2_MBUS_FMT_YVYU12_2X12, { 24, "YVYU12_2X12" } },
	{ V4L2_MBUS_FMT_UYVY12_1X24, { 24, "UYVY12_1X24" } },
	{ V4L2_MBUS_FMT_VYUY12_1X24, { 24, "VYUY12_1X24" } },
	{ V4L2_MBUS_FMT_YUYV12_1X24, { 24, "YUYV12_1X24" } },
	{ V4L2_MBUS_FMT_YVYU12_1X24, { 24, "YVYU12_1X24" } },
	{ V4L2_MBUS_FMT_SBGGR8_1X8, { 8, "SBGGR8_1X8" } },
	{ V4L2_MBUS_FMT_SGBRG8_1X8, { 8, "SGBRG8_1X8" } },
	{ V4L2_MBUS_FMT_SGRBG8_1X8, { 8, "SGRBG8_1X8" } },
	{ V4L2_MBUS_FMT_SRGGB8_1X8, { 8, "SRGGB8_1X8" } },
	{ V4L2_MBUS_FMT_SBGGR10_ALAW8_1X8, { 8, "SBGGR10_ALAW8_1X8" } },
	{ V4L2_MBUS_FMT_SGBRG10_ALAW8_1X8, { 8, "SGBRG10_ALAW8_1X8" } },
	{ V4L2_MBUS_FMT_SGRBG10_ALAW8_1X8, { 8, "SGRBG10_ALAW8_1X8" } },
	{ V4L2_MBUS_FMT_SRGGB10_ALAW8_1X8, { 8, "SRGGB10_ALAW8_1X8" } },
	{ V4L2_MBUS_FMT_SBGGR10_DPCM8_1X8, { 8, "SBGGR10_DPCM8_1X8" } },
	{ V4L2_MBUS_FMT_SGBRG10_DPCM8_1X8, { 8, "SGBRG10_DPCM8_1X8" } },
	{ V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8, { 8, "SGRBG10_DPCM8_1X8" } },
	{ V4L2_MBUS_FMT_SRGGB10_DPCM8_1X8, { 8, "SRGGB10_DPCM8_1X8" } },
	{ V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE, { 16, "SBGGR10_2X8_PADHI_BE" } },
	{ V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE, { 16, "SBGGR10_2X8_PADHI_LE" } },
	{ V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE, { 16, "SBGGR10_2X8_PADLO_BE" } },
	{ V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE, { 16, "SBGGR10_2X8_PADLO_LE" } },
	{ V4L2_MBUS_FMT_SBGGR10_1X10, { 10, "SBGGR10_1X10" } },
	{ V4L2_MBUS_FMT_SGBRG10_1X10, { 10, "SGBRG10_1X10" } },
	{ V4L2_MBUS_FMT_SGRBG10_1X10, { 10, "SGRBG10_1X10" } },
	{ V4L2_MBUS_FMT_SRGGB10_1X10, { 10, "SRGGB10_1X10" } },
	{ V4L2_MBUS_FMT_SBGGR12_1X12, { 12, "SBGGR12_1X12" } },
	{ V4L2_MBUS_FMT_SGBRG12_1X12, { 12, "SGBRG12_1X12" } },
	{ V4L2_MBUS_FMT_SGRBG12_1X12, { 12, "SGRBG12_1X12" } },
	{ V4L2_MBUS_FMT_SRGGB12_1X12, { 12, "SRGGB12_1X12" } },
	{ V4L2_MBUS_FMT_AHSV8888_1X32, { 32, "AHSV8888_1X32" } },
};

} /* namespace */

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
 * \var V4L2SubdeviceFormat::mbus_code
 * \brief The image format bus code
 */

/**
 * \var V4L2SubdeviceFormat::size
 * \brief The image size in pixels
 */

/**
 * \brief Assemble and return a string describing the format
 * \return A string describing the V4L2SubdeviceFormat
 */
const std::string V4L2SubdeviceFormat::toString() const
{
	std::stringstream mbus;
	mbus << size.toString() << "-";

	const auto it = formatInfoMap.find(mbus_code);

	if (it == formatInfoMap.end())
		mbus << utils::hex(mbus_code, 4);
	else
		mbus << it->second.name;

	return mbus.str();
}

/**
 * \brief Retrieve the number of bits per pixel for the V4L2 subdevice format
 * \return The number of bits per pixel for the format, or 0 if the format is
 * not supported
 */
uint8_t V4L2SubdeviceFormat::bitsPerPixel() const
{
	const auto it = formatInfoMap.find(mbus_code);
	if (it == formatInfoMap.end()) {
		LOG(V4L2, Error) << "No information available for format '"
				 << toString() << "'";
		return 0;
	}

	return it->second.bitsPerPixel;
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
	return V4L2Device::open(O_RDWR);
}

/**
 * \fn V4L2Subdevice::entity()
 * \brief Retrieve the media entity associated with the subdevice
 * \return The subdevice's associated media entity.
 */

/**
 * \brief Get selection rectangle \a rect for \a target
 * \param[in] pad The 0-indexed pad number the rectangle is retrieved from
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[out] rect The retrieved selection rectangle
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getSelection(unsigned int pad, unsigned int target,
				Rectangle *rect)
{
	struct v4l2_subdev_selection sel = {};

	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = pad;
	sel.target = target;
	sel.flags = 0;

	int ret = ioctl(VIDIOC_SUBDEV_G_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to get rectangle " << target << " on pad "
			<< pad << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}

/**
 * \brief Set selection rectangle \a rect for \a target
 * \param[in] pad The 0-indexed pad number the rectangle is to be applied to
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[inout] rect The selection rectangle to be applied
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setSelection(unsigned int pad, unsigned int target,
				Rectangle *rect)
{
	struct v4l2_subdev_selection sel = {};

	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = pad;
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
			<< pad << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}
/**
 * \brief Enumerate all media bus codes and frame sizes on a \a pad
 * \param[in] pad The 0-indexed pad number to enumerate formats on
 *
 * Enumerate all media bus codes and frame sizes supported by the subdevice on
 * a \a pad.
 *
 * \return A list of the supported device formats
 */
V4L2Subdevice::Formats V4L2Subdevice::formats(unsigned int pad)
{
	Formats formats;

	if (pad >= entity_->pads().size()) {
		LOG(V4L2, Error) << "Invalid pad: " << pad;
		return {};
	}

	for (unsigned int code : enumPadCodes(pad)) {
		std::vector<SizeRange> sizes = enumPadSizes(pad, code);
		if (sizes.empty())
			return {};

		const auto inserted = formats.insert({ code, sizes });
		if (!inserted.second) {
			LOG(V4L2, Error)
				<< "Could not add sizes for media bus code "
				<< code << " on pad " << pad;
			return {};
		}
	}

	return formats;
}

/**
 * \brief Retrieve the image format set on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be retrieved from
 * \param[out] format The image bus format
 * \param[in] whence The format to get, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getFormat(unsigned int pad, V4L2SubdeviceFormat *format,
			     Whence whence)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = whence == ActiveFormat ? V4L2_SUBDEV_FORMAT_ACTIVE
			: V4L2_SUBDEV_FORMAT_TRY;
	subdevFmt.pad = pad;

	int ret = ioctl(VIDIOC_SUBDEV_G_FMT, &subdevFmt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to get format on pad " << pad
			<< ": " << strerror(-ret);
		return ret;
	}

	format->size.width = subdevFmt.format.width;
	format->size.height = subdevFmt.format.height;
	format->mbus_code = subdevFmt.format.code;

	return 0;
}

/**
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
int V4L2Subdevice::setFormat(unsigned int pad, V4L2SubdeviceFormat *format,
			     Whence whence)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = whence == ActiveFormat ? V4L2_SUBDEV_FORMAT_ACTIVE
			: V4L2_SUBDEV_FORMAT_TRY;
	subdevFmt.pad = pad;
	subdevFmt.format.width = format->size.width;
	subdevFmt.format.height = format->size.height;
	subdevFmt.format.code = format->mbus_code;

	int ret = ioctl(VIDIOC_SUBDEV_S_FMT, &subdevFmt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to set format on pad " << pad
			<< ": " << strerror(-ret);
		return ret;
	}

	format->size.width = subdevFmt.format.width;
	format->size.height = subdevFmt.format.height;
	format->mbus_code = subdevFmt.format.code;

	return 0;
}

/**
 * \brief Create a new video subdevice instance from \a entity in media device
 * \a media
 * \param[in] media The media device where the entity is registered
 * \param[in] entity The media entity name
 *
 * Releasing memory of the newly created instance is responsibility of the
 * caller of this function.
 *
 * \return A newly created V4L2Subdevice on success, nullptr otherwise
 */
V4L2Subdevice *V4L2Subdevice::fromEntityName(const MediaDevice *media,
					     const std::string &entity)
{
	MediaEntity *mediaEntity = media->getEntityByName(entity);
	if (!mediaEntity)
		return nullptr;

	return new V4L2Subdevice(mediaEntity);
}

std::string V4L2Subdevice::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

std::vector<unsigned int> V4L2Subdevice::enumPadCodes(unsigned int pad)
{
	std::vector<unsigned int> codes;
	int ret;

	for (unsigned int index = 0; ; index++) {
		struct v4l2_subdev_mbus_code_enum mbusEnum = {};
		mbusEnum.pad = pad;
		mbusEnum.index = index;
		mbusEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = ioctl(VIDIOC_SUBDEV_ENUM_MBUS_CODE, &mbusEnum);
		if (ret)
			break;

		codes.push_back(mbusEnum.code);
	}

	if (ret < 0 && ret != -EINVAL) {
		LOG(V4L2, Error)
			<< "Unable to enumerate formats on pad " << pad
			<< ": " << strerror(-ret);
		return {};
	}

	return codes;
}

std::vector<SizeRange> V4L2Subdevice::enumPadSizes(unsigned int pad,
						   unsigned int code)
{
	std::vector<SizeRange> sizes;
	int ret;

	for (unsigned int index = 0;; index++) {
		struct v4l2_subdev_frame_size_enum sizeEnum = {};
		sizeEnum.index = index;
		sizeEnum.pad = pad;
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
			<< "Unable to enumerate sizes on pad " << pad
			<< ": " << strerror(-ret);
		return {};
	}

	return sizes;
}

} /* namespace libcamera */
