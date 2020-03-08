/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.cpp - V4L2 Subdevice
 */

#include "v4l2_subdevice.h"

#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/v4l2-subdev.h>

#include <libcamera/geometry.h>

#include "log.h"
#include "media_device.h"
#include "media_object.h"
#include "utils.h"

/**
 * \file v4l2_subdevice.h
 * \brief V4L2 Subdevice API
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(V4L2)

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
	std::stringstream ss;
	ss << size.toString() << "-" << utils::hex(mbus_code, 4);
	return ss.str();
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
 * \brief Set a crop rectangle on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the rectangle is to be applied to
 * \param[inout] rect The rectangle describing crop target area
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setCrop(unsigned int pad, Rectangle *rect)
{
	return setSelection(pad, V4L2_SEL_TGT_CROP, rect);
}

/**
 * \brief Set a compose rectangle on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the rectangle is to be applied to
 * \param[inout] rect The rectangle describing the compose target area
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setCompose(unsigned int pad, Rectangle *rect)
{
	return setSelection(pad, V4L2_SEL_TGT_COMPOSE, rect);
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
ImageFormats V4L2Subdevice::formats(unsigned int pad)
{
	ImageFormats formats;

	if (pad >= entity_->pads().size()) {
		LOG(V4L2, Error) << "Invalid pad: " << pad;
		return {};
	}

	for (unsigned int code : enumPadCodes(pad)) {
		std::vector<SizeRange> sizes = enumPadSizes(pad, code);
		if (sizes.empty())
			return {};

		if (formats.addFormat(code, sizes)) {
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
 * actually applied format parameters, as \ref V4L2Subdevice::getFormat would
 * do.
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
	sel.r.width = rect->w;
	sel.r.height = rect->h;

	int ret = ioctl(VIDIOC_SUBDEV_S_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to set rectangle " << target << " on pad "
			<< pad << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->w = sel.r.width;
	rect->h = sel.r.height;

	return 0;
}

} /* namespace libcamera */
