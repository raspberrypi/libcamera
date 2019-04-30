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

/**
 * \file v4l2_subdevice.h
 * \brief V4L2 Subdevice API
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(V4L2Subdev)

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
 * (see V4L2Device::V4L2DeviceFormat).
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

	ss.fill(0);
	ss << size.toString() << "-0x" << std::hex << std::setw(4) << mbus_code;

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
 * \brief Create a V4L2 subdevice from a MediaEntity using its device node
 * path
 */
V4L2Subdevice::V4L2Subdevice(const MediaEntity *entity)
	: entity_(entity), fd_(-1)
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
	int ret;

	if (isOpen()) {
		LOG(V4L2Subdev, Error) << "Device already open";
		return -EBUSY;
	}

	ret = ::open(entity_->deviceNode().c_str(), O_RDWR);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
			<< "Failed to open V4L2 subdevice '"
			<< entity_->deviceNode() << "': " << strerror(-ret);
		return ret;
	}
	fd_ = ret;

	return 0;
}

/**
 * \brief Check if the subdevice is open
 * \return True if the subdevice is open, false otherwise
 */
bool V4L2Subdevice::isOpen() const
{
	return fd_ != -1;
}

/**
 * \brief Close the subdevice, releasing any resources acquired by open()
 */
void V4L2Subdevice::close()
{
	if (!isOpen())
		return;

	::close(fd_);
	fd_ = -1;
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
 * \brief List the sub-device image resolutions and formats on \a pad
 * \param[in] pad The 0-indexed pad number to enumerate formats on
 *
 * Retrieve a list of image formats and sizes on the \a pad of a video
 * subdevice. Subdevices can report either a list of discrete sizes they
 * support or a list of intervals expressed as a [min-max] sizes range.
 *
 * Each image size list is associated with a media bus pixel code for which
 * the reported resolutions are supported.
 *
 * \return A map of image formats associated with a list of image sizes, or
 * an empty map on error or if the pad does not exist
 */
FormatEnum V4L2Subdevice::formats(unsigned int pad)
{
	FormatEnum formatMap = {};
	struct v4l2_subdev_mbus_code_enum mbusEnum = {};
	int ret;

	if (pad >= entity_->pads().size()) {
		LOG(V4L2Subdev, Error) << "Invalid pad: " << pad;
		return formatMap;
	}

	mbusEnum.pad = pad;
	mbusEnum.index = 0;
	mbusEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	while (true) {
		ret = ioctl(fd_, VIDIOC_SUBDEV_ENUM_MBUS_CODE, &mbusEnum);
		if (ret)
			break;

		ret = enumPadSizes(pad, mbusEnum.code,
				   &formatMap[mbusEnum.code]);
		if (ret)
			break;

		mbusEnum.index++;
	}

	if (ret && (errno != EINVAL && errno != ENOTTY)) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
			<< "Unable to enumerate formats on pad " << pad
			<< ": " << strerror(-ret);
		formatMap.clear();
	}

	return formatMap;
}

/**
 * \brief Retrieve the image format set on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be retrieved from
 * \param[out] format The image bus format
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getFormat(unsigned int pad, V4L2SubdeviceFormat *format)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	subdevFmt.pad = pad;

	int ret = ioctl(fd_, VIDIOC_SUBDEV_G_FMT, &subdevFmt);
	if (ret) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
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
 *
 * Apply the requested image format to the desired media pad and return the
 * actually applied format parameters, as \ref V4L2Subdevice::getFormat would
 * do.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setFormat(unsigned int pad, V4L2SubdeviceFormat *format)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	subdevFmt.pad = pad;
	subdevFmt.format.width = format->size.width;
	subdevFmt.format.height = format->size.height;
	subdevFmt.format.code = format->mbus_code;

	int ret = ioctl(fd_, VIDIOC_SUBDEV_S_FMT, &subdevFmt);
	if (ret) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
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

int V4L2Subdevice::enumPadSizes(unsigned int pad,unsigned int code,
				std::vector<SizeRange> *sizes)
{
	struct v4l2_subdev_frame_size_enum sizeEnum = {};
	int ret;

	sizeEnum.index = 0;
	sizeEnum.pad = pad;
	sizeEnum.code = code;
	sizeEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	while (true) {
		ret = ioctl(fd_, VIDIOC_SUBDEV_ENUM_FRAME_SIZE, &sizeEnum);
		if (ret)
			break;

		sizes->emplace_back(sizeEnum.min_width, sizeEnum.min_height,
				    sizeEnum.max_width, sizeEnum.max_height);

		sizeEnum.index++;
	}

	if (ret && (errno != EINVAL && errno != ENOTTY)) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
			<< "Unable to enumerate sizes on pad " << pad
			<< ": " << strerror(-ret);
		sizes->clear();

		return ret;
	}

	return 0;
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

	int ret = ioctl(fd_, VIDIOC_SUBDEV_S_SELECTION, &sel);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
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
