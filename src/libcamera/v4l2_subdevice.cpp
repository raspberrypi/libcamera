/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.cpp - V4L2 Subdevice
 */

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/v4l2-subdev.h>

#include "geometry.h"
#include "log.h"
#include "media_object.h"
#include "v4l2_subdevice.h"

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
 * \var V4L2SubdeviceFormat::width
 * \brief The image width in pixels
 */

/**
 * \var V4L2SubdeviceFormat::height
 * \brief The image height in pixels
 */

/**
 * \var V4L2SubdeviceFormat::mbus_code
 * \brief The image format bus code
 */

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

/**
 * \brief Open a V4L2 subdevice
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::open()
{
	int ret;

	if (isOpen()) {
		LOG(V4L2Subdev, Error) << "Device already open";
		return -EBUSY;
	}

	ret = ::open(deviceNode().c_str(), O_RDWR);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
			<< "Failed to open V4L2 subdevice '" << deviceNode()
			<< "': " << strerror(-ret);
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
 * \fn V4L2Subdevice::deviceNode()
 * \brief Retrieve the path of the device node associated with the subdevice
 *
 * \return The subdevice's device node system path
 */

/**
 * \fn V4L2Subdevice::deviceName()
 * \brief Retrieve the name of the media entity associated with the subdevice
 *
 * \return The name of the media entity the subdevice is associated to
 */

std::string V4L2Subdevice::logPrefix() const
{
	return "'" + deviceName() + "'";
}

/**
 * \brief Set a crop rectangle on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the rectangle is to be applied to
 * \param[inout] rect The rectangle describing crop target area
 *
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
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setCompose(unsigned int pad, Rectangle *rect)
{
	return setSelection(pad, V4L2_SEL_TGT_COMPOSE, rect);
}

/**
 * \brief Retrieve the image format set on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be retrieved from
 * \param[out] format The image bus format
 *
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

	format->width = subdevFmt.format.width;
	format->height = subdevFmt.format.height;
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
	subdevFmt.format.width = format->width;
	subdevFmt.format.height = format->height;
	subdevFmt.format.code = format->mbus_code;

	int ret = ioctl(fd_, VIDIOC_SUBDEV_S_FMT, &subdevFmt);
	if (ret) {
		ret = -errno;
		LOG(V4L2Subdev, Error)
			<< "Unable to set format on pad " << pad
			<< ": " << strerror(-ret);
		return ret;
	}

	format->width = subdevFmt.format.width;
	format->height = subdevFmt.format.height;
	format->mbus_code = subdevFmt.format.code;

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

	sel.r.left = rect->y;
	sel.r.top = rect->x;
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

	return 0;
}

} /* namespace libcamera */
