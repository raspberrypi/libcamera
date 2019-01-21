/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.cpp - V4L2 Device
 */

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "log.h"
#include "media_object.h"
#include "v4l2_device.h"

/**
 * \file v4l2_device.h
 * \brief V4L2 Device API
 */
namespace libcamera {

LOG_DEFINE_CATEGORY(V4L2)

/**
 * \struct V4L2Capability
 * \brief struct v4l2_capability object wrapper and helpers
 *
 * The V4L2Capability structure manages the information returned by the
 * VIDIOC_QUERYCAP ioctl.
 */

/**
 * \fn const char *V4L2Capability::driver()
 * \brief Retrieve the driver module name
 * \return The string containing the name of the driver module
 */

/**
 * \fn const char *V4L2Capability::card()
 * \brief Retrieve the device card name
 * \return The string containing the device name
 */

/**
 * \fn const char *V4L2Capability::bus_info()
 * \brief Retrieve the location of the device in the system
 * \return The string containing the device location
 */

/**
 * \fn bool V4L2Capability::isCapture()
 * \brief Identify if the device is capable of capturing video
 * \return True if the device can capture video frames
 */

/**
 * \fn bool V4L2Capability::isOutput()
 * \brief Identify if the device is capable of outputting video
 * \return True if the device can output video frames
 */

/**
 * \fn bool V4L2Capability::hasStreaming()
 * \brief Determine if the device can perform Streaming I/O
 * \return True if the device provides Streaming I/O IOCTLs
 */

/**
 * \class V4L2Device
 * \brief V4L2Device object and API
 *
 * The V4L2 Device API class models an instance of a V4L2 device node.
 * It is constructed with the path to a V4L2 video device node. The device node
 * is only opened upon a call to open() which must be checked for success.
 *
 * The device capabilities are validated when the device is opened and the
 * device is rejected if it is not a suitable V4L2 capture or output device, or
 * if the device does not support streaming I/O.
 *
 * No API call other than open(), isOpen() and close() shall be called on an
 * unopened device instance.
 *
 * Upon destruction any device left open will be closed, and any resources
 * released.
 */

/**
 * \brief Construct a V4L2Device
 * \param devnode The file-system path to the video device node
 */
V4L2Device::V4L2Device(const std::string &devnode)
	: devnode_(devnode), fd_(-1)
{
}

/**
 * \brief Construct a V4L2Device from a MediaEntity
 * \param entity The MediaEntity to build the device from
 *
 * Construct a V4L2Device from a MediaEntity's device node path.
 */
V4L2Device::V4L2Device(const MediaEntity &entity)
	: V4L2Device(entity.devnode())
{
}

V4L2Device::~V4L2Device()
{
	close();
}

/**
 * \brief Open a V4L2 device and query its capabilities
 * \return 0 on success, or a negative error code otherwise
 */
int V4L2Device::open()
{
	int ret;

	if (isOpen()) {
		LOG(V4L2, Error) << "Device already open";
		return -EBUSY;
	}

	ret = ::open(devnode_.c_str(), O_RDWR);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to open V4L2 device '" << devnode_
			<< "': " << strerror(-ret);
		return ret;
	}
	fd_ = ret;

	ret = ioctl(fd_, VIDIOC_QUERYCAP, &caps_);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to query device capabilities: "
			<< strerror(-ret);
		return ret;
	}

	LOG(V4L2, Debug)
		<< "Opened '" << devnode_ << "' "
		<< caps_.bus_info() << ": " << caps_.driver()
		<< ": " << caps_.card();

	if (!caps_.isCapture() && !caps_.isOutput()) {
		LOG(V4L2, Debug) << "Device is not a supported type";
		return -EINVAL;
	}

	if (!caps_.hasStreaming()) {
		LOG(V4L2, Error) << "Device does not support streaming I/O";
		return -EINVAL;
	}

	return 0;
}

/**
 * \brief Check if device is successfully opened
 * \return True if the device is open, false otherwise
 */
bool V4L2Device::isOpen() const
{
	return fd_ != -1;
}

/**
 * \brief Close the device, releasing any resources acquired by open()
 */
void V4L2Device::close()
{
	if (fd_ < 0)
		return;

	::close(fd_);
	fd_ = -1;
}

/**
 * \fn const char *V4L2Device::driverName()
 * \brief Retrieve the name of the V4L2 device driver
 * \return The string containing the driver name
 */

/**
 * \fn const char *V4L2Device::deviceName()
 * \brief Retrieve the name of the V4L2 device
 * \return The string containing the device name
 */

/**
 * \fn const char *V4L2Device::busName()
 * \brief Retrieve the location of the device in the system
 * \return The string containing the device location
 */

} /* namespace libcamera */
