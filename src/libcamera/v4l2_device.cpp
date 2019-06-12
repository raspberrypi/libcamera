/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.cpp - Common base for V4L2 video devices and subdevices
 */

#include "v4l2_device.h"

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "log.h"
#include "v4l2_controls.h"

/**
 * \file v4l2_device.h
 * \brief Common base for V4L2 devices and subdevices
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(V4L2)

/**
 * \class V4L2Device
 * \brief Base class for V4L2VideoDevice and V4L2Subdevice
 *
 * The V4L2Device class groups together the methods and fields common to
 * both the V4L2VideoDevice and V4L2Subdevice classes, and provides a base
 * class whith methods to open and close the device node associated with the
 * device and to perform IOCTL system calls on it.
 *
 * The V4L2Device class cannot be instantiated directly, as its constructor
 * is protected. Users should instead create instances of one the derived
 * classes to model either a V4L2 video device or a V4L2 subdevice.
 */

/**
 * \brief Construct a V4L2Device
 * \param[in] deviceNode The device node filesystem path
 *
 * Initialize the file descriptor to -1 and store the \a deviceNode to be used
 * at open() time, and the \a logTag to prefix log messages with.
 */
V4L2Device::V4L2Device(const std::string &deviceNode)
	: deviceNode_(deviceNode), fd_(-1)
{
}

/**
 * \brief Destroy a V4L2Device
 */
V4L2Device::~V4L2Device()
{
}

/**
 * \brief Open a V4L2 device node
 * \param[in] flags Access mode flags
 *
 * Open the device node path with the provided access mode \a flags and
 * initialize the file descriptor, which was initially set to -1.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::open(unsigned int flags)
{
	if (isOpen()) {
		LOG(V4L2, Error) << "Device already open";
		return -EBUSY;
	}

	int ret = ::open(deviceNode_.c_str(), flags);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error) << "Failed to open V4L2 device: "
				 << strerror(-ret);
		return ret;
	}

	fd_ = ret;

	listControls();

	return 0;
}

/**
 * \brief Close the device node
 *
 * Reset the file descriptor to -1
 */
void V4L2Device::close()
{
	if (!isOpen())
		return;

	if (::close(fd_) < 0)
		LOG(V4L2, Error) << "Failed to close V4L2 device: "
				 << strerror(errno);
	fd_ = -1;
}

/**
 * \fn V4L2Device::isOpen()
 * \brief Check if the V4L2 device node is open
 * \return True if the V4L2 device node is open, false otherwise
 */

/**
 * \brief Retrieve information about a control
 * \param[in] id The control ID
 * \return A pointer to the V4L2ControlInfo for control \a id, or nullptr
 * if the device doesn't have such a control
 */
const V4L2ControlInfo *V4L2Device::getControlInfo(unsigned int id) const
{
	auto it = controls_.find(id);
	if (it == controls_.end())
		return nullptr;

	return &it->second;
}

/**
 * \brief Perform an IOCTL system call on the device node
 * \param[in] request The IOCTL request code
 * \param[in] argp A pointer to the IOCTL argument
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::ioctl(unsigned long request, void *argp)
{
	/*
	 * Printing out an error message is usually better performed
	 * in the caller, which can provide more context.
	 */
	if (::ioctl(fd_, request, argp) < 0)
		return -errno;

	return 0;
}

/**
 * \fn V4L2Device::deviceNode()
 * \brief Retrieve the device node path
 * \return The device node path
 */

/**
 * \fn V4L2Device::fd()
 * \brief Retrieve the V4L2 device file descriptor
 * \return The V4L2 device file descriptor, -1 if the device node is not open
 */

/*
 * \brief List and store information about all controls supported by the
 * V4L2 device
 */
void V4L2Device::listControls()
{
	struct v4l2_query_ext_ctrl ctrl = {};

	/* \todo Add support for menu and compound controls. */
	ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
	while (ioctl(VIDIOC_QUERY_EXT_CTRL, &ctrl) == 0) {
		if (ctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS ||
		    ctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
			ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
			continue;
		}

		V4L2ControlInfo info(ctrl);
		switch (info.type()) {
		case V4L2_CTRL_TYPE_INTEGER:
		case V4L2_CTRL_TYPE_BOOLEAN:
		case V4L2_CTRL_TYPE_MENU:
		case V4L2_CTRL_TYPE_BUTTON:
		case V4L2_CTRL_TYPE_INTEGER64:
		case V4L2_CTRL_TYPE_BITMASK:
		case V4L2_CTRL_TYPE_INTEGER_MENU:
			break;
		/* \todo Support compound controls. */
		default:
			LOG(V4L2, Error) << "Control type '" << info.type()
					 << "' not supported";
			continue;
		}

		controls_.emplace(ctrl.id, info);
		ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}
}

} /* namespace libcamera */
