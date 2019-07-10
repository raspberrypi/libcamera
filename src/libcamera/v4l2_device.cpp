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
 * class with methods to open and close the device node associated with the
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
 * \fn V4L2Device::controls()
 * \brief Retrieve the supported V4L2 controls and their information
 * \return A map of the V4L2 controls supported by the device
 */

/**
 * \brief Read controls from the device
 * \param[inout] ctrls The list of controls to read
 *
 * This method reads the value of all controls contained in \a ctrls, and stores
 * their values in the corresponding \a ctrls entry.
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is a compound control, or if any
 * other error occurs during validation of the requested controls, no control is
 * read and this method returns -EINVAL.
 *
 * If an error occurs while reading the controls, the index of the first control
 * that couldn't be read is returned. The value of all controls below that index
 * are updated in \a ctrls, while the value of all the other controls are not
 * changed.
 *
 * \return 0 on success or an error code otherwise
 * \retval -EINVAL One of the control is not supported or not accessible
 * \retval i The index of the control that failed
 */
int V4L2Device::getControls(V4L2ControlList *ctrls)
{
	unsigned int count = ctrls->size();
	if (count == 0)
		return 0;

	const V4L2ControlInfo *controlInfo[count];
	struct v4l2_ext_control v4l2Ctrls[count];
	memset(v4l2Ctrls, 0, sizeof(v4l2Ctrls));

	for (unsigned int i = 0; i < count; ++i) {
		const V4L2Control *ctrl = ctrls->getByIndex(i);
		const auto iter = controls_.find(ctrl->id());
		if (iter == controls_.end()) {
			LOG(V4L2, Error)
				<< "Control '" << ctrl->id() << "' not found";
			return -EINVAL;
		}

		const V4L2ControlInfo *info = &iter->second;
		controlInfo[i] = info;
		v4l2Ctrls[i].id = info->id();
	}

	struct v4l2_ext_controls v4l2ExtCtrls = {};
	v4l2ExtCtrls.which = V4L2_CTRL_WHICH_CUR_VAL;
	v4l2ExtCtrls.controls = v4l2Ctrls;
	v4l2ExtCtrls.count = count;

	int ret = ioctl(VIDIOC_G_EXT_CTRLS, &v4l2ExtCtrls);
	if (ret) {
		unsigned int errorIdx = v4l2ExtCtrls.error_idx;

		/* Generic validation error. */
		if (errorIdx == 0 || errorIdx >= count) {
			LOG(V4L2, Error) << "Unable to read controls: "
					 << strerror(ret);
			return -EINVAL;
		}

		/* A specific control failed. */
		LOG(V4L2, Error) << "Unable to read control " << errorIdx
				 << ": " << strerror(ret);
		count = errorIdx - 1;
		ret = errorIdx;
	}

	updateControls(ctrls, controlInfo, v4l2Ctrls, count);

	return ret;
}

/**
 * \brief Write controls to the device
 * \param[in] ctrls The list of controls to write
 *
 * This method writes the value of all controls contained in \a ctrls, and
 * stores the values actually applied to the device in the corresponding
 * \a ctrls entry.
 *
 * If any control in \a ctrls is not supported by the device, is disabled (i.e.
 * has the V4L2_CTRL_FLAG_DISABLED flag set), is read-only, is a
 * compound control, or if any other error occurs during validation of
 * the requested controls, no control is written and this method returns
 * -EINVAL.
 *
 * If an error occurs while writing the controls, the index of the first
 * control that couldn't be written is returned. All controls below that index
 * are written and their values are updated in \a ctrls, while all other
 * controls are not written and their values are not changed.
 *
 * \return 0 on success or an error code otherwise
 * \retval -EINVAL One of the control is not supported or not accessible
 * \retval i The index of the control that failed
 */
int V4L2Device::setControls(V4L2ControlList *ctrls)
{
	unsigned int count = ctrls->size();
	if (count == 0)
		return 0;

	const V4L2ControlInfo *controlInfo[count];
	struct v4l2_ext_control v4l2Ctrls[count];
	memset(v4l2Ctrls, 0, sizeof(v4l2Ctrls));

	for (unsigned int i = 0; i < count; ++i) {
		const V4L2Control *ctrl = ctrls->getByIndex(i);
		const auto iter = controls_.find(ctrl->id());
		if (iter == controls_.end()) {
			LOG(V4L2, Error)
				<< "Control '" << ctrl->id() << "' not found";
			return -EINVAL;
		}

		const V4L2ControlInfo *info = &iter->second;
		controlInfo[i] = info;
		v4l2Ctrls[i].id = info->id();

		/* Set the v4l2_ext_control value for the write operation. */
		switch (info->type()) {
		case V4L2_CTRL_TYPE_INTEGER64:
			v4l2Ctrls[i].value64 = ctrl->value();
			break;
		default:
			/*
			 * \todo To be changed when support for string and
			 * compound controls will be added.
			 */
			v4l2Ctrls[i].value = ctrl->value();
			break;
		}
	}

	struct v4l2_ext_controls v4l2ExtCtrls = {};
	v4l2ExtCtrls.which = V4L2_CTRL_WHICH_CUR_VAL;
	v4l2ExtCtrls.controls = v4l2Ctrls;
	v4l2ExtCtrls.count = count;

	int ret = ioctl(VIDIOC_S_EXT_CTRLS, &v4l2ExtCtrls);
	if (ret) {
		unsigned int errorIdx = v4l2ExtCtrls.error_idx;

		/* Generic validation error. */
		if (errorIdx == 0 || errorIdx >= count) {
			LOG(V4L2, Error) << "Unable to read controls: "
					 << strerror(ret);
			return -EINVAL;
		}

		/* A specific control failed. */
		LOG(V4L2, Error) << "Unable to read control " << errorIdx
				 << ": " << strerror(ret);
		count = errorIdx - 1;
		ret = errorIdx;
	}

	updateControls(ctrls, controlInfo, v4l2Ctrls, count);

	return ret;
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
	while (1) {
		ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		if (ioctl(VIDIOC_QUERY_EXT_CTRL, &ctrl))
			break;

		if (ctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS ||
		    ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			continue;

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
			LOG(V4L2, Debug) << "Control type '" << info.type()
					 << "' not supported";
			continue;
		}

		controls_.emplace(ctrl.id, info);
	}
}

/*
 * \brief Update the value of the first \a count V4L2 controls in \a ctrls using
 * values in \a v4l2Ctrls
 * \param[inout] ctrls List of V4L2 controls to update
 * \param[in] controlInfo List of V4L2 control information
 * \param[in] v4l2Ctrls List of V4L2 extended controls as returned by the driver
 * \param[in] count The number of controls to update
 */
void V4L2Device::updateControls(V4L2ControlList *ctrls,
				const V4L2ControlInfo **controlInfo,
				const struct v4l2_ext_control *v4l2Ctrls,
				unsigned int count)
{
	for (unsigned int i = 0; i < count; ++i) {
		const struct v4l2_ext_control *v4l2Ctrl = &v4l2Ctrls[i];
		const V4L2ControlInfo *info = controlInfo[i];
		V4L2Control *ctrl = ctrls->getByIndex(i);

		switch (info->type()) {
		case V4L2_CTRL_TYPE_INTEGER64:
			ctrl->setValue(v4l2Ctrl->value64);
			break;
		default:
			/*
			 * \todo To be changed when support for string and
			 * compound controls will be added.
			 */
			ctrl->setValue(v4l2Ctrl->value);
			break;
		}
	}
}

} /* namespace libcamera */
