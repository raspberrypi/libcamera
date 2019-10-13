/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.h - Common base for V4L2 video devices and subdevices
 */
#ifndef __LIBCAMERA_V4L2_DEVICE_H__
#define __LIBCAMERA_V4L2_DEVICE_H__

#include <map>
#include <memory>
#include <vector>

#include <linux/videodev2.h>

#include "log.h"
#include "v4l2_controls.h"

namespace libcamera {

class V4L2Device : protected Loggable
{
public:
	void close();
	bool isOpen() const { return fd_ != -1; }

	const ControlInfoMap &controls() const { return controls_; }

	int getControls(ControlList *ctrls);
	int setControls(ControlList *ctrls);

	const std::string &deviceNode() const { return deviceNode_; }

protected:
	V4L2Device(const std::string &deviceNode);
	~V4L2Device();

	int open(unsigned int flags);
	int setFd(int fd);

	int ioctl(unsigned long request, void *argp);

	int fd() { return fd_; }

private:
	void listControls();
	void updateControls(ControlList *ctrls,
			    const struct v4l2_ext_control *v4l2Ctrls,
			    unsigned int count);

	std::vector<std::unique_ptr<V4L2ControlId>> controlIds_;
	ControlInfoMap controls_;
	std::string deviceNode_;
	int fd_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_V4L2_DEVICE_H__ */
