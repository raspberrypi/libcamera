/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.h - Common base for V4L2 video devices and subdevices
 */
#ifndef __LIBCAMERA_INTERNAL_V4L2_DEVICE_H__
#define __LIBCAMERA_INTERNAL_V4L2_DEVICE_H__

#include <map>
#include <memory>
#include <vector>

#include <linux/videodev2.h>

#include <libcamera/signal.h>
#include <libcamera/span.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/v4l2_controls.h"

namespace libcamera {

class EventNotifier;

class V4L2Device : protected Loggable
{
public:
	void close();
	bool isOpen() const { return fd_ != -1; }

	const ControlInfoMap &controls() const { return controls_; }

	ControlList getControls(const std::vector<uint32_t> &ids);
	int setControls(ControlList *ctrls);

	const struct v4l2_query_ext_ctrl *controlInfo(uint32_t id) const;

	const std::string &deviceNode() const { return deviceNode_; }
	std::string devicePath() const;

	int setFrameStartEnabled(bool enable);
	Signal<uint32_t> frameStart;

protected:
	V4L2Device(const std::string &deviceNode);
	~V4L2Device();

	int open(unsigned int flags);
	int setFd(int fd);

	int ioctl(unsigned long request, void *argp);

	int fd() const { return fd_; }

private:
	void listControls();
	void updateControls(ControlList *ctrls,
			    Span<const v4l2_ext_control> v4l2Ctrls);

	void eventAvailable(EventNotifier *notifier);

	std::map<unsigned int, struct v4l2_query_ext_ctrl> controlInfo_;
	std::vector<std::unique_ptr<V4L2ControlId>> controlIds_;
	ControlInfoMap controls_;
	std::string deviceNode_;
	int fd_;

	EventNotifier *fdEventNotifier_;
	bool frameStartEnabled_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_V4L2_DEVICE_H__ */
