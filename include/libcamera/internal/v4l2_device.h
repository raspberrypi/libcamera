/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.h - Common base for V4L2 video devices and subdevices
 */

#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <linux/videodev2.h>

#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/span.h>
#include <libcamera/base/unique_fd.h>

#include <libcamera/color_space.h>
#include <libcamera/controls.h>

#include "libcamera/internal/formats.h"

namespace libcamera {

class EventNotifier;

class V4L2Device : protected Loggable
{
public:
	void close();
	bool isOpen() const { return fd_.isValid(); }

	const ControlInfoMap &controls() const { return controls_; }

	ControlList getControls(const std::vector<uint32_t> &ids);
	int setControls(ControlList *ctrls);

	const struct v4l2_query_ext_ctrl *controlInfo(uint32_t id) const;

	const std::string &deviceNode() const { return deviceNode_; }
	std::string devicePath() const;

	int setFrameStartEnabled(bool enable);
	Signal<uint32_t> frameStart;

	void updateControlInfo();

protected:
	V4L2Device(const std::string &deviceNode);
	~V4L2Device();

	int open(unsigned int flags);
	int setFd(UniqueFD fd);

	int ioctl(unsigned long request, void *argp);

	int fd() const { return fd_.get(); }

	template<typename T>
	static std::optional<ColorSpace> toColorSpace(const T &v4l2Format,
						      PixelFormatInfo::ColourEncoding colourEncoding);

	template<typename T>
	static int fromColorSpace(const std::optional<ColorSpace> &colorSpace, T &v4l2Format);

private:
	static ControlType v4l2CtrlType(uint32_t ctrlType);
	static std::unique_ptr<ControlId> v4l2ControlId(const v4l2_query_ext_ctrl &ctrl);
	std::optional<ControlInfo> v4l2ControlInfo(const v4l2_query_ext_ctrl &ctrl);
	std::optional<ControlInfo> v4l2MenuControlInfo(const v4l2_query_ext_ctrl &ctrl);

	void listControls();
	void updateControls(ControlList *ctrls,
			    Span<const v4l2_ext_control> v4l2Ctrls);

	void eventAvailable();

	std::map<unsigned int, struct v4l2_query_ext_ctrl> controlInfo_;
	std::vector<std::unique_ptr<ControlId>> controlIds_;
	ControlIdMap controlIdMap_;
	ControlInfoMap controls_;
	std::string deviceNode_;
	UniqueFD fd_;

	EventNotifier *fdEventNotifier_;
	bool frameStartEnabled_;
};

} /* namespace libcamera */
