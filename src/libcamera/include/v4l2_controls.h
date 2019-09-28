/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_controls.h - V4L2 Controls Support
 */

#ifndef __LIBCAMERA_V4L2_CONTROLS_H__
#define __LIBCAMERA_V4L2_CONTROLS_H__

#include <map>
#include <stdint.h>
#include <string>
#include <vector>

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <libcamera/controls.h>

namespace libcamera {

class V4L2ControlInfo
{
public:
	V4L2ControlInfo(const struct v4l2_query_ext_ctrl &ctrl);

	unsigned int id() const { return id_; }
	unsigned int type() const { return type_; }
	size_t size() const { return size_; }
	const std::string &name() const { return name_; }

	int64_t min() const { return min_; }
	int64_t max() const { return max_; }

private:
	unsigned int id_;
	unsigned int type_;
	size_t size_;
	std::string name_;

	int64_t min_;
	int64_t max_;
};

using V4L2ControlInfoMap = std::map<unsigned int, V4L2ControlInfo>;

class V4L2Control
{
public:
	V4L2Control(unsigned int id, const ControlValue &value = ControlValue())
		: id_(id), value_(value)
	{
	}

	unsigned int id() const { return id_; }
	const ControlValue &value() const { return value_; }
	ControlValue &value() { return value_; }

private:
	unsigned int id_;
	ControlValue value_;
};

class V4L2ControlList
{
public:
	using iterator = std::vector<V4L2Control>::iterator;
	using const_iterator = std::vector<V4L2Control>::const_iterator;

	iterator begin() { return controls_.begin(); }
	const_iterator begin() const { return controls_.begin(); }
	iterator end() { return controls_.end(); }
	const_iterator end() const { return controls_.end(); }

	bool empty() const { return controls_.empty(); }
	std::size_t size() const { return controls_.size(); }

	void clear() { controls_.clear(); }
	void add(unsigned int id, int64_t value = 0);

	V4L2Control *getByIndex(unsigned int index);
	V4L2Control *operator[](unsigned int id);

private:
	std::vector<V4L2Control> controls_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_V4L2_CONTROLS_H__ */
