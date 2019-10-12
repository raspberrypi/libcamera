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

#include <linux/videodev2.h>

#include <libcamera/controls.h>

namespace libcamera {

class V4L2ControlId : public ControlId
{
public:
	V4L2ControlId(const struct v4l2_query_ext_ctrl &ctrl);
};

class V4L2ControlInfo
{
public:
	V4L2ControlInfo(const V4L2ControlId &id,
			const struct v4l2_query_ext_ctrl &ctrl);

	const ControlId &id() const { return *id_; }
	const ControlRange &range() const { return range_; }

private:
	const V4L2ControlId *id_;
	ControlRange range_;
};

class V4L2ControlInfoMap : private std::map<unsigned int, V4L2ControlInfo>
{
public:
	V4L2ControlInfoMap &operator=(std::map<unsigned int, V4L2ControlInfo> &&info);

	using std::map<unsigned int, V4L2ControlInfo>::begin;
	using std::map<unsigned int, V4L2ControlInfo>::cbegin;
	using std::map<unsigned int, V4L2ControlInfo>::end;
	using std::map<unsigned int, V4L2ControlInfo>::cend;
	using std::map<unsigned int, V4L2ControlInfo>::at;
	using std::map<unsigned int, V4L2ControlInfo>::empty;
	using std::map<unsigned int, V4L2ControlInfo>::size;
	using std::map<unsigned int, V4L2ControlInfo>::count;
	using std::map<unsigned int, V4L2ControlInfo>::find;

	const ControlIdMap &idmap() const { return idmap_; }

private:
	ControlIdMap idmap_;
};

class V4L2ControlList : public ControlList
{
public:
	V4L2ControlList(const V4L2ControlInfoMap &info)
		: ControlList(info.idmap())
	{
	}
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_V4L2_CONTROLS_H__ */
