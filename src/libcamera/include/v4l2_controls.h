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
	V4L2ControlInfo(const struct v4l2_query_ext_ctrl &ctrl);

	const ControlRange &range() const { return range_; }

private:
	ControlRange range_;
};

class V4L2ControlInfoMap : private std::map<const ControlId *, V4L2ControlInfo>
{
public:
	V4L2ControlInfoMap &operator=(std::map<const ControlId *, V4L2ControlInfo> &&info);

	using std::map<const ControlId *, V4L2ControlInfo>::key_type;
	using std::map<const ControlId *, V4L2ControlInfo>::mapped_type;
	using std::map<const ControlId *, V4L2ControlInfo>::value_type;
	using std::map<const ControlId *, V4L2ControlInfo>::size_type;
	using std::map<const ControlId *, V4L2ControlInfo>::iterator;
	using std::map<const ControlId *, V4L2ControlInfo>::const_iterator;

	using std::map<const ControlId *, V4L2ControlInfo>::begin;
	using std::map<const ControlId *, V4L2ControlInfo>::cbegin;
	using std::map<const ControlId *, V4L2ControlInfo>::end;
	using std::map<const ControlId *, V4L2ControlInfo>::cend;
	using std::map<const ControlId *, V4L2ControlInfo>::at;
	using std::map<const ControlId *, V4L2ControlInfo>::empty;
	using std::map<const ControlId *, V4L2ControlInfo>::size;
	using std::map<const ControlId *, V4L2ControlInfo>::count;
	using std::map<const ControlId *, V4L2ControlInfo>::find;

	mapped_type &at(unsigned int key);
	const mapped_type &at(unsigned int key) const;
	size_type count(unsigned int key) const;
	iterator find(unsigned int key);
	const_iterator find(unsigned int key) const;

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
