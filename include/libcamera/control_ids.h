/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_ids.h : Control ID list
 */

#ifndef __LIBCAMERA_CONTROL_IDS_H__
#define __LIBCAMERA_CONTROL_IDS_H__

#include <functional>

namespace libcamera {

enum ControlId {
	AwbEnable,
	Brightness,
	Contrast,
	Saturation,
	ManualExposure,
	ManualGain,
};

} /* namespace libcamera */

namespace std {

template<>
struct hash<libcamera::ControlId> {
	using argument_type = libcamera::ControlId;
	using result_type = std::size_t;

	result_type operator()(const argument_type &key) const noexcept
	{
		return std::hash<std::underlying_type<argument_type>::type>()(key);
	}
};

} /* namespace std */

#endif // __LIBCAMERA_CONTROL_IDS_H__
