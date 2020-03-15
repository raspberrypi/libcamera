/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * pixelformats.h - libcamera pixel formats
 */
#ifndef __LIBCAMERA_PIXEL_FORMATS_H__
#define __LIBCAMERA_PIXEL_FORMATS_H__

#include <set>
#include <stdint.h>
#include <string>

#include <linux/drm_fourcc.h>

namespace libcamera {

class PixelFormat
{
public:
	PixelFormat();
	PixelFormat(uint32_t fourcc, const std::set<uint64_t> &modifiers = {});

	bool operator==(const PixelFormat &other) const;
	bool operator!=(const PixelFormat &other) const { return !(*this == other); }
	bool operator<(const PixelFormat &other) const;

	bool isValid() const { return fourcc_ != 0; }

	operator uint32_t() const { return fourcc_; }
	uint32_t fourcc() const { return fourcc_; }
	const std::set<uint64_t> &modifiers() const { return modifiers_; }

	std::string toString() const;

private:
	uint32_t fourcc_;
	std::set<uint64_t> modifiers_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIXEL_FORMATS_H__ */
