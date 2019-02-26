/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.h - V4L2 Subdevice
 */
#ifndef __LIBCAMERA_V4L2_SUBDEVICE_H__
#define __LIBCAMERA_V4L2_SUBDEVICE_H__

#include <string>

#include "log.h"
#include "media_object.h"

namespace libcamera {

struct Rectangle;

struct V4L2SubdeviceFormat {
	uint32_t mbus_code;
	uint32_t width;
	uint32_t height;
};

class V4L2Subdevice : protected Loggable
{
public:
	explicit V4L2Subdevice(const MediaEntity *entity);
	V4L2Subdevice(const V4L2Subdevice &) = delete;
	V4L2Subdevice &operator=(const V4L2Subdevice &) = delete;

	int open();
	bool isOpen() const;
	void close();

	std::string deviceNode() const { return entity_->deviceNode(); }
	std::string deviceName() const { return entity_->name(); }

	int setCrop(unsigned int pad, Rectangle *rect);
	int setCompose(unsigned int pad, Rectangle *rect);

	int getFormat(unsigned int pad, V4L2SubdeviceFormat *format);
	int setFormat(unsigned int pad, V4L2SubdeviceFormat *format);

protected:
	std::string logPrefix() const;

private:
	int setSelection(unsigned int pad, unsigned int target,
			 Rectangle *rect);

	const MediaEntity *entity_;
	int fd_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_V4L2_SUBDEVICE_H__ */
