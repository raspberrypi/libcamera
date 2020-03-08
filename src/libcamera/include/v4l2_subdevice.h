/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.h - V4L2 Subdevice
 */
#ifndef __LIBCAMERA_V4L2_SUBDEVICE_H__
#define __LIBCAMERA_V4L2_SUBDEVICE_H__

#include <map>
#include <string>
#include <vector>

#include <libcamera/geometry.h>

#include "formats.h"
#include "log.h"
#include "media_object.h"
#include "v4l2_device.h"

namespace libcamera {

class MediaDevice;

struct V4L2SubdeviceFormat {
	uint32_t mbus_code;
	Size size;

	const std::string toString() const;
};

class V4L2Subdevice : public V4L2Device
{
public:
	enum Whence {
		ActiveFormat,
		TryFormat,
	};

	explicit V4L2Subdevice(const MediaEntity *entity);
	V4L2Subdevice(const V4L2Subdevice &) = delete;
	V4L2Subdevice &operator=(const V4L2Subdevice &) = delete;
	~V4L2Subdevice();

	int open();

	const MediaEntity *entity() const { return entity_; }

	int setCrop(unsigned int pad, Rectangle *rect);
	int setCompose(unsigned int pad, Rectangle *rect);

	ImageFormats formats(unsigned int pad);

	int getFormat(unsigned int pad, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat);
	int setFormat(unsigned int pad, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat);

	static V4L2Subdevice *fromEntityName(const MediaDevice *media,
					     const std::string &entity);

protected:
	std::string logPrefix() const;

private:
	std::vector<unsigned int> enumPadCodes(unsigned int pad);
	std::vector<SizeRange> enumPadSizes(unsigned int pad,
					    unsigned int code);

	int setSelection(unsigned int pad, unsigned int target,
			 Rectangle *rect);

	const MediaEntity *entity_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_V4L2_SUBDEVICE_H__ */
