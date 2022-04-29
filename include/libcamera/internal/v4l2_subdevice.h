/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.h - V4L2 Subdevice
 */

#pragma once

#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>

#include <libcamera/color_space.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/media_object.h"
#include "libcamera/internal/v4l2_device.h"

namespace libcamera {

class MediaDevice;

struct V4L2SubdeviceFormat {
	uint32_t mbus_code;
	Size size;
	std::optional<ColorSpace> colorSpace;

	const std::string toString() const;
	uint8_t bitsPerPixel() const;
};

std::ostream &operator<<(std::ostream &out, const V4L2SubdeviceFormat &f);

class V4L2Subdevice : public V4L2Device
{
public:
	using Formats = std::map<unsigned int, std::vector<SizeRange>>;

	enum Whence {
		ActiveFormat,
		TryFormat,
	};

	explicit V4L2Subdevice(const MediaEntity *entity);
	~V4L2Subdevice();

	int open();

	const MediaEntity *entity() const { return entity_; }

	int getSelection(unsigned int pad, unsigned int target,
			 Rectangle *rect);
	int setSelection(unsigned int pad, unsigned int target,
			 Rectangle *rect);

	Formats formats(unsigned int pad);

	int getFormat(unsigned int pad, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat);
	int setFormat(unsigned int pad, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat);

	const std::string &model();

	static std::unique_ptr<V4L2Subdevice>
	fromEntityName(const MediaDevice *media, const std::string &entity);

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY(V4L2Subdevice)

	std::vector<unsigned int> enumPadCodes(unsigned int pad);
	std::vector<SizeRange> enumPadSizes(unsigned int pad,
					    unsigned int code);

	const MediaEntity *entity_;

	std::string model_;
};

} /* namespace libcamera */
