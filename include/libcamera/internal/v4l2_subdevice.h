/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * V4L2 Subdevice
 */

#pragma once

#include <memory>
#include <optional>
#include <ostream>
#include <stdint.h>
#include <string>
#include <vector>

#include <linux/v4l2-subdev.h>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>

#include <libcamera/color_space.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/media_object.h"
#include "libcamera/internal/v4l2_device.h"

namespace libcamera {

class MediaDevice;

class MediaBusFormatInfo
{
public:
	enum class Type {
		Image,
		Metadata,
		EmbeddedData,
	};

	bool isValid() const { return code != 0; }

	static const MediaBusFormatInfo &info(uint32_t code);

	const char *name;
	uint32_t code;
	Type type;
	unsigned int bitsPerPixel;
	PixelFormatInfo::ColourEncoding colourEncoding;
};

struct V4L2SubdeviceCapability final : v4l2_subdev_capability {
	bool isReadOnly() const
	{
		return capabilities & V4L2_SUBDEV_CAP_RO_SUBDEV;
	}
	bool hasStreams() const
	{
		return capabilities & V4L2_SUBDEV_CAP_STREAMS;
	}
};

struct V4L2SubdeviceFormat {
	uint32_t code;
	Size size;
	std::optional<ColorSpace> colorSpace;

	const std::string toString() const;
};

std::ostream &operator<<(std::ostream &out, const V4L2SubdeviceFormat &f);

class V4L2Subdevice : public V4L2Device
{
public:
	using Formats = std::map<unsigned int, std::vector<SizeRange>>;

	enum Whence {
		TryFormat = V4L2_SUBDEV_FORMAT_TRY,
		ActiveFormat = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	struct Stream {
		Stream()
			: pad(0), stream(0)
		{
		}

		Stream(unsigned int p, unsigned int s)
			: pad(p), stream(s)
		{
		}

		unsigned int pad;
		unsigned int stream;
	};

	struct Route {
		Route()
			: flags(0)
		{
		}

		Route(const Stream &snk, const Stream &src, uint32_t f)
			: sink(snk), source(src), flags(f)
		{
		}

		Stream sink;
		Stream source;
		uint32_t flags;
	};

	using Routing = std::vector<Route>;

	explicit V4L2Subdevice(const MediaEntity *entity);
	~V4L2Subdevice();

	int open();

	const MediaEntity *entity() const { return entity_; }

	int getSelection(const Stream &stream, unsigned int target,
			 Rectangle *rect);
	int getSelection(unsigned int pad, unsigned int target, Rectangle *rect)
	{
		return getSelection({ pad, 0 }, target, rect);
	}
	int setSelection(const Stream &stream, unsigned int target,
			 Rectangle *rect);
	int setSelection(unsigned int pad, unsigned int target, Rectangle *rect)
	{
		return setSelection({ pad, 0 }, target, rect);
	}

	Formats formats(const Stream &stream);
	Formats formats(unsigned int pad)
	{
		return formats({ pad, 0 });
	}

	int getFormat(const Stream &stream, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat);
	int getFormat(unsigned int pad, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat)
	{
		return getFormat({ pad, 0 }, format, whence);
	}
	int setFormat(const Stream &stream, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat);
	int setFormat(unsigned int pad, V4L2SubdeviceFormat *format,
		      Whence whence = ActiveFormat)
	{
		return setFormat({ pad, 0 }, format, whence);
	}

	int getRouting(Routing *routing, Whence whence = ActiveFormat);
	int setRouting(Routing *routing, Whence whence = ActiveFormat);

	const std::string &model();
	const V4L2SubdeviceCapability &caps() const { return caps_; }

	static std::unique_ptr<V4L2Subdevice>
	fromEntityName(const MediaDevice *media, const std::string &entity);

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY(V4L2Subdevice)

	std::optional<ColorSpace>
	toColorSpace(const v4l2_mbus_framefmt &format) const;

	std::vector<unsigned int> enumPadCodes(const Stream &stream);
	std::vector<SizeRange> enumPadSizes(const Stream &stream,
					    unsigned int code);

	int getRoutingLegacy(Routing *routing, Whence whence);
	int setRoutingLegacy(Routing *routing, Whence whence);

	const MediaEntity *entity_;

	std::string model_;
	struct V4L2SubdeviceCapability caps_;
};

bool operator==(const V4L2Subdevice::Stream &lhs, const V4L2Subdevice::Stream &rhs);
static inline bool operator!=(const V4L2Subdevice::Stream &lhs,
			      const V4L2Subdevice::Stream &rhs)
{
	return !(lhs == rhs);
}

std::ostream &operator<<(std::ostream &out, const V4L2Subdevice::Stream &stream);
std::ostream &operator<<(std::ostream &out, const V4L2Subdevice::Route &route);
std::ostream &operator<<(std::ostream &out, const V4L2Subdevice::Routing &routing);

} /* namespace libcamera */
