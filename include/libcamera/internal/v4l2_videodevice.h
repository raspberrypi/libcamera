/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_videodevice.h - V4L2 Video Device
 */

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <ostream>
#include <stdint.h>
#include <string>
#include <unordered_set>
#include <vector>

#include <linux/videodev2.h>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/timer.h>
#include <libcamera/base/unique_fd.h>
#include <libcamera/base/utils.h>

#include <libcamera/color_space.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/v4l2_device.h"
#include "libcamera/internal/v4l2_pixelformat.h"

namespace libcamera {

class EventNotifier;
class MediaDevice;
class MediaEntity;

struct V4L2Capability final : v4l2_capability {
	const char *driver() const
	{
		return reinterpret_cast<const char *>(v4l2_capability::driver);
	}
	const char *card() const
	{
		return reinterpret_cast<const char *>(v4l2_capability::card);
	}
	const char *bus_info() const
	{
		return reinterpret_cast<const char *>(v4l2_capability::bus_info);
	}
	unsigned int device_caps() const
	{
		return capabilities & V4L2_CAP_DEVICE_CAPS
				    ? v4l2_capability::device_caps
				    : v4l2_capability::capabilities;
	}
	bool isMultiplanar() const
	{
		return device_caps() & (V4L2_CAP_VIDEO_CAPTURE_MPLANE |
					V4L2_CAP_VIDEO_OUTPUT_MPLANE |
					V4L2_CAP_VIDEO_M2M_MPLANE);
	}
	bool isCapture() const
	{
		return device_caps() & (V4L2_CAP_VIDEO_CAPTURE |
					V4L2_CAP_VIDEO_CAPTURE_MPLANE |
					V4L2_CAP_META_CAPTURE);
	}
	bool isOutput() const
	{
		return device_caps() & (V4L2_CAP_VIDEO_OUTPUT |
					V4L2_CAP_VIDEO_OUTPUT_MPLANE |
					V4L2_CAP_META_OUTPUT);
	}
	bool isVideo() const
	{
		return device_caps() & (V4L2_CAP_VIDEO_CAPTURE |
					V4L2_CAP_VIDEO_CAPTURE_MPLANE |
					V4L2_CAP_VIDEO_OUTPUT |
					V4L2_CAP_VIDEO_OUTPUT_MPLANE);
	}
	bool isM2M() const
	{
		return device_caps() & (V4L2_CAP_VIDEO_M2M |
					V4L2_CAP_VIDEO_M2M_MPLANE);
	}
	bool isMeta() const
	{
		return device_caps() & (V4L2_CAP_META_CAPTURE |
					V4L2_CAP_META_OUTPUT);
	}
	bool isVideoCapture() const
	{
		return isVideo() && isCapture();
	}
	bool isVideoOutput() const
	{
		return isVideo() && isOutput();
	}
	bool isMetaCapture() const
	{
		return isMeta() && isCapture();
	}
	bool isMetaOutput() const
	{
		return isMeta() && isOutput();
	}
	bool hasStreaming() const
	{
		return device_caps() & V4L2_CAP_STREAMING;
	}
	bool hasMediaController() const
	{
		return device_caps() & V4L2_CAP_IO_MC;
	}
};

class V4L2BufferCache
{
public:
	V4L2BufferCache(unsigned int numEntries);
	V4L2BufferCache(const std::vector<std::unique_ptr<FrameBuffer>> &buffers);
	~V4L2BufferCache();

	bool isEmpty() const;
	int get(const FrameBuffer &buffer);
	void put(unsigned int index);

private:
	class Entry
	{
	public:
		Entry();
		Entry(bool free, uint64_t lastUsed, const FrameBuffer &buffer);

		bool operator==(const FrameBuffer &buffer) const;

		bool free_;
		uint64_t lastUsed_;

	private:
		struct Plane {
			Plane(const FrameBuffer::Plane &plane)
				: fd(plane.fd.get()), length(plane.length)
			{
			}

			int fd;
			unsigned int length;
		};

		std::vector<Plane> planes_;
	};

	std::atomic<uint64_t> lastUsedCounter_;
	std::vector<Entry> cache_;
	/* \todo Expose the miss counter through an instrumentation API. */
	unsigned int missCounter_;
};

class V4L2DeviceFormat
{
public:
	struct Plane {
		uint32_t size = 0;
		uint32_t bpl = 0;
	};

	V4L2PixelFormat fourcc;
	Size size;
	std::optional<ColorSpace> colorSpace;

	std::array<Plane, 3> planes;
	unsigned int planesCount = 0;

	const std::string toString() const;
};

std::ostream &operator<<(std::ostream &out, const V4L2DeviceFormat &f);

class V4L2VideoDevice : public V4L2Device
{
public:
	using Formats = std::map<V4L2PixelFormat, std::vector<SizeRange>>;

	explicit V4L2VideoDevice(const std::string &deviceNode);
	explicit V4L2VideoDevice(const MediaEntity *entity);
	~V4L2VideoDevice();

	int open();
	int open(SharedFD handle, enum v4l2_buf_type type);
	void close();

	const char *driverName() const { return caps_.driver(); }
	const char *deviceName() const { return caps_.card(); }
	const char *busName() const { return caps_.bus_info(); }

	const V4L2Capability &caps() const { return caps_; }

	int getFormat(V4L2DeviceFormat *format);
	int tryFormat(V4L2DeviceFormat *format);
	int setFormat(V4L2DeviceFormat *format);
	Formats formats(uint32_t code = 0);

	int setSelection(unsigned int target, Rectangle *rect);

	int allocateBuffers(unsigned int count,
			    std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int exportBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int importBuffers(unsigned int count);
	int releaseBuffers();

	int queueBuffer(FrameBuffer *buffer);
	Signal<FrameBuffer *> bufferReady;

	int streamOn();
	int streamOff();

	void setDequeueTimeout(utils::Duration timeout);
	Signal<> dequeueTimeout;

	static std::unique_ptr<V4L2VideoDevice>
	fromEntityName(const MediaDevice *media, const std::string &entity);

	V4L2PixelFormat toV4L2PixelFormat(const PixelFormat &pixelFormat) const;

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY(V4L2VideoDevice)

	enum class State {
		Streaming,
		Stopping,
		Stopped,
	};

	int initFormats();

	int getFormatMeta(V4L2DeviceFormat *format);
	int trySetFormatMeta(V4L2DeviceFormat *format, bool set);

	int getFormatMultiplane(V4L2DeviceFormat *format);
	int trySetFormatMultiplane(V4L2DeviceFormat *format, bool set);

	int getFormatSingleplane(V4L2DeviceFormat *format);
	int trySetFormatSingleplane(V4L2DeviceFormat *format, bool set);

	std::vector<V4L2PixelFormat> enumPixelformats(uint32_t code);
	std::vector<SizeRange> enumSizes(V4L2PixelFormat pixelFormat);

	int requestBuffers(unsigned int count, enum v4l2_memory memoryType);
	int createBuffers(unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	std::unique_ptr<FrameBuffer> createBuffer(unsigned int index);
	UniqueFD exportDmabufFd(unsigned int index, unsigned int plane);

	void bufferAvailable();
	FrameBuffer *dequeueBuffer();

	void watchdogExpired();

	template<typename T>
	static std::optional<ColorSpace> toColorSpace(const T &v4l2Format);

	V4L2Capability caps_;
	V4L2DeviceFormat format_;
	const PixelFormatInfo *formatInfo_;
	std::unordered_set<V4L2PixelFormat> pixelFormats_;

	enum v4l2_buf_type bufferType_;
	enum v4l2_memory memoryType_;

	V4L2BufferCache *cache_;
	std::map<unsigned int, FrameBuffer *> queuedBuffers_;

	EventNotifier *fdBufferNotifier_;

	State state_;
	std::optional<unsigned int> firstFrame_;

	Timer watchdog_;
	utils::Duration watchdogDuration_;
};

class V4L2M2MDevice
{
public:
	V4L2M2MDevice(const std::string &deviceNode);
	~V4L2M2MDevice();

	int open();
	void close();

	V4L2VideoDevice *output() { return output_; }
	V4L2VideoDevice *capture() { return capture_; }

private:
	std::string deviceNode_;

	V4L2VideoDevice *output_;
	V4L2VideoDevice *capture_;
};

} /* namespace libcamera */
