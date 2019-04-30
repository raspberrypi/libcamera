/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.h - V4L2 Device
 */
#ifndef __LIBCAMERA_V4L2_DEVICE_H__
#define __LIBCAMERA_V4L2_DEVICE_H__

#include <atomic>
#include <string>
#include <vector>

#include <linux/videodev2.h>

#include <libcamera/geometry.h>
#include <libcamera/signal.h>

#include "log.h"

namespace libcamera {

class Buffer;
class BufferPool;
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
					V4L2_CAP_VIDEO_OUTPUT_MPLANE);
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
					V4L2_CAP_VIDEO_OUTPUT_MPLANE);
	}
	bool isVideo() const
	{
		return device_caps() & (V4L2_CAP_VIDEO_CAPTURE |
					V4L2_CAP_VIDEO_CAPTURE_MPLANE |
					V4L2_CAP_VIDEO_OUTPUT |
					V4L2_CAP_VIDEO_OUTPUT_MPLANE);
	}
	bool isMeta() const
	{
		return device_caps() & V4L2_CAP_META_CAPTURE;
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
	bool hasStreaming() const
	{
		return device_caps() & V4L2_CAP_STREAMING;
	}
};

class V4L2DeviceFormat
{
public:
	uint32_t fourcc;
	Size size;

	struct {
		uint32_t size;
		uint32_t bpl;
	} planes[3];
	unsigned int planesCount;

	const std::string toString() const;
};

class V4L2Device : protected Loggable
{
public:
	explicit V4L2Device(const std::string &deviceNode);
	explicit V4L2Device(const MediaEntity *entity);
	V4L2Device(const V4L2Device &) = delete;
	~V4L2Device();

	V4L2Device &operator=(const V4L2Device &) = delete;

	int open();
	bool isOpen() const;
	void close();

	const char *driverName() const { return caps_.driver(); }
	const char *deviceName() const { return caps_.card(); }
	const char *busName() const { return caps_.bus_info(); }
	const std::string &deviceNode() const { return deviceNode_; }

	int getFormat(V4L2DeviceFormat *format);
	int setFormat(V4L2DeviceFormat *format);

	int exportBuffers(BufferPool *pool);
	int importBuffers(BufferPool *pool);
	int releaseBuffers();

	int queueBuffer(Buffer *buffer);
	Signal<Buffer *> bufferReady;

	int streamOn();
	int streamOff();

	static V4L2Device *fromEntityName(const MediaDevice *media,
					  const std::string &entity);

protected:
	std::string logPrefix() const;

private:
	int getFormatSingleplane(V4L2DeviceFormat *format);
	int setFormatSingleplane(V4L2DeviceFormat *format);

	int getFormatMultiplane(V4L2DeviceFormat *format);
	int setFormatMultiplane(V4L2DeviceFormat *format);

	int requestBuffers(unsigned int count);
	int createPlane(Buffer *buffer, unsigned int plane,
			unsigned int length);

	Buffer *dequeueBuffer();
	void bufferAvailable(EventNotifier *notifier);

	std::string deviceNode_;
	int fd_;
	V4L2Capability caps_;

	enum v4l2_buf_type bufferType_;
	enum v4l2_memory memoryType_;

	BufferPool *bufferPool_;
	std::atomic<unsigned int> queuedBuffersCount_;

	EventNotifier *fdEvent_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_V4L2_DEVICE_H__ */
