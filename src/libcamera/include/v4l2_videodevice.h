/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_videodevice.h - V4L2 Video Device
 */
#ifndef __LIBCAMERA_V4L2_VIDEODEVICE_H__
#define __LIBCAMERA_V4L2_VIDEODEVICE_H__

#include <string>
#include <vector>

#include <linux/videodev2.h>
#include <memory>

#include <libcamera/buffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixelformats.h>
#include <libcamera/signal.h>

#include "formats.h"
#include "log.h"
#include "v4l2_device.h"

namespace libcamera {

class EventNotifier;
class FileDescriptor;
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
};

class V4L2BufferCache
{
public:
	V4L2BufferCache(unsigned int numEntries);
	V4L2BufferCache(const std::vector<std::unique_ptr<FrameBuffer>> &buffers);
	~V4L2BufferCache();

	int get(const FrameBuffer &buffer);
	void put(unsigned int index);

private:
	class Entry
	{
	public:
		Entry();
		Entry(bool free, const FrameBuffer &buffer);

		bool operator==(const FrameBuffer &buffer);

		bool free;

	private:
		struct Plane {
			Plane(const FrameBuffer::Plane &plane)
				: fd(plane.fd.fd()), length(plane.length)
			{
			}

			int fd;
			unsigned int length;
		};

		std::vector<Plane> planes_;
	};

	std::vector<Entry> cache_;
	/* \todo Expose the miss counter through an instrumentation API. */
	unsigned int missCounter_;
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

class V4L2VideoDevice : public V4L2Device
{
public:
	explicit V4L2VideoDevice(const std::string &deviceNode);
	explicit V4L2VideoDevice(const MediaEntity *entity);
	V4L2VideoDevice(const V4L2VideoDevice &) = delete;
	~V4L2VideoDevice();

	V4L2VideoDevice &operator=(const V4L2VideoDevice &) = delete;

	int open();
	int open(int handle, enum v4l2_buf_type type);
	void close();

	const char *driverName() const { return caps_.driver(); }
	const char *deviceName() const { return caps_.card(); }
	const char *busName() const { return caps_.bus_info(); }

	int getFormat(V4L2DeviceFormat *format);
	int setFormat(V4L2DeviceFormat *format);
	ImageFormats formats();

	int exportBuffers(BufferPool *pool);
	int importBuffers(BufferPool *pool);
	int releaseBuffers();

	int queueBuffer(Buffer *buffer);
	std::vector<std::unique_ptr<Buffer>> queueAllBuffers();
	Signal<Buffer *> bufferReady;

	int streamOn();
	int streamOff();

	static V4L2VideoDevice *fromEntityName(const MediaDevice *media,
					       const std::string &entity);

	static PixelFormat toPixelFormat(uint32_t v4l2Fourcc);
	uint32_t toV4L2Fourcc(PixelFormat pixelFormat);
	static uint32_t toV4L2Fourcc(PixelFormat pixelFormat, bool multiplanar);

protected:
	std::string logPrefix() const;

private:
	int getFormatMeta(V4L2DeviceFormat *format);
	int setFormatMeta(V4L2DeviceFormat *format);

	int getFormatMultiplane(V4L2DeviceFormat *format);
	int setFormatMultiplane(V4L2DeviceFormat *format);

	int getFormatSingleplane(V4L2DeviceFormat *format);
	int setFormatSingleplane(V4L2DeviceFormat *format);

	std::vector<unsigned int> enumPixelformats();
	std::vector<SizeRange> enumSizes(unsigned int pixelFormat);

	int requestBuffers(unsigned int count);
	int createPlane(BufferMemory *buffer, unsigned int index,
			unsigned int plane, unsigned int length);
	FileDescriptor exportDmabufFd(unsigned int index, unsigned int plane);

	Buffer *dequeueBuffer();
	void bufferAvailable(EventNotifier *notifier);

	V4L2Capability caps_;

	enum v4l2_buf_type bufferType_;
	enum v4l2_memory memoryType_;

	BufferPool *bufferPool_;
	std::map<unsigned int, Buffer *> queuedBuffers_;

	EventNotifier *fdEvent_;
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

#endif /* __LIBCAMERA_V4L2_VIDEODEVICE_H__ */
