/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * rpi_stream.h - Raspberry Pi device stream abstraction class.
 */

#pragma once

#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libcamera/base/flags.h>

#include <libcamera/stream.h>

#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

namespace RPi {

enum BufferMask {
	MaskID			= 0x00ffff,
	MaskStats		= 0x010000,
	MaskEmbeddedData	= 0x020000,
	MaskBayerData		= 0x040000,
	MaskExternalBuffer	= 0x100000,
};

struct BufferObject {
	BufferObject(FrameBuffer *b)
		: buffer(b), mapped(std::nullopt)
	{
	}

	BufferObject(FrameBuffer *b, libcamera::MappedFrameBuffer::MapFlags f)
		: buffer(b), mapped(std::make_optional<libcamera::MappedFrameBuffer>(b, f))
	{
	}

	FrameBuffer *buffer;
	std::optional<libcamera::MappedFrameBuffer> mapped;
};

using BufferMap = std::unordered_map<unsigned int, BufferObject>;

/*
 * Device stream abstraction for either an internal or external stream.
 * Used for both Unicam and the ISP.
 */
class Stream : public libcamera::Stream
{
public:
	enum Flags {
		None		= 0,
		ImportOnly	= (1 << 0),
		External	= (1 << 1),
		RequiresMmap	= (1 << 2),
		Config		= (1 << 3),
	};

	Stream()
		: flags_(Flags::None), id_(BufferMask::MaskID)
	{
	}

	Stream(const char *name, MediaEntity *dev, unsigned int flags = Flags::None)
		: flags_(flags), name_(name),
		  dev_(std::make_unique<V4L2VideoDevice>(dev)), id_(BufferMask::MaskID)
	{
	}

	void setFlags(unsigned int flags);
	void clearFlags(unsigned int flags);
	unsigned int getFlags() const;

	V4L2VideoDevice *dev() const;
	const std::string &name() const;
	void resetBuffers();

	void setExportedBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	const BufferMap &getBuffers() const;
	int getBufferId(FrameBuffer *buffer) const;

	void setExternalBuffer(FrameBuffer *buffer);
	void removeExternalBuffer(FrameBuffer *buffer);

	int prepareBuffers(unsigned int count);
	int queueBuffer(FrameBuffer *buffer);
	void returnBuffer(FrameBuffer *buffer);

	const BufferObject &getBuffer(int id);

	int queueAllBuffers();
	void releaseBuffers();

private:
	class IdGenerator
	{
	public:
		IdGenerator(int max)
			: max_(max), id_(1)
		{
		}

		int get()
		{
			int id;
			if (!recycle_.empty()) {
				id = recycle_.front();
				recycle_.pop();
			} else {
				id = id_++;
				ASSERT(id_ <= max_);
			}
			return id;
		}

		void release(int id)
		{
			recycle_.push(id);
		}

		void reset()
		{
			id_ = 1;
			recycle_ = {};
		}

	private:
		int max_;
		int id_;
		std::queue<int> recycle_;
	};

	void bufferEmplace(unsigned int id, FrameBuffer *buffer);
	void clearBuffers();
	int queueToDevice(FrameBuffer *buffer);

	unsigned int flags_;

	/* Stream name identifier. */
	std::string name_;

	/* The actual device stream. */
	std::unique_ptr<V4L2VideoDevice> dev_;

	/* Tracks a unique id key for the bufferMap_ */
	IdGenerator id_;

	/* All frame buffers associated with this device stream. */
	BufferMap bufferMap_;

	/*
	 * List of frame buffers that we can use if none have been provided by
	 * the application for external streams. This is populated by the
	 * buffers exported internally.
	 */
	std::queue<FrameBuffer *> availableBuffers_;

	/*
	 * List of frame buffers that are to be queued into the device from a Request.
	 * A nullptr indicates any internal buffer can be used (from availableBuffers_),
	 * whereas a valid pointer indicates an external buffer to be queued.
	 *
	 * Ordering buffers to be queued is important here as it must match the
	 * requests coming from the application.
	 */
	std::queue<FrameBuffer *> requestBuffers_;

	/*
	 * This is a list of buffers exported internally. Need to keep this around
	 * as the stream needs to maintain ownership of these buffers.
	 */
	std::vector<std::unique_ptr<FrameBuffer>> internalBuffers_;
};

/*
 * The following class is just a convenient (and typesafe) array of device
 * streams indexed with an enum class.
 */
template<typename E, std::size_t N>
class Device : public std::array<class Stream, N>
{
private:
	constexpr auto index(E e) const noexcept
	{
		return static_cast<std::underlying_type_t<E>>(e);
	}
public:
	Stream &operator[](E e)
	{
		return std::array<class Stream, N>::operator[](index(e));
	}
	const Stream &operator[](E e) const
	{
		return std::array<class Stream, N>::operator[](index(e));
	}
};

} /* namespace RPi */

} /* namespace libcamera */
