/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpi_stream.h - Raspberry Pi device stream abstraction class.
 */
#ifndef __LIBCAMERA_PIPELINE_RPI_STREAM_H__
#define __LIBCAMERA_PIPELINE_RPI_STREAM_H__

#include <queue>
#include <string>
#include <vector>

#include <libcamera/stream.h>

#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

namespace RPi {

/*
 * Device stream abstraction for either an internal or external stream.
 * Used for both Unicam and the ISP.
 */
class RPiStream : public Stream
{
public:
	RPiStream()
	{
	}

	RPiStream(const char *name, MediaEntity *dev, bool importOnly = false)
		: external_(false), importOnly_(importOnly), name_(name),
		  dev_(std::make_unique<V4L2VideoDevice>(dev))
	{
	}

	V4L2VideoDevice *dev() const;
	std::string name() const;
	bool isImporter() const;
	void reset();

	void setExternal(bool external);
	bool isExternal() const;

	void setExternalBuffers(std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	const std::vector<std::unique_ptr<FrameBuffer>> *getBuffers() const;
	bool findFrameBuffer(FrameBuffer *buffer) const;

	int importBuffers(unsigned int count);
	int allocateBuffers(unsigned int count);

	int queueBuffers();
	void releaseBuffers();

private:
	/*
	 * Indicates that this stream is active externally, i.e. the buffers
	 * are provided by the application.
	 */
	bool external_;

	/* Indicates that this stream only imports buffers, e.g. ISP input. */
	bool importOnly_;

	/* Stream name identifier. */
	std::string name_;

	/* The actual device stream. */
	std::unique_ptr<V4L2VideoDevice> dev_;

	/* Internally allocated framebuffers associated with this device stream. */
	std::vector<std::unique_ptr<FrameBuffer>> internalBuffers_;

	/* Externally allocated framebuffers associated with this device stream. */
	std::vector<std::unique_ptr<FrameBuffer>> *externalBuffers_;
};

/*
 * The following class is just a convenient (and typesafe) array of device
 * streams indexed with an enum class.
 */
template<typename E, std::size_t N>
class RPiDevice : public std::array<class RPiStream, N>
{
private:
	constexpr auto index(E e) const noexcept
	{
		return static_cast<std::underlying_type_t<E>>(e);
	}
public:
	RPiStream &operator[](E e)
	{
		return std::array<class RPiStream, N>::operator[](index(e));
	}
	const RPiStream &operator[](E e) const
	{
		return std::array<class RPiStream, N>::operator[](index(e));
	}
};

} /* namespace RPi */

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_RPI_STREAM_H__ */
