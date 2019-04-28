/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.cpp - Video stream for a Camera
 */

#include <libcamera/stream.h>

#include <iomanip>
#include <sstream>

/**
 * \file stream.h
 * \brief Video stream for a Camera
 *
 * A camera device can provide frames in different resolutions and formats
 * concurrently from a single image source. The Stream class represents
 * one of the multiple concurrent streams.
 *
 * All streams exposed by a camera device share the same image source and are
 * thus not fully independent. Parameters related to the image source, such as
 * the exposure time or flash control, are common to all streams. Other
 * parameters, such as format or resolution, may be specified per-stream,
 * depending on the capabilities of the camera device.
 *
 * Camera devices expose at least one stream, and may expose additional streams
 * based on the device capabilities. This can be used, for instance, to
 * implement concurrent viewfinder and video capture, or concurrent viewfinder,
 * video capture and still image capture.
 */

namespace libcamera {

/**
 * \struct StreamConfiguration
 * \brief Configuration parameters for a stream
 *
 * The StreamConfiguration structure models all information which can be
 * configured for a single video stream.
 */

/**
 * \var StreamConfiguration::size
 * \brief Stream size in pixels
 */

/**
 * \var StreamConfiguration::pixelFormat
 * \brief Stream pixel format
 *
 * This is a little endian four character code representation of the pixel
 * format described in V4L2 using the V4L2_PIX_FMT_* definitions.
 */

/**
 * \var StreamConfiguration::bufferCount
 * \brief Requested number of buffers to allocate for the stream
 */

/**
 * \brief Assemble and return a string describing the configuration
 *
 * \return A string describing the StreamConfiguration
 */
std::string StreamConfiguration::toString() const
{
	std::stringstream ss;

	ss.fill(0);
	ss << size.toString() << "-0x" << std::hex << std::setw(8)
	   << pixelFormat;

	return ss.str();
}

/**
 * \class StreamUsage
 * \brief Stream usage information
 *
 * The StreamUsage class describes how an application intends to use a stream.
 * Usages are specified by applications and passed to cameras, that then select
 * the most appropriate streams and their default configurations.
 */

/**
 * \enum StreamUsage::Role
 * \brief Identify the role a stream is intended to play
 * \var StreamUsage::StillCapture
 * The stream is intended to capture high-resolution, high-quality still images
 * with low frame rate. The captured frames may be exposed with flash.
 * \var StreamUsage::VideoRecording
 * The stream is intended to capture video for the purpose of recording or
 * streaming. The video stream may produce a high frame rate and may be
 * enhanced with video stabilization.
 * \var StreamUsage::Viewfinder
 * The stream is intended to capture video for the purpose of display on the
 * local screen. The StreamUsage includes the desired resolution. Trade-offs
 * between quality and usage of system resources are acceptable.
 */

/**
 * \fn StreamUsage::role()
 * \brief Retrieve the stream role
 * \return The stream role
 */

/**
 * \fn StreamUsage::size()
 * \brief Retrieve desired size
 * \return The desired size
 */

/**
 * \brief Create a stream usage
 * \param[in] role Stream role
 */
StreamUsage::StreamUsage(Role role)
	: role_(role)
{
}

/**
 * \brief Create a stream usage with a desired size
 * \param[in] role Stream role
 * \param[in] size The desired size
 */
StreamUsage::StreamUsage(Role role, const Size &size)
	: role_(role), size_(size)
{
}

/**
 * \class Stream
 * \brief Video stream for a camera
 *
 * The Stream class models all static information which are associated with a
 * single video stream. Streams are exposed by the Camera object they belong to.
 *
 * Cameras may supply more than one stream from the same video source. In such
 * cases an application can inspect all available streams and select the ones
 * that best fit its use case.
 *
 * \todo Add capabilities to the stream API. Without this the Stream class only
 * serves to reveal how many streams of unknown capabilities a camera supports.
 * This in itself is productive as it allows applications to configure and
 * capture from one or more streams even if they won't be able to select the
 * optimal stream for the task.
 */

/**
 * \class Stream::StillCapture
 * \brief Describe a still capture usage
 */
Stream::StillCapture::StillCapture()
	: StreamUsage(Role::StillCapture)
{
}

/**
 * \class Stream::VideoRecording
 * \brief Describe a video recording usage
 */
Stream::VideoRecording::VideoRecording()
	: StreamUsage(Role::VideoRecording)
{
}

/**
 * \class Stream::Viewfinder
 * \brief Describe a viewfinder usage
 */

/**
 * \brief Create a viewfinder usage with a desired dimension
 * \param[in] width The desired viewfinder width
 * \param[in] height The desired viewfinder height
 */
Stream::Viewfinder::Viewfinder(int width, int height)
	: StreamUsage(Role::Viewfinder, Size(width, height))
{
}

/**
 * \brief Construct a stream with default parameters
 */
Stream::Stream()
{
}

/**
 * \fn Stream::bufferPool()
 * \brief Retrieve the buffer pool for the stream
 *
 * The buffer pool handles the buffers used to capture frames at the output of
 * the stream. It is initially created empty and shall be populated with
 * buffers before being used.
 *
 * \return A reference to the buffer pool
 */

/**
 * \fn Stream::configuration()
 * \brief Retrieve the active configuration of the stream
 * \return The active configuration of the stream
 */

/**
 * \var Stream::bufferPool_
 * \brief The pool of buffers associated with the stream
 *
 * The stream buffer pool is populated by the Camera class after a succesfull
 * stream configuration.
 */

/**
 * \var Stream::configuration_
 * \brief The stream configuration
 *
 * The configuration for the stream is set by any successful call to
 * Camera::configure() that includes the stream, and remains valid until the
 * next call to Camera::configure() regardless of if it includes the stream.
 */

} /* namespace libcamera */
