/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * stream.cpp - Video stream for a Camera
 */

#include <libcamera/stream.h>

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

} /* namespace libcamera */
