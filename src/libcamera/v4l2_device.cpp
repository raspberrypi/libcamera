/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_device.cpp - V4L2 Device
 */

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "log.h"
#include "media_object.h"
#include "v4l2_device.h"

/**
 * \file v4l2_device.h
 * \brief V4L2 Device API
 */
namespace libcamera {

LOG_DEFINE_CATEGORY(V4L2)

/**
 * \struct V4L2Capability
 * \brief struct v4l2_capability object wrapper and helpers
 *
 * The V4L2Capability structure manages the information returned by the
 * VIDIOC_QUERYCAP ioctl.
 */

/**
 * \fn const char *V4L2Capability::driver()
 * \brief Retrieve the driver module name
 * \return The string containing the name of the driver module
 */

/**
 * \fn const char *V4L2Capability::card()
 * \brief Retrieve the device card name
 * \return The string containing the device name
 */

/**
 * \fn const char *V4L2Capability::bus_info()
 * \brief Retrieve the location of the device in the system
 * \return The string containing the device location
 */

/**
 * \fn unsigned int V4L2Capability::device_caps()
 * \brief Retrieve the capabilities of the device
 * \return The device specific capabilities if V4L2_CAP_DEVICE_CAPS is set or
 * 	   driver capabilities otherwise
 */

/**
 * \fn bool V4L2Capability::isMultiplanar()
 * \brief Identify if the device implements the V4L2 multiplanar APIs
 * \return True if the device supports multiplanar APIs
 */

/**
 * \fn bool V4L2Capability::isCapture()
 * \brief Identify if the device is capable of capturing video
 * \return True if the device can capture video frames
 */

/**
 * \fn bool V4L2Capability::isOutput()
 * \brief Identify if the device is capable of outputting video
 * \return True if the device can output video frames
 */

/**
 * \fn bool V4L2Capability::hasStreaming()
 * \brief Determine if the device can perform Streaming I/O
 * \return True if the device provides Streaming I/O IOCTLs
 */

/**
 * \class V4L2DeviceFormat
 * \brief The V4L2 device image format and sizes
 *
 * Describes the image format and image sizes to be programmed on a V4L2
 * video device. The image format is defined by fourcc code as defined by
 * the V4L2 APIs with the V4L2_PIX_FMT_ macros, a visible width and height
 * and a variable number of planes (1 to 3) with variable sizes and line
 * strides.
 *
 * Formats defined as 'single planar' by the V4L2 APIs are represented with
 * V4L2DeviceFormat instances with a single plane
 * (V4L2DeviceFormat::planes = 1). Semi-planar and multiplanar formats use
 * 2 and 3 planes respectively.
 *
 * V4L2DeviceFormat defines the exchange format between components that
 * receive image configuration requests from applications and a V4L2Device.
 * The V4L2Device validates and applies the requested size and format to
 * the device driver.
 */

/**
 * \var V4L2DeviceFormat::width
 * \brief The image width
 */

/**
 * \var V4L2DeviceFormat::height
 * \brief The image height
 */

/**
 * \var V4L2DeviceFormat::fourcc
 * \brief The pixel encoding scheme
 *
 * The fourcc code, as defined by the V4L2 APIs with the V4L2_PIX_FMT_ macros,
 * that identifies the image format pixel encoding scheme.
 */

/**
 * \var V4L2DeviceFormat::planesFmt
 * \brief The per-plane size information
 *
 * Images are stored in memory in one or more data planes. Each data plane
 * has a specific size and line length, which could differ from the image
 * visible sizes to accommodate line or plane padding data.
 *
 * Only the first V4L2DeviceFormat::planes entries are considered valid.
 *
 */

/**
 * \var V4L2DeviceFormat::planes
 * \brief The number of valid data planes
 */

/**
 * \class V4L2Device
 * \brief V4L2Device object and API
 *
 * The V4L2 Device API class models an instance of a V4L2 device node.
 * It is constructed with the path to a V4L2 video device node. The device node
 * is only opened upon a call to open() which must be checked for success.
 *
 * The device capabilities are validated when the device is opened and the
 * device is rejected if it is not a suitable V4L2 capture or output device, or
 * if the device does not support streaming I/O.
 *
 * No API call other than open(), isOpen() and close() shall be called on an
 * unopened device instance.
 *
 * Upon destruction any device left open will be closed, and any resources
 * released.
 */

/**
 * \brief Construct a V4L2Device
 * \param deviceNode The file-system path to the video device node
 */
V4L2Device::V4L2Device(const std::string &deviceNode)
	: deviceNode_(deviceNode), fd_(-1)
{
}

/**
 * \brief Construct a V4L2Device from a MediaEntity
 * \param entity The MediaEntity to build the device from
 *
 * Construct a V4L2Device from a MediaEntity's device node path.
 */
V4L2Device::V4L2Device(const MediaEntity &entity)
	: V4L2Device(entity.deviceNode())
{
}

V4L2Device::~V4L2Device()
{
	close();
}

/**
 * \brief Open a V4L2 device and query its capabilities
 * \return 0 on success, or a negative error code otherwise
 */
int V4L2Device::open()
{
	int ret;

	if (isOpen()) {
		LOG(V4L2, Error) << "Device already open";
		return -EBUSY;
	}

	ret = ::open(deviceNode_.c_str(), O_RDWR);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to open V4L2 device '" << deviceNode_
			<< "': " << strerror(-ret);
		return ret;
	}
	fd_ = ret;

	ret = ioctl(fd_, VIDIOC_QUERYCAP, &caps_);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to query device capabilities: "
			<< strerror(-ret);
		return ret;
	}

	LOG(V4L2, Debug)
		<< "Opened '" << deviceNode_ << "' "
		<< caps_.bus_info() << ": " << caps_.driver()
		<< ": " << caps_.card();

	if (!caps_.isCapture() && !caps_.isOutput()) {
		LOG(V4L2, Debug) << "Device is not a supported type";
		return -EINVAL;
	}

	if (!caps_.hasStreaming()) {
		LOG(V4L2, Error) << "Device does not support streaming I/O";
		return -EINVAL;
	}

	if (caps_.isCapture())
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_CAPTURE;
	else
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_OUTPUT;

	return 0;
}

/**
 * \brief Check if device is successfully opened
 * \return True if the device is open, false otherwise
 */
bool V4L2Device::isOpen() const
{
	return fd_ != -1;
}

/**
 * \brief Close the device, releasing any resources acquired by open()
 */
void V4L2Device::close()
{
	if (fd_ < 0)
		return;

	::close(fd_);
	fd_ = -1;
}

/**
 * \fn const char *V4L2Device::driverName()
 * \brief Retrieve the name of the V4L2 device driver
 * \return The string containing the driver name
 */

/**
 * \fn const char *V4L2Device::deviceName()
 * \brief Retrieve the name of the V4L2 device
 * \return The string containing the device name
 */

/**
 * \fn const char *V4L2Device::busName()
 * \brief Retrieve the location of the device in the system
 * \return The string containing the device location
 */

/**
 * \brief Retrieve the image format set on the V4L2 device
 * \return 0 for success, a negative error code otherwise
 */
int V4L2Device::getFormat(V4L2DeviceFormat *fmt)
{
	return caps_.isMultiplanar() ? getFormatMultiplane(fmt) :
				       getFormatSingleplane(fmt);
}

/**
 * \brief Configure an image format on the V4L2 device
 * \return 0 for success, a negative error code otherwise
 */
int V4L2Device::setFormat(V4L2DeviceFormat *fmt)
{
	return caps_.isMultiplanar() ? setFormatMultiplane(fmt) :
				       setFormatSingleplane(fmt);
}

int V4L2Device::getFormatSingleplane(V4L2DeviceFormat *fmt)
{
	struct v4l2_format v4l2Fmt;
	struct v4l2_pix_format *pix = &v4l2Fmt.fmt.pix;
	int ret;

	v4l2Fmt.type = bufferType_;
	ret = ioctl(fd_, VIDIOC_G_FMT, &v4l2Fmt);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	fmt->width = pix->width;
	fmt->height = pix->height;
	fmt->fourcc = pix->pixelformat;
	fmt->planes = 1;
	fmt->planesFmt[0].bpl = pix->bytesperline;
	fmt->planesFmt[0].size = pix->sizeimage;

	return 0;
}

int V4L2Device::setFormatSingleplane(V4L2DeviceFormat *fmt)
{
	struct v4l2_format v4l2Fmt;
	struct v4l2_pix_format *pix = &v4l2Fmt.fmt.pix;
	int ret;

	v4l2Fmt.type = bufferType_;
	pix->width = fmt->width;
	pix->height = fmt->height;
	pix->pixelformat = fmt->fourcc;

	ret = ioctl(fd_, VIDIOC_S_FMT, &v4l2Fmt);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	return 0;
}

int V4L2Device::getFormatMultiplane(V4L2DeviceFormat *fmt)
{
	struct v4l2_format v4l2Fmt;
	struct v4l2_pix_format_mplane *pix = &v4l2Fmt.fmt.pix_mp;
	int ret;

	v4l2Fmt.type = bufferType_;
	ret = ioctl(fd_, VIDIOC_G_FMT, &v4l2Fmt);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	fmt->width = pix->width;
	fmt->height = pix->height;
	fmt->fourcc = pix->pixelformat;
	fmt->planes = pix->num_planes;

	for (unsigned int i = 0; i < fmt->planes; ++i) {
		fmt->planesFmt[i].bpl = pix->plane_fmt[i].bytesperline;
		fmt->planesFmt[i].size = pix->plane_fmt[i].sizeimage;
	}

	return 0;
}

int V4L2Device::setFormatMultiplane(V4L2DeviceFormat *fmt)
{
	struct v4l2_format v4l2Fmt;
	struct v4l2_pix_format_mplane *pix = &v4l2Fmt.fmt.pix_mp;
	int ret;

	v4l2Fmt.type = bufferType_;
	pix->width = fmt->width;
	pix->height = fmt->height;
	pix->pixelformat = fmt->fourcc;
	pix->num_planes = fmt->planes;

	for (unsigned int i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = fmt->planesFmt[i].bpl;
		pix->plane_fmt[i].sizeimage = fmt->planesFmt[i].size;
	}

	ret = ioctl(fd_, VIDIOC_S_FMT, &v4l2Fmt);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	return 0;
}

} /* namespace libcamera */
