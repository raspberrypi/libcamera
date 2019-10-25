/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_videodevice.cpp - V4L2 Video Device
 */

#include "v4l2_videodevice.h"

#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include <linux/drm_fourcc.h>

#include <libcamera/buffer.h>
#include <libcamera/event_notifier.h>

#include "log.h"
#include "media_device.h"
#include "media_object.h"
#include "utils.h"

/**
 * \file v4l2_videodevice.h
 * \brief V4L2 Video Device
 */
namespace libcamera {

LOG_DECLARE_CATEGORY(V4L2)

/**
 * \struct V4L2Capability
 * \brief struct v4l2_capability object wrapper and helpers
 *
 * The V4L2Capability structure manages the information returned by the
 * VIDIOC_QUERYCAP ioctl.
 */

/**
 * \fn V4L2Capability::driver()
 * \brief Retrieve the driver module name
 * \return The string containing the name of the driver module
 */

/**
 * \fn V4L2Capability::card()
 * \brief Retrieve the video device card name
 * \return The string containing the video device name
 */

/**
 * \fn V4L2Capability::bus_info()
 * \brief Retrieve the location of the video device in the system
 * \return The string containing the video device location
 */

/**
 * \fn V4L2Capability::device_caps()
 * \brief Retrieve the capabilities of the video device
 * \return The video device specific capabilities if V4L2_CAP_DEVICE_CAPS is
 * set or driver capabilities otherwise
 */

/**
 * \fn V4L2Capability::isMultiplanar()
 * \brief Identify if the video device implements the V4L2 multiplanar APIs
 * \return True if the video device supports multiplanar APIs
 */

/**
 * \fn V4L2Capability::isCapture()
 * \brief Identify if the video device captures data
 * \return True if the video device can capture data
 */

/**
 * \fn V4L2Capability::isOutput()
 * \brief Identify if the video device outputs data
 * \return True if the video device can output data
 */

/**
 * \fn V4L2Capability::isVideo()
 * \brief Identify if the video device captures or outputs images
 * \return True if the video device can capture or output images
 */

/**
 * \fn V4L2Capability::isM2M()
 * \brief Identify if the device is a Memory-to-Memory device
 * \return True if the device can capture and output images using the M2M API
 */

/**
 * \fn V4L2Capability::isMeta()
 * \brief Identify if the video device captures or outputs image meta-data
 * \return True if the video device can capture or output image meta-data
 */

/**
 * \fn V4L2Capability::isVideoCapture()
 * \brief Identify if the video device captures images
 * \return True if the video device can capture images
 */

/**
 * \fn V4L2Capability::isVideoOutput()
 * \brief Identify if the video device outputs images
 * \return True if the video device can output images
 */

/**
 * \fn V4L2Capability::isMetaCapture()
 * \brief Identify if the video device captures image meta-data
 * \return True if the video device can capture image meta-data
 */

/**
 * \fn V4L2Capability::isMetaOutput()
 * \brief Identify if the video device outputs image meta-data
 * \return True if the video device can output image meta-data
 */

/**
 * \fn V4L2Capability::hasStreaming()
 * \brief Determine if the video device can perform Streaming I/O
 * \return True if the video device provides Streaming I/O IOCTLs
 */

/**
 * \class V4L2DeviceFormat
 * \brief The V4L2 video device image format and sizes
 *
 * This class describes the image format and resolution to be programmed on a
 * V4L2 video device. The image format is defined by a fourcc code (as specified
 * by the V4L2 API with the V4L2_PIX_FMT_* macros), a resolution (width and
 * height) and one to three planes with configurable line stride and a total
 * per-plane size in bytes.
 *
 * Image formats, as defined by the V4L2 APIs, are categorised as packed,
 * semi-planar and planar, and describe the layout of the image pixel components
 * stored in memory.
 *
 * Packed image formats store pixel components one after the other, in a
 * contiguous memory area. Examples of packed image formats are YUYV
 * permutations, RGB with different pixel sub-sampling ratios such as RGB565 or
 * RGB666 or Raw-Bayer formats such as SRGGB8 or SGRBG12.
 *
 * Semi-planar and planar image formats store the pixel components in separate
 * and possibly non-contiguous memory areas, named planes, whose sizes depend on
 * the pixel components sub-sampling ratios, which are defined by the format.
 * Semi-planar formats use two planes to store pixel components and notable
 * examples of such formats are the NV12 and NV16 formats, while planar formats
 * use three planes to store pixel components and notable examples are YUV422
 * and YUV420.
 *
 * Image formats supported by the V4L2 API are defined and described in Section
 * number 2 of the "Part I - Video for Linux API" chapter of the "Linux Media
 * Infrastructure userspace API", part of the Linux kernel documentation.
 *
 * In the context of this document, packed image formats are referred to as
 * "packed formats" and semi-planar and planar image formats are referred to as
 * "planar formats".
 *
 * V4L2 also defines two different sets of APIs to work with devices that store
 * planes in contiguous or separate memory areas. They are named "Single-plane
 * APIs" and "Multi-plane APIs" respectively and are documented in Section 2.1
 * and Section 2.2 of the above mentioned "Part I - Video for Linux API"
 * documentation.
 *
 * The single-plane API allows, among other parameters, the configuration of the
 * image resolution, the pixel format and the stride length. In that case the
 * stride applies to all planes (possibly sub-sampled). The multi-plane API
 * allows configuring the resolution, the pixel format and a per-plane stride
 * length and total size.
 *
 * Packed image formats, which occupy a single memory area, are easily described
 * through the single-plane API. When used on a video device that implements the
 * multi-plane API, only the size and stride information contained in the first
 * plane are taken into account.
 *
 * Planar image formats, which occupy distinct memory areas, are easily
 * described through the multi-plane APIs. When used on a video device that
 * implements the single-plane API, all planes are stored one after the other
 * in a contiguous memory area, and it is not possible to configure per-plane
 * stride length and size, but only a global stride length which is applied to
 * all planes.
 *
 * The V4L2DeviceFormat class describes both packed and planar image formats,
 * regardless of the API type (single or multi plane) implemented by the video
 * device the format has to be applied to. The total size and bytes per line
 * of images represented with packed formats are configured using the first
 * entry of the V4L2DeviceFormat::planes array, while the per-plane size and
 * per-plane stride length of images represented with planar image formats are
 * configured using the opportune number of entries of the
 * V4L2DeviceFormat::planes array, as prescribed by the image format
 * definition (semi-planar formats use 2 entries, while planar formats use the
 * whole 3 entries). The number of valid entries of the
 * V4L2DeviceFormat::planes array is defined by the
 * V4L2DeviceFormat::planesCount value.
 */

/**
 * \var V4L2DeviceFormat::size
 * \brief The image size in pixels
 */

/**
 * \var V4L2DeviceFormat::fourcc
 * \brief The fourcc code describing the pixel encoding scheme
 *
 * The fourcc code, as defined by the V4L2 API with the V4L2_PIX_FMT_* macros,
 * that identifies the image format pixel encoding scheme.
 */

/**
 * \var V4L2DeviceFormat::planes
 * \brief The per-plane memory size information
 *
 * Images are stored in memory in one or more data planes. Each data plane has a
 * specific line stride and memory size, which could differ from the image
 * visible sizes to accommodate padding at the end of lines and end of planes.
 * Only the first \ref planesCount entries are considered valid.
 */

/**
 * \var V4L2DeviceFormat::planesCount
 * \brief The number of valid data planes
 */

/**
 * \brief Assemble and return a string describing the format
 * \return A string describing the V4L2DeviceFormat
 */
const std::string V4L2DeviceFormat::toString() const
{
	std::stringstream ss;
	ss << size.toString() << "-" << utils::hex(fourcc);
	return ss.str();
}

/**
 * \class V4L2VideoDevice
 * \brief V4L2VideoDevice object and API
 *
 * The V4L2VideoDevice class models an instance of a V4L2 video device.
 * It is constructed with the path to a V4L2 video device node. The device node
 * is only opened upon a call to open() which must be checked for success.
 *
 * The video device capabilities are validated when the device is opened and the
 * device is rejected if it is not a suitable V4L2 capture or output video
 * device, or if the video device does not support streaming I/O.
 *
 * No API call other than open(), isOpen() and close() shall be called on an
 * unopened device instance.
 *
 * The V4L2VideoDevice class tracks queued buffers and handles buffer events. It
 * automatically dequeues completed buffers and emits the \ref bufferReady
 * signal.
 *
 * Upon destruction any device left open will be closed, and any resources
 * released.
 */

/**
 * \brief Construct a V4L2VideoDevice
 * \param[in] deviceNode The file-system path to the video device node
 */
V4L2VideoDevice::V4L2VideoDevice(const std::string &deviceNode)
	: V4L2Device(deviceNode), bufferPool_(nullptr), fdEvent_(nullptr)
{
	/*
	 * We default to an MMAP based CAPTURE video device, however this will
	 * be updated based upon the device capabilities.
	 */
	bufferType_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	memoryType_ = V4L2_MEMORY_MMAP;
}

/**
 * \brief Construct a V4L2VideoDevice from a MediaEntity
 * \param[in] entity The MediaEntity to build the video device from
 *
 * Construct a V4L2VideoDevice from a MediaEntity's device node path.
 */
V4L2VideoDevice::V4L2VideoDevice(const MediaEntity *entity)
	: V4L2VideoDevice(entity->deviceNode())
{
}

V4L2VideoDevice::~V4L2VideoDevice()
{
	close();
}

/**
 * \brief Open the V4L2 video device node and query its capabilities
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::open()
{
	int ret;

	ret = V4L2Device::open(O_RDWR | O_NONBLOCK);
	if (ret < 0)
		return ret;

	ret = ioctl(VIDIOC_QUERYCAP, &caps_);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to query device capabilities: "
			<< strerror(-ret);
		return ret;
	}

	if (!caps_.hasStreaming()) {
		LOG(V4L2, Error) << "Device does not support streaming I/O";
		return -EINVAL;
	}

	/*
	 * Set buffer type and wait for read notifications on CAPTURE video
	 * devices (POLLIN), and write notifications for OUTPUT video devices
	 * (POLLOUT).
	 */
	if (caps_.isVideoCapture()) {
		fdEvent_ = new EventNotifier(fd(), EventNotifier::Read);
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else if (caps_.isVideoOutput()) {
		fdEvent_ = new EventNotifier(fd(), EventNotifier::Write);
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_OUTPUT;
	} else if (caps_.isMetaCapture()) {
		fdEvent_ = new EventNotifier(fd(), EventNotifier::Read);
		bufferType_ = V4L2_BUF_TYPE_META_CAPTURE;
	} else if (caps_.isMetaOutput()) {
		fdEvent_ = new EventNotifier(fd(), EventNotifier::Write);
		bufferType_ = V4L2_BUF_TYPE_META_OUTPUT;
	} else {
		LOG(V4L2, Error) << "Device is not a supported type";
		return -EINVAL;
	}

	fdEvent_->activated.connect(this, &V4L2VideoDevice::bufferAvailable);
	fdEvent_->setEnabled(false);

	LOG(V4L2, Debug)
		<< "Opened device " << caps_.bus_info() << ": "
		<< caps_.driver() << ": " << caps_.card();

	return 0;
}

/**
 * \brief Open a V4L2 video device from an opened file handle and query its
 * capabilities
 * \param[in] handle The file descriptor to set
 * \param[in] type The device type to operate on
 *
 * This methods opens a video device from the existing file descriptor \a
 * handle. Like open(), this method queries the capabilities of the device, but
 * handles it according to the given device \a type instead of determining its
 * type from the capabilities. This can be used to force a given device type for
 * memory-to-memory devices.
 *
 * The file descriptor \a handle is duplicated, and the caller is responsible
 * for closing the \a handle when it has no further use for it. The close()
 * method will close the duplicated file descriptor, leaving \a handle
 * untouched.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::open(int handle, enum v4l2_buf_type type)
{
	int ret;
	int newFd;

	newFd = dup(handle);
	if (newFd < 0) {
		ret = -errno;
		LOG(V4L2, Error) << "Failed to duplicate file handle: "
				 << strerror(-ret);
		return ret;
	}

	ret = V4L2Device::setFd(newFd);
	if (ret < 0) {
		LOG(V4L2, Error) << "Failed to set file handle: "
				 << strerror(-ret);
		::close(newFd);
		return ret;
	}

	ret = ioctl(VIDIOC_QUERYCAP, &caps_);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to query device capabilities: "
			<< strerror(-ret);
		return ret;
	}

	if (!caps_.hasStreaming()) {
		LOG(V4L2, Error) << "Device does not support streaming I/O";
		return -EINVAL;
	}

	/*
	 * Set buffer type and wait for read notifications on CAPTURE video
	 * devices (POLLIN), and write notifications for OUTPUT video devices
	 * (POLLOUT).
	 */
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		fdEvent_ = new EventNotifier(fd(), EventNotifier::Write);
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		fdEvent_ = new EventNotifier(fd(), EventNotifier::Read);
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_CAPTURE;
		break;
	default:
		LOG(V4L2, Error) << "Unsupported buffer type";
		return -EINVAL;
	}

	fdEvent_->activated.connect(this, &V4L2VideoDevice::bufferAvailable);
	fdEvent_->setEnabled(false);

	LOG(V4L2, Debug)
		<< "Opened device " << caps_.bus_info() << ": "
		<< caps_.driver() << ": " << caps_.card();

	return 0;
}

/**
 * \brief Close the video device, releasing any resources acquired by open()
 */
void V4L2VideoDevice::close()
{
	if (!isOpen())
		return;

	releaseBuffers();
	delete fdEvent_;

	V4L2Device::close();
}

/**
 * \fn V4L2VideoDevice::driverName()
 * \brief Retrieve the name of the V4L2 device driver
 * \return The string containing the driver name
 */

/**
 * \fn V4L2VideoDevice::deviceName()
 * \brief Retrieve the name of the V4L2 video device
 * \return The string containing the device name
 */

/**
 * \fn V4L2VideoDevice::busName()
 * \brief Retrieve the location of the device in the system
 * \return The string containing the device location
 */

std::string V4L2VideoDevice::logPrefix() const
{
	return deviceNode() + (V4L2_TYPE_IS_OUTPUT(bufferType_) ? "[out]" : "[cap]");
}

/**
 * \brief Retrieve the image format set on the V4L2 video device
 * \param[out] format The image format applied on the video device
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::getFormat(V4L2DeviceFormat *format)
{
	if (caps_.isMeta())
		return getFormatMeta(format);
	else if (caps_.isMultiplanar())
		return getFormatMultiplane(format);
	else
		return getFormatSingleplane(format);
}

/**
 * \brief Configure an image format on the V4L2 video device
 * \param[inout] format The image format to apply to the video device
 *
 * Apply the supplied \a format to the video device, and return the actually
 * applied format parameters, as \ref V4L2VideoDevice::getFormat would do.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::setFormat(V4L2DeviceFormat *format)
{
	if (caps_.isMeta())
		return setFormatMeta(format);
	else if (caps_.isMultiplanar())
		return setFormatMultiplane(format);
	else
		return setFormatSingleplane(format);
}

int V4L2VideoDevice::getFormatMeta(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_meta_format *pix = &v4l2Format.fmt.meta;
	int ret;

	v4l2Format.type = bufferType_;
	ret = ioctl(VIDIOC_G_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	format->size.width = 0;
	format->size.height = 0;
	format->fourcc = pix->dataformat;
	format->planesCount = 1;
	format->planes[0].bpl = pix->buffersize;
	format->planes[0].size = pix->buffersize;

	return 0;
}

int V4L2VideoDevice::setFormatMeta(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_meta_format *pix = &v4l2Format.fmt.meta;
	int ret;

	v4l2Format.type = bufferType_;
	pix->dataformat = format->fourcc;
	pix->buffersize = format->planes[0].size;
	ret = ioctl(VIDIOC_S_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the video device,
	 * which might differ from the requested one.
	 */
	format->size.width = 0;
	format->size.height = 0;
	format->fourcc = format->fourcc;
	format->planesCount = 1;
	format->planes[0].bpl = pix->buffersize;
	format->planes[0].size = pix->buffersize;

	return 0;
}

int V4L2VideoDevice::getFormatMultiplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format_mplane *pix = &v4l2Format.fmt.pix_mp;
	int ret;

	v4l2Format.type = bufferType_;
	ret = ioctl(VIDIOC_G_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	format->size.width = pix->width;
	format->size.height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = pix->num_planes;

	for (unsigned int i = 0; i < format->planesCount; ++i) {
		format->planes[i].bpl = pix->plane_fmt[i].bytesperline;
		format->planes[i].size = pix->plane_fmt[i].sizeimage;
	}

	return 0;
}

int V4L2VideoDevice::setFormatMultiplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format_mplane *pix = &v4l2Format.fmt.pix_mp;
	int ret;

	v4l2Format.type = bufferType_;
	pix->width = format->size.width;
	pix->height = format->size.height;
	pix->pixelformat = format->fourcc;
	pix->num_planes = format->planesCount;
	pix->field = V4L2_FIELD_NONE;

	for (unsigned int i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = format->planes[i].bpl;
		pix->plane_fmt[i].sizeimage = format->planes[i].size;
	}

	ret = ioctl(VIDIOC_S_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the video device,
	 * which might differ from the requested one.
	 */
	format->size.width = pix->width;
	format->size.height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = pix->num_planes;
	for (unsigned int i = 0; i < format->planesCount; ++i) {
		format->planes[i].bpl = pix->plane_fmt[i].bytesperline;
		format->planes[i].size = pix->plane_fmt[i].sizeimage;
	}

	return 0;
}

int V4L2VideoDevice::getFormatSingleplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format *pix = &v4l2Format.fmt.pix;
	int ret;

	v4l2Format.type = bufferType_;
	ret = ioctl(VIDIOC_G_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	format->size.width = pix->width;
	format->size.height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = 1;
	format->planes[0].bpl = pix->bytesperline;
	format->planes[0].size = pix->sizeimage;

	return 0;
}

int V4L2VideoDevice::setFormatSingleplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format *pix = &v4l2Format.fmt.pix;
	int ret;

	v4l2Format.type = bufferType_;
	pix->width = format->size.width;
	pix->height = format->size.height;
	pix->pixelformat = format->fourcc;
	pix->bytesperline = format->planes[0].bpl;
	pix->field = V4L2_FIELD_NONE;
	ret = ioctl(VIDIOC_S_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the device,
	 * which might differ from the requested one.
	 */
	format->size.width = pix->width;
	format->size.height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = 1;
	format->planes[0].bpl = pix->bytesperline;
	format->planes[0].size = pix->sizeimage;

	return 0;
}

/**
 * \brief Enumerate all pixel formats and frame sizes
 *
 * Enumerate all pixel formats and frame sizes supported by the video device.
 *
 * \return A list of the supported video device formats
 */
ImageFormats V4L2VideoDevice::formats()
{
	ImageFormats formats;

	for (unsigned int pixelformat : enumPixelformats()) {
		std::vector<SizeRange> sizes = enumSizes(pixelformat);
		if (sizes.empty())
			return {};

		if (formats.addFormat(pixelformat, sizes)) {
			LOG(V4L2, Error)
				<< "Could not add sizes for pixel format "
				<< pixelformat;
			return {};
		}
	}

	return formats;
}

std::vector<unsigned int> V4L2VideoDevice::enumPixelformats()
{
	std::vector<unsigned int> formats;
	int ret;

	for (unsigned int index = 0; ; index++) {
		struct v4l2_fmtdesc pixelformatEnum = {};
		pixelformatEnum.index = index;
		pixelformatEnum.type = bufferType_;

		ret = ioctl(VIDIOC_ENUM_FMT, &pixelformatEnum);
		if (ret)
			break;

		formats.push_back(pixelformatEnum.pixelformat);
	}

	if (ret && ret != -EINVAL) {
		LOG(V4L2, Error)
			<< "Unable to enumerate pixel formats: "
			<< strerror(-ret);
		return {};
	}

	return formats;
}

std::vector<SizeRange> V4L2VideoDevice::enumSizes(unsigned int pixelFormat)
{
	std::vector<SizeRange> sizes;
	int ret;

	for (unsigned int index = 0;; index++) {
		struct v4l2_frmsizeenum frameSize = {};
		frameSize.index = index;
		frameSize.pixel_format = pixelFormat;

		ret = ioctl(VIDIOC_ENUM_FRAMESIZES, &frameSize);
		if (ret)
			break;

		if (index != 0 &&
		    frameSize.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
			LOG(V4L2, Error)
				<< "Non-zero index for non discrete type";
			return {};
		}

		switch (frameSize.type) {
		case V4L2_FRMSIZE_TYPE_DISCRETE:
			sizes.emplace_back(frameSize.discrete.width,
					   frameSize.discrete.height);
			break;
		case V4L2_FRMSIZE_TYPE_CONTINUOUS:
			sizes.emplace_back(frameSize.stepwise.min_width,
					   frameSize.stepwise.min_height,
					   frameSize.stepwise.max_width,
					   frameSize.stepwise.max_height);
			break;
		case V4L2_FRMSIZE_TYPE_STEPWISE:
			sizes.emplace_back(frameSize.stepwise.min_width,
					   frameSize.stepwise.min_height,
					   frameSize.stepwise.max_width,
					   frameSize.stepwise.max_height,
					   frameSize.stepwise.step_width,
					   frameSize.stepwise.step_height);
			break;
		default:
			LOG(V4L2, Error)
				<< "Unknown VIDIOC_ENUM_FRAMESIZES type "
				<< frameSize.type;
			return {};
		}
	}

	if (ret && ret != -EINVAL) {
		LOG(V4L2, Error)
			<< "Unable to enumerate frame sizes: "
			<< strerror(-ret);
		return {};
	}

	return sizes;
}

int V4L2VideoDevice::requestBuffers(unsigned int count)
{
	struct v4l2_requestbuffers rb = {};
	int ret;

	rb.count = count;
	rb.type = bufferType_;
	rb.memory = memoryType_;

	ret = ioctl(VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to request " << count << " buffers: "
			<< strerror(-ret);
		return ret;
	}

	LOG(V4L2, Debug) << rb.count << " buffers requested.";

	return rb.count;
}

/**
 * \brief Request buffers to be allocated from the video device and stored in
 * the buffer pool provided.
 * \param[out] pool BufferPool to populate with buffers
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::exportBuffers(BufferPool *pool)
{
	unsigned int allocatedBuffers;
	unsigned int i;
	int ret;

	memoryType_ = V4L2_MEMORY_MMAP;

	ret = requestBuffers(pool->count());
	if (ret < 0)
		return ret;

	allocatedBuffers = ret;
	if (allocatedBuffers < pool->count()) {
		LOG(V4L2, Error)
			<< "Not enough buffers provided by V4L2VideoDevice";
		requestBuffers(0);
		return -ENOMEM;
	}

	/* Map the buffers. */
	for (i = 0; i < pool->count(); ++i) {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
		struct v4l2_buffer buf = {};
		BufferMemory &buffer = pool->buffers()[i];

		buf.index = i;
		buf.type = bufferType_;
		buf.memory = memoryType_;
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;

		ret = ioctl(VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			LOG(V4L2, Error)
				<< "Unable to query buffer " << i << ": "
				<< strerror(-ret);
			break;
		}

		if (V4L2_TYPE_IS_MULTIPLANAR(buf.type)) {
			for (unsigned int p = 0; p < buf.length; ++p) {
				ret = createPlane(&buffer, i, p,
						  buf.m.planes[p].length);
				if (ret)
					break;
			}
		} else {
			ret = createPlane(&buffer, i, 0, buf.length);
		}

		if (ret) {
			LOG(V4L2, Error) << "Failed to create plane";
			break;
		}
	}

	if (ret) {
		requestBuffers(0);
		pool->destroyBuffers();
		return ret;
	}

	bufferPool_ = pool;

	return 0;
}

int V4L2VideoDevice::createPlane(BufferMemory *buffer, unsigned int index,
				 unsigned int planeIndex, unsigned int length)
{
	struct v4l2_exportbuffer expbuf = {};
	int ret;

	LOG(V4L2, Debug)
		<< "Buffer " << index
		<< " plane " << planeIndex
		<< ": length=" << length;

	expbuf.type = bufferType_;
	expbuf.index = index;
	expbuf.plane = planeIndex;
	expbuf.flags = O_RDWR;

	ret = ioctl(VIDIOC_EXPBUF, &expbuf);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to export buffer: " << strerror(-ret);
		return ret;
	}

	buffer->planes().emplace_back();
	Plane &plane = buffer->planes().back();
	plane.setDmabuf(expbuf.fd, length);
	::close(expbuf.fd);

	return 0;
}

/**
 * \brief Import the externally allocated \a pool of buffers
 * \param[in] pool BufferPool of buffers to import
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::importBuffers(BufferPool *pool)
{
	unsigned int allocatedBuffers;
	int ret;

	memoryType_ = V4L2_MEMORY_DMABUF;

	ret = requestBuffers(pool->count());
	if (ret < 0)
		return ret;

	allocatedBuffers = ret;
	if (allocatedBuffers < pool->count()) {
		LOG(V4L2, Error)
			<< "Not enough buffers provided by V4L2VideoDevice";
		requestBuffers(0);
		return -ENOMEM;
	}

	LOG(V4L2, Debug) << "provided pool of " << pool->count() << " buffers";
	bufferPool_ = pool;

	return 0;
}

/**
 * \brief Release all internally allocated buffers
 */
int V4L2VideoDevice::releaseBuffers()
{
	LOG(V4L2, Debug) << "Releasing bufferPool";

	bufferPool_ = nullptr;

	return requestBuffers(0);
}

/**
 * \brief Queue a buffer into the video device
 * \param[in] buffer The buffer to be queued
 *
 * For capture video devices the \a buffer will be filled with data by the
 * device. For output video devices the \a buffer shall contain valid data and
 * will be processed by the device. Once the device has finished processing the
 * buffer, it will be available for dequeue.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::queueBuffer(Buffer *buffer)
{
	struct v4l2_plane v4l2Planes[VIDEO_MAX_PLANES] = {};
	struct v4l2_buffer buf = {};
	int ret;

	buf.index = buffer->index();
	buf.type = bufferType_;
	buf.memory = memoryType_;
	buf.field = V4L2_FIELD_NONE;

	bool multiPlanar = V4L2_TYPE_IS_MULTIPLANAR(buf.type);
	BufferMemory *mem = &bufferPool_->buffers()[buf.index];
	const std::vector<Plane> &planes = mem->planes();

	if (buf.memory == V4L2_MEMORY_DMABUF) {
		if (multiPlanar) {
			for (unsigned int p = 0; p < planes.size(); ++p)
				v4l2Planes[p].m.fd = planes[p].dmabuf();
		} else {
			buf.m.fd = planes[0].dmabuf();
		}
	}

	if (multiPlanar) {
		buf.length = planes.size();
		buf.m.planes = v4l2Planes;
	}

	if (V4L2_TYPE_IS_OUTPUT(bufferType_)) {
		buf.bytesused = buffer->bytesused_;
		buf.sequence = buffer->sequence_;
		buf.timestamp.tv_sec = buffer->timestamp_ / 1000000000;
		buf.timestamp.tv_usec = (buffer->timestamp_ / 1000) % 1000000;
	}

	LOG(V4L2, Debug) << "Queueing buffer " << buf.index;

	ret = ioctl(VIDIOC_QBUF, &buf);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to queue buffer " << buf.index << ": "
			<< strerror(-ret);
		return ret;
	}

	if (queuedBuffers_.empty())
		fdEvent_->setEnabled(true);

	queuedBuffers_[buf.index] = buffer;

	return 0;
}

/**
 * \brief Queue all buffers into the video device
 *
 * When starting video capture users of the video device often need to queue
 * all allocated buffers to the device. This helper method simplifies the
 * implementation of the user by queuing all buffers and returning a vector of
 * Buffer instances for each queued buffer.
 *
 * This method is meant to be used with video capture devices internal to a
 * pipeline handler, such as ISP statistics capture devices, or raw CSI-2
 * receivers. For video capture devices facing applications, buffers shall
 * instead be queued when requests are received, and for video output devices,
 * buffers shall be queued when frames are ready to be output.
 *
 * The caller shall ensure that the returned buffers vector remains valid until
 * all the queued buffers are dequeued, either during capture, or by stopping
 * the video device.
 *
 * Calling this method on an output device or on a device that has buffers
 * already queued is an error and will return an empty vector.
 *
 * \return A vector of queued buffers, which will be empty if an error occurs
 */
std::vector<std::unique_ptr<Buffer>> V4L2VideoDevice::queueAllBuffers()
{
	int ret;

	if (!queuedBuffers_.empty())
		return {};

	if (V4L2_TYPE_IS_OUTPUT(bufferType_))
		return {};

	std::vector<std::unique_ptr<Buffer>> buffers;

	for (unsigned int i = 0; i < bufferPool_->count(); ++i) {
		Buffer *buffer = new Buffer(i);
		buffers.emplace_back(buffer);
		ret = queueBuffer(buffer);
		if (ret)
			return {};
	}

	return buffers;
}

/**
 * \brief Dequeue the next available buffer from the video device
 *
 * This method dequeues the next available buffer from the device. If no buffer
 * is available to be dequeued it will return nullptr immediately.
 *
 * \return A pointer to the dequeued buffer on success, or nullptr otherwise
 */
Buffer *V4L2VideoDevice::dequeueBuffer()
{
	struct v4l2_buffer buf = {};
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	int ret;

	buf.type = bufferType_;
	buf.memory = memoryType_;

	if (V4L2_TYPE_IS_MULTIPLANAR(buf.type)) {
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;
	}

	ret = ioctl(VIDIOC_DQBUF, &buf);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to dequeue buffer: " << strerror(-ret);
		return nullptr;
	}

	ASSERT(buf.index < bufferPool_->count());

	auto it = queuedBuffers_.find(buf.index);
	Buffer *buffer = it->second;
	queuedBuffers_.erase(it);

	if (queuedBuffers_.empty())
		fdEvent_->setEnabled(false);

	buffer->index_ = buf.index;
	buffer->bytesused_ = buf.bytesused;
	buffer->timestamp_ = buf.timestamp.tv_sec * 1000000000ULL
			   + buf.timestamp.tv_usec * 1000ULL;
	buffer->sequence_ = buf.sequence;
	buffer->status_ = buf.flags & V4L2_BUF_FLAG_ERROR
			? Buffer::BufferError : Buffer::BufferSuccess;

	return buffer;
}

/**
 * \brief Slot to handle completed buffer events from the V4L2 video device
 * \param[in] notifier The event notifier
 *
 * When this slot is called, a Buffer has become available from the device, and
 * will be emitted through the bufferReady Signal.
 *
 * For Capture video devices the Buffer will contain valid data.
 * For Output video devices the Buffer can be considered empty.
 */
void V4L2VideoDevice::bufferAvailable(EventNotifier *notifier)
{
	Buffer *buffer = dequeueBuffer();
	if (!buffer)
		return;

	LOG(V4L2, Debug) << "Buffer " << buffer->index() << " is available";

	/* Notify anyone listening to the device. */
	bufferReady.emit(buffer);
}

/**
 * \var V4L2VideoDevice::bufferReady
 * \brief A Signal emitted when a buffer completes
 */

/**
 * \brief Start the video stream
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::streamOn()
{
	int ret;

	ret = ioctl(VIDIOC_STREAMON, &bufferType_);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to start streaming: " << strerror(-ret);
		return ret;
	}

	return 0;
}

/**
 * \brief Stop the video stream
 *
 * Buffers that are still queued when the video stream is stopped are
 * immediately dequeued with their status set to Buffer::BufferError,
 * and the bufferReady signal is emitted for them. The order in which those
 * buffers are dequeued is not specified.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::streamOff()
{
	int ret;

	ret = ioctl(VIDIOC_STREAMOFF, &bufferType_);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to stop streaming: " << strerror(-ret);
		return ret;
	}

	/* Send back all queued buffers. */
	for (auto it : queuedBuffers_) {
		unsigned int index = it.first;
		Buffer *buffer = it.second;

		buffer->index_ = index;
		buffer->cancel();
		bufferReady.emit(buffer);
	}

	queuedBuffers_.clear();
	fdEvent_->setEnabled(false);

	return 0;
}

/**
 * \brief Create a new video device instance from \a entity in media device
 * \a media
 * \param[in] media The media device where the entity is registered
 * \param[in] entity The media entity name
 *
 * Releasing memory of the newly created instance is responsibility of the
 * caller of this function.
 *
 * \return A newly created V4L2VideoDevice on success, nullptr otherwise
 */
V4L2VideoDevice *V4L2VideoDevice::fromEntityName(const MediaDevice *media,
						 const std::string &entity)
{
	MediaEntity *mediaEntity = media->getEntityByName(entity);
	if (!mediaEntity)
		return nullptr;

	return new V4L2VideoDevice(mediaEntity);
}

/**
 * \brief Convert a \a v4l2Fourcc to the corresponding PixelFormat
 * \param[in] v4l2Fourcc The V4L2 pixel format (V4L2_PIX_FORMAT_*)
 * \return The PixelFormat corresponding to \a v4l2Fourcc
 */
PixelFormat V4L2VideoDevice::toPixelFormat(uint32_t v4l2Fourcc)
{
	switch (v4l2Fourcc) {
	/* RGB formats. */
	case V4L2_PIX_FMT_RGB24:
		return DRM_FORMAT_BGR888;
	case V4L2_PIX_FMT_BGR24:
		return DRM_FORMAT_RGB888;
	case V4L2_PIX_FMT_ARGB32:
		return DRM_FORMAT_BGRA8888;

	/* YUV packed formats. */
	case V4L2_PIX_FMT_YUYV:
		return DRM_FORMAT_YUYV;
	case V4L2_PIX_FMT_YVYU:
		return DRM_FORMAT_YVYU;
	case V4L2_PIX_FMT_UYVY:
		return DRM_FORMAT_UYVY;
	case V4L2_PIX_FMT_VYUY:
		return DRM_FORMAT_VYUY;

	/* YUY planar formats. */
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV16M:
		return DRM_FORMAT_NV16;
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV61M:
		return DRM_FORMAT_NV61;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
		return DRM_FORMAT_NV12;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		return DRM_FORMAT_NV21;

	/* Compressed formats. */
	case V4L2_PIX_FMT_MJPEG:
		return DRM_FORMAT_MJPEG;

	/* V4L2 formats not yet supported by DRM. */
	case V4L2_PIX_FMT_GREY:
	default:
		/*
		 * \todo We can't use LOG() in a static method of a Loggable
		 * class. Until we fix the logger, work around it.
		 */
		libcamera::_log(__FILE__, __LINE__, _LOG_CATEGORY(V4L2)(),
				LogError).stream()
			<< "Unsupported V4L2 pixel format "
			<< utils::hex(v4l2Fourcc);
		return 0;
	}
}

/**
 * \brief Convert \a PixelFormat to its corresponding V4L2 FourCC
 * \param[in] pixelFormat The PixelFormat to convert
 *
 * For multiplanar formats, the V4L2 format variant (contiguous or
 * non-contiguous planes) is selected automatically based on the capabilities
 * of the video device. If the video device supports the V4L2 multiplanar API,
 * non-contiguous formats are preferred.
 *
 * \return The V4L2_PIX_FMT_* pixel format code corresponding to \a pixelFormat
 */
uint32_t V4L2VideoDevice::toV4L2Fourcc(PixelFormat pixelFormat)
{
	return V4L2VideoDevice::toV4L2Fourcc(pixelFormat, caps_.isMultiplanar());
}

/**
 * \brief Convert \a pixelFormat to its corresponding V4L2 FourCC
 * \param[in] pixelFormat The PixelFormat to convert
 * \param[in] multiplanar V4L2 Multiplanar API support flag
 *
 * Multiple V4L2 formats may exist for one PixelFormat when the format uses
 * multiple planes, as V4L2 defines separate 4CCs for contiguous and separate
 * planes formats. Set the \a multiplanar parameter to false to select a format
 * with contiguous planes, or to true to select a format with non-contiguous
 * planes.
 *
 * \return The V4L2_PIX_FMT_* pixel format code corresponding to \a pixelFormat
 */
uint32_t V4L2VideoDevice::toV4L2Fourcc(PixelFormat pixelFormat, bool multiplanar)
{
	switch (pixelFormat) {
	/* RGB formats. */
	case DRM_FORMAT_BGR888:
		return V4L2_PIX_FMT_RGB24;
	case DRM_FORMAT_RGB888:
		return V4L2_PIX_FMT_BGR24;
	case DRM_FORMAT_BGRA8888:
		return V4L2_PIX_FMT_ARGB32;

	/* YUV packed formats. */
	case DRM_FORMAT_YUYV:
		return V4L2_PIX_FMT_YUYV;
	case DRM_FORMAT_YVYU:
		return V4L2_PIX_FMT_YVYU;
	case DRM_FORMAT_UYVY:
		return V4L2_PIX_FMT_UYVY;
	case DRM_FORMAT_VYUY:
		return V4L2_PIX_FMT_VYUY;

	/*
	 * YUY planar formats.
	 * \todo Add support for non-contiguous memory planes
	 * \todo Select the format variant not only based on \a multiplanar but
	 * also take into account the formats supported by the device.
	 */
	case DRM_FORMAT_NV16:
		return V4L2_PIX_FMT_NV16;
	case DRM_FORMAT_NV61:
		return V4L2_PIX_FMT_NV61;
	case DRM_FORMAT_NV12:
		return V4L2_PIX_FMT_NV12;
	case DRM_FORMAT_NV21:
		return V4L2_PIX_FMT_NV21;

	/* Compressed formats. */
	case DRM_FORMAT_MJPEG:
		return V4L2_PIX_FMT_MJPEG;
	}

	/*
	 * \todo We can't use LOG() in a static method of a Loggable
	 * class. Until we fix the logger, work around it.
	 */
	libcamera::_log(__FILE__, __LINE__, _LOG_CATEGORY(V4L2)(), LogError).stream()
		<< "Unsupported V4L2 pixel format "
		<< utils::hex(pixelFormat);
	return 0;
}

/**
 * \class V4L2M2MDevice
 * \brief Memory-to-Memory video device
 *
 * The V4L2M2MDevice manages two V4L2VideoDevice instances on the same
 * deviceNode which operate together using two queues to implement the V4L2
 * Memory to Memory API.
 *
 * The two devices should be opened by calling open() on the V4L2M2MDevice, and
 * can be closed by calling close on the V4L2M2MDevice.
 *
 * Calling V4L2VideoDevice::open() and V4L2VideoDevice::close() on the capture
 * or output V4L2VideoDevice is not permitted.
 */

/**
 * \fn V4L2M2MDevice::output
 * \brief Retrieve the output V4L2VideoDevice instance
 * \return The output V4L2VideoDevice instance
 */

/**
 * \fn V4L2M2MDevice::capture
 * \brief Retrieve the capture V4L2VideoDevice instance
 * \return The capture V4L2VideoDevice instance
 */

/**
 * \brief Create a new V4L2M2MDevice from the \a deviceNode
 * \param[in] deviceNode The file-system path to the video device node
 */
V4L2M2MDevice::V4L2M2MDevice(const std::string &deviceNode)
	: deviceNode_(deviceNode)
{
	output_ = new V4L2VideoDevice(deviceNode);
	capture_ = new V4L2VideoDevice(deviceNode);
}

V4L2M2MDevice::~V4L2M2MDevice()
{
	delete capture_;
	delete output_;
}

/**
 * \brief Open a V4L2 Memory to Memory device
 *
 * Open the device node and prepare the two V4L2VideoDevice instances to handle
 * their respective buffer queues.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2M2MDevice::open()
{
	int fd;
	int ret;

	/*
	 * The output and capture V4L2VideoDevice instances use the same file
	 * handle for the same device node. The local file handle can be closed
	 * as the V4L2VideoDevice::open() retains a handle by duplicating the
	 * fd passed in.
	 */
	fd = ::open(deviceNode_.c_str(), O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to open V4L2 M2M device: " << strerror(-ret);
		return ret;
	}

	ret = output_->open(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (ret)
		goto err;

	ret = capture_->open(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (ret)
		goto err;

	::close(fd);

	return 0;

err:
	close();
	::close(fd);

	return ret;
}

/**
 * \brief Close the memory-to-memory device, releasing any resources acquired by
 * open()
 */
void V4L2M2MDevice::close()
{
	capture_->close();
	output_->close();
}

} /* namespace libcamera */
