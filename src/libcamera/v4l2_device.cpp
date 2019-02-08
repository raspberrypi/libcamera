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
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include <libcamera/buffer.h>
#include <libcamera/event_notifier.h>

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
 * This class describes the image format and resolution to be programmed on a
 * V4L2 video device. The image format is defined by a fourcc code (as specified
 * by the V4L2 API with the V4L2_PIX_FMT_* macros), a resolution (width and
 * height) and one to three planes with configurable line stride and a total
 * per-plane size in bytes.
 *
 * Image formats, as defined by the V4L2 APIs, are categorized as packed,
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
 * through the single-plane API. When used on a device that implements the
 * multi-plane API, only the size and stride information contained in the first
 * plane are taken into account.
 *
 * Planar image formats, which occupy distinct memory areas, are easily
 * described through the multi-plane APIs. When used on a device that implements
 * the single-plane API, all planes are stored one after the other in a
 * contiguous memory area, and it is not possible to configure per-plane stride
 * length and size, but only a global stride length which is applied to all
 * planes.
 *
 * The V4L2DeviceFormat class describes both packed and planar image formats,
 * regardless of the API type (single or multi plane) implemented by the device
 * the format has to be applied to. The total size and bytes per line of images
 * represented with packed formats are configured using the first entry of the
 * V4L2DeviceFormat::planes array, while the per-plane size and per-plane stride
 * length of images represented with planar image formats are configured using
 * the opportune number of entries of the V4L2DeviceFormat::planes array, as
 * prescribed by the image format definition (semi-planar formats use 2 entries,
 * while planar formats use the whole 3 entries). The number of valid entries of
 * the V4L2DeviceFormat::planes array is defined by the
 * V4L2DeviceFormat::planesCount value.
 */

/**
 * \var V4L2DeviceFormat::width
 * \brief The image width in pixels
 */

/**
 * \var V4L2DeviceFormat::height
 * \brief The image height in pixels
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
 * The V4L2Device class tracks queued buffers and handles buffer events. It
 * automatically dequeues completed buffers and emits the \ref bufferReady
 * signal.
 *
 * Upon destruction any device left open will be closed, and any resources
 * released.
 */

/**
 * \brief Construct a V4L2Device
 * \param deviceNode The file-system path to the video device node
 */
V4L2Device::V4L2Device(const std::string &deviceNode)
	: deviceNode_(deviceNode), fd_(-1), bufferPool_(nullptr),
	  queuedBuffersCount_(0), fdEvent_(nullptr)
{
	/*
	 * We default to an MMAP based CAPTURE device, however this will be
	 * updated based upon the device capabilities.
	 */
	bufferType_ = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	memoryType_ = V4L2_MEMORY_MMAP;
}

/**
 * \brief Construct a V4L2Device from a MediaEntity
 * \param entity The MediaEntity to build the device from
 *
 * Construct a V4L2Device from a MediaEntity's device node path.
 */
V4L2Device::V4L2Device(const MediaEntity *entity)
	: V4L2Device(entity->deviceNode())
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

	ret = ::open(deviceNode_.c_str(), O_RDWR | O_NONBLOCK);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to open V4L2 device: " << strerror(-ret);
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
		<< "Opened device " << caps_.bus_info() << ": "
		<< caps_.driver() << ": " << caps_.card();

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

	fdEvent_ = new EventNotifier(fd_, EventNotifier::Read);
	fdEvent_->activated.connect(this, &V4L2Device::bufferAvailable);
	fdEvent_->setEnabled(false);

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

	releaseBuffers();
	delete fdEvent_;

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

std::string V4L2Device::logPrefix() const
{
	return deviceNode_;
}

/**
 * \brief Retrieve the image format set on the V4L2 device
 * \param[out] format The image format applied on the device
 *
 * \return 0 for success, a negative error code otherwise
 */
int V4L2Device::getFormat(V4L2DeviceFormat *format)
{
	return caps_.isMultiplanar() ? getFormatMultiplane(format) :
				       getFormatSingleplane(format);
}

/**
 * \brief Configure an image format on the V4L2 device
 * \param[in] format The image format to apply to the device
 *
 * Apply the supplied \a format to the device, and return the actually
 * applied format parameters, as \ref V4L2Device::getFormat would do.
 *
 * \return 0 for success, a negative error code otherwise
 */
int V4L2Device::setFormat(V4L2DeviceFormat *format)
{
	return caps_.isMultiplanar() ? setFormatMultiplane(format) :
				       setFormatSingleplane(format);
}

int V4L2Device::getFormatSingleplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format *pix = &v4l2Format.fmt.pix;
	int ret;

	v4l2Format.type = bufferType_;
	ret = ioctl(fd_, VIDIOC_G_FMT, &v4l2Format);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	format->width = pix->width;
	format->height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = 1;
	format->planes[0].bpl = pix->bytesperline;
	format->planes[0].size = pix->sizeimage;

	return 0;
}

int V4L2Device::setFormatSingleplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format *pix = &v4l2Format.fmt.pix;
	int ret;

	v4l2Format.type = bufferType_;
	pix->width = format->width;
	pix->height = format->height;
	pix->pixelformat = format->fourcc;
	pix->bytesperline = format->planes[0].bpl;

	ret = ioctl(fd_, VIDIOC_S_FMT, &v4l2Format);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the device,
	 * which might differ from the requested one.
	 */
	format->width = pix->width;
	format->height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = 1;
	format->planes[0].bpl = pix->bytesperline;
	format->planes[0].size = pix->sizeimage;

	return 0;
}

int V4L2Device::getFormatMultiplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format_mplane *pix = &v4l2Format.fmt.pix_mp;
	int ret;

	v4l2Format.type = bufferType_;
	ret = ioctl(fd_, VIDIOC_G_FMT, &v4l2Format);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to get format: " << strerror(-ret);
		return ret;
	}

	format->width = pix->width;
	format->height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = pix->num_planes;

	for (unsigned int i = 0; i < format->planesCount; ++i) {
		format->planes[i].bpl = pix->plane_fmt[i].bytesperline;
		format->planes[i].size = pix->plane_fmt[i].sizeimage;
	}

	return 0;
}

int V4L2Device::setFormatMultiplane(V4L2DeviceFormat *format)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_pix_format_mplane *pix = &v4l2Format.fmt.pix_mp;
	int ret;

	v4l2Format.type = bufferType_;
	pix->width = format->width;
	pix->height = format->height;
	pix->pixelformat = format->fourcc;
	pix->num_planes = format->planesCount;

	for (unsigned int i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = format->planes[i].bpl;
		pix->plane_fmt[i].sizeimage = format->planes[i].size;
	}

	ret = ioctl(fd_, VIDIOC_S_FMT, &v4l2Format);
	if (ret) {
		ret = -errno;
		LOG(Error) << "Unable to set format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the device,
	 * which might differ from the requested one.
	 */
	format->width = pix->width;
	format->height = pix->height;
	format->fourcc = pix->pixelformat;
	format->planesCount = pix->num_planes;
	for (unsigned int i = 0; i < format->planesCount; ++i) {
		format->planes[i].bpl = pix->plane_fmt[i].bytesperline;
		format->planes[i].size = pix->plane_fmt[i].sizeimage;
	}

	return 0;
}

int V4L2Device::requestBuffers(unsigned int count)
{
	struct v4l2_requestbuffers rb = {};
	int ret;

	rb.count = count;
	rb.type = bufferType_;
	rb.memory = memoryType_;

	ret = ioctl(fd_, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Unable to request " << count << " buffers: "
			<< strerror(-ret);
		return ret;
	}

	LOG(V4L2, Debug) << rb.count << " buffers requested.";

	return rb.count;
}

/**
 * \brief Request \a count buffers to be allocated from the device and stored in
 * the buffer pool provided.
 * \param[in] count Number of buffers to allocate
 * \param[out] pool BufferPool to populate with buffers
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::exportBuffers(unsigned int count, BufferPool *pool)
{
	unsigned int allocatedBuffers;
	unsigned int i;
	int ret;

	memoryType_ = V4L2_MEMORY_MMAP;

	ret = requestBuffers(count);
	if (ret < 0)
		return ret;

	allocatedBuffers = ret;
	if (allocatedBuffers < count) {
		LOG(V4L2, Error) << "Not enough buffers provided by V4L2Device";
		requestBuffers(0);
		return -ENOMEM;
	}

	count = ret;

	/* Map the buffers. */
	for (i = 0; i < count; ++i) {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
		struct v4l2_buffer buf = {};
		struct Buffer &buffer = pool->buffers()[i];

		buf.index = i;
		buf.type = bufferType_;
		buf.memory = memoryType_;
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;

		ret = ioctl(fd_, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			ret = -errno;
			LOG(V4L2, Error)
				<< "Unable to query buffer " << i << ": "
				<< strerror(-ret);
			break;
		}

		if (V4L2_TYPE_IS_MULTIPLANAR(buf.type)) {
			for (unsigned int p = 0; p < buf.length; ++p) {
				ret = createPlane(&buffer, p,
						  buf.m.planes[p].length);
				if (ret)
					break;
			}
		} else {
			ret = createPlane(&buffer, 0, buf.length);
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

int V4L2Device::createPlane(Buffer *buffer, unsigned int planeIndex,
			    unsigned int length)
{
	struct v4l2_exportbuffer expbuf = {};
	int ret;

	LOG(V4L2, Debug)
		<< "Buffer " << buffer->index()
		<< " plane " << planeIndex
		<< ": length=" << length;

	expbuf.type = bufferType_;
	expbuf.index = buffer->index();
	expbuf.plane = planeIndex;
	expbuf.flags = O_RDWR;

	ret = ioctl(fd_, VIDIOC_EXPBUF, &expbuf);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to export buffer: " << strerror(-ret);
		return ret;
	}

	buffer->planes().emplace_back();
	Plane &plane = buffer->planes().back();
	plane.setDmabuf(expbuf.fd, length);

	return 0;
}

/**
 * \brief Release all internally allocated buffers
 */
int V4L2Device::releaseBuffers()
{
	LOG(V4L2, Debug) << "Releasing bufferPool";

	requestBuffers(0);
	bufferPool_ = nullptr;

	return 0;
}

/**
 * \brief Queue a buffer into the device
 * \param[in] buffer The buffer to be queued
 *
 * For capture devices the \a buffer will be filled with data by the device.
 * For output devices the \a buffer shall contain valid data and will be
 * processed by the device. Once the device has finished processing the buffer,
 * it will be available for dequeue.
 *
 * \todo Support output devices (bytesused, ...)
 * \todo Support imported buffers (dmabuf fd)
 *
 * \return 0 on success or a negative error number otherwise
 */
int V4L2Device::queueBuffer(Buffer *buffer)
{
	struct v4l2_buffer buf = {};
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	int ret;

	buf.index = buffer->index();
	buf.type = bufferType_;
	buf.memory = memoryType_;

	if (V4L2_TYPE_IS_MULTIPLANAR(buf.type)) {
		buf.length = buffer->planes().size();
		buf.m.planes = planes;
	}

	LOG(V4L2, Debug) << "Queueing buffer " << buf.index;

	ret = ioctl(fd_, VIDIOC_QBUF, &buf);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to queue buffer " << buf.index << ": "
			<< strerror(-ret);
		return ret;
	}

	if (queuedBuffersCount_++ == 0)
		fdEvent_->setEnabled(true);

	return 0;
}

/**
 * \brief Dequeue the next available buffer from the device
 *
 * This method dequeues the next available buffer from the device. If no buffer
 * is available to be dequeued it will return nullptr immediately.
 *
 * \return A pointer to the dequeued buffer on succcess, or nullptr otherwise
 */
Buffer *V4L2Device::dequeueBuffer()
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

	ret = ioctl(fd_, VIDIOC_DQBUF, &buf);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to dequeue buffer: " << strerror(-ret);
		return nullptr;
	}

	ASSERT(buf.index < bufferPool_->count());

	if (--queuedBuffersCount_ == 0)
		fdEvent_->setEnabled(false);

	Buffer *buffer = &bufferPool_->buffers()[buf.index];

	buffer->bytesused_ = buf.bytesused;
	buffer->timestamp_ = buf.timestamp.tv_sec * 1000000000ULL
			   + buf.timestamp.tv_usec * 1000ULL;
	buffer->sequence_ = buf.sequence;

	return buffer;
}

/**
 * \brief Slot to handle completed buffer events from the V4L2 device
 * \param[in] notifier The event notifier
 *
 * When this slot is called, a Buffer has become available from the device, and
 * will be emitted through the bufferReady Signal.
 *
 * For Capture devices the Buffer will contain valid data.
 * For Output devices the Buffer can be considered empty.
 */
void V4L2Device::bufferAvailable(EventNotifier *notifier)
{
	Buffer *buffer = dequeueBuffer();
	if (!buffer)
		return;

	LOG(V4L2, Debug) << "Buffer " << buffer->index() << " is available";

	/* Notify anyone listening to the device. */
	bufferReady.emit(buffer);

	/* Notify anyone listening to the buffer specifically. */
	buffer->completed.emit(buffer);
}

/**
 * \var V4L2Device::bufferReady
 * \brief A Signal emitted when a buffer completes
 */

/**
 * \brief Start the video stream
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::streamOn()
{
	int ret;

	ret = ioctl(fd_, VIDIOC_STREAMON, &bufferType_);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to start streaming: " << strerror(-ret);
		return ret;
	}

	return 0;
}

/**
 * \brief Stop the video stream
 *
 * \todo Ensure completion notifications are sent for all queued buffers
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Device::streamOff()
{
	int ret;

	ret = ioctl(fd_, VIDIOC_STREAMOFF, &bufferType_);
	if (ret < 0) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Failed to stop streaming: " << strerror(-ret);
		return ret;
	}

	queuedBuffersCount_ = 0;
	fdEvent_->setEnabled(false);

	return 0;
}

} /* namespace libcamera */
