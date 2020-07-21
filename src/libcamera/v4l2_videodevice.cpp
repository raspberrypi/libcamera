/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_videodevice.cpp - V4L2 Video Device
 */

#include "libcamera/internal/v4l2_videodevice.h"

#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include <linux/version.h>

#include <libcamera/event_notifier.h>
#include <libcamera/file_descriptor.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"
#include "libcamera/internal/utils.h"

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
 * \class V4L2BufferCache
 * \brief Hot cache of associations between V4L2 buffer indexes and FrameBuffer
 *
 * When importing buffers, V4L2 performs lazy mapping of dmabuf instances at
 * VIDIOC_QBUF (or VIDIOC_PREPARE_BUF) time and keeps the mapping associated
 * with the V4L2 buffer, as identified by its index. If the same V4L2 buffer is
 * then reused and queued with different dmabufs, the old dmabufs will be
 * unmapped and the new ones mapped. To keep this process efficient, it is
 * crucial to consistently use the same V4L2 buffer for given dmabufs through
 * the whole duration of a capture cycle.
 *
 * The V4L2BufferCache class keeps a map of previous dmabufs to V4L2 buffer
 * index associations to help selecting V4L2 buffers. It tracks, for every
 * entry, if the V4L2 buffer is in use, and offers lookup of the best free V4L2
 * buffer for a set of dmabufs.
 */

/**
 * \brief Create an empty cache with \a numEntries entries
 * \param[in] numEntries Number of entries to reserve in the cache
 *
 * Create a cache with \a numEntries entries all marked as unused. The entries
 * will be populated as the cache is used. This is typically used to implement
 * buffer import, with buffers added to the cache as they are queued.
 */
V4L2BufferCache::V4L2BufferCache(unsigned int numEntries)
	: lastUsedCounter_(1), missCounter_(0)
{
	cache_.resize(numEntries);
}

/**
 * \brief Create a pre-populated cache
 * \param[in] buffers Array of buffers to pre-populated with
 *
 * Create a cache pre-populated with \a buffers. This is typically used to
 * implement buffer export, with all buffers added to the cache when they are
 * allocated.
 */
V4L2BufferCache::V4L2BufferCache(const std::vector<std::unique_ptr<FrameBuffer>> &buffers)
	: lastUsedCounter_(1), missCounter_(0)
{
	for (const std::unique_ptr<FrameBuffer> &buffer : buffers)
		cache_.emplace_back(true,
				    lastUsedCounter_.fetch_add(1, std::memory_order_acq_rel),
				    buffer->planes());
}

V4L2BufferCache::~V4L2BufferCache()
{
	if (missCounter_ > cache_.size())
		LOG(V4L2, Debug) << "Cache misses: " << missCounter_;
}

/**
 * \brief Find the best V4L2 buffer for a FrameBuffer
 * \param[in] buffer The FrameBuffer
 *
 * Find the best V4L2 buffer index to be used for the FrameBuffer \a buffer
 * based on previous mappings of frame buffers to V4L2 buffers. If a free V4L2
 * buffer previously used with the same dmabufs as \a buffer is found in the
 * cache, return its index. Otherwise return the index of the first free V4L2
 * buffer and record its association with the dmabufs of \a buffer.
 *
 * \return The index of the best V4L2 buffer, or -ENOENT if no free V4L2 buffer
 * is available
 */
int V4L2BufferCache::get(const FrameBuffer &buffer)
{
	bool hit = false;
	int use = -1;
	uint64_t oldest = UINT64_MAX;

	for (unsigned int index = 0; index < cache_.size(); index++) {
		const Entry &entry = cache_[index];

		if (!entry.free)
			continue;

		/* Try to find a cache hit by comparing the planes. */
		if (entry == buffer) {
			hit = true;
			use = index;
			break;
		}

		if (entry.lastUsed < oldest) {
			use = index;
			oldest = entry.lastUsed;
		}
	}

	if (!hit)
		missCounter_++;

	if (use < 0)
		return -ENOENT;

	cache_[use] = Entry(false,
			    lastUsedCounter_.fetch_add(1, std::memory_order_acq_rel),
			    buffer);

	return use;
}

/**
 * \brief Mark buffer \a index as free in the cache
 * \param[in] index The V4L2 buffer index
 */
void V4L2BufferCache::put(unsigned int index)
{
	ASSERT(index < cache_.size());
	cache_[index].free = true;
}

V4L2BufferCache::Entry::Entry()
	: free(true), lastUsed(0)
{
}

V4L2BufferCache::Entry::Entry(bool free, uint64_t lastUsed, const FrameBuffer &buffer)
	: free(free), lastUsed(lastUsed)
{
	for (const FrameBuffer::Plane &plane : buffer.planes())
		planes_.emplace_back(plane);
}

bool V4L2BufferCache::Entry::operator==(const FrameBuffer &buffer) const
{
	const std::vector<FrameBuffer::Plane> &planes = buffer.planes();

	if (planes_.size() != planes.size())
		return false;

	for (unsigned int i = 0; i < planes.size(); i++)
		if (planes_[i].fd != planes[i].fd.fd() ||
		    planes_[i].length != planes[i].length)
			return false;
	return true;
}

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
	ss << size.toString() << "-" << fourcc.toString();
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
 * The V4L2VideoDevice class supports the V4L2 MMAP and DMABUF memory types:
 *
 * - The allocateBuffers() function wraps buffer allocation with the V4L2 MMAP
 *   memory type. It requests buffers from the driver, allocating the
 *   corresponding memory, and exports them as a set of FrameBuffer objects.
 *   Upon successful return the driver's internal buffer management is
 *   initialized in MMAP mode, and the video device is ready to accept
 *   queueBuffer() calls.
 *
 *   This is the most traditional V4L2 buffer management, and is mostly useful
 *   to support internal buffer pools in pipeline handlers, either for CPU
 *   consumption (such as statistics or parameters pools), or for internal
 *   image buffers shared between devices.
 *
 * - The exportBuffers() function operates similarly to allocateBuffers(), but
 *   leaves the driver's internal buffer management uninitialized. It uses the
 *   V4L2 buffer orphaning support to allocate buffers with the MMAP method,
 *   export them as a set of FrameBuffer objects, and reset the driver's
 *   internal buffer management. The video device shall be initialized with
 *   importBuffers() or allocateBuffers() before it can accept queueBuffer()
 *   calls. The exported buffers are directly usable with any V4L2 video device
 *   in DMABUF mode, or with other dmabuf importers.
 *
 *   This method is mostly useful to implement buffer allocation helpers or to
 *   allocate ancillary buffers, when a V4L2 video device is used in DMABUF
 *   mode but no other source of buffers is available. An example use case
 *   would be allocation of scratch buffers to be used in case of buffer
 *   underruns on a video device that is otherwise supplied with external
 *   buffers.
 *
 * - The importBuffers() function initializes the driver's buffer management to
 *   import buffers in DMABUF mode. It requests buffers from the driver, but
 *   doesn't allocate memory. Upon successful return, the video device is ready
 *   to accept queueBuffer() calls. The buffers to be imported are provided to
 *   queueBuffer(), and may be supplied externally, or come from a previous
 *   exportBuffers() call.
 *
 *   This is the usual buffers initialization method for video devices whose
 *   buffers are exposed outside of libcamera. It is also typically used on one
 *   of the two video device that participate in buffer sharing inside
 *   pipelines, the other video device typically using allocateBuffers().
 *
 * - The releaseBuffers() function resets the driver's internal buffer
 *   management that was initialized by a previous call to allocateBuffers() or
 *   importBuffers(). Any memory allocated by allocateBuffers() is freed.
 *   Buffer exported by exportBuffers() are not affected by this function.
 *
 * The V4L2VideoDevice class tracks queued buffers and handles buffer events. It
 * automatically dequeues completed buffers and emits the \ref bufferReady
 * signal.
 *
 * Upon destruction any device left open will be closed, and any resources
 * released.
 *
 * \context This class is \threadbound.
 */

/**
 * \typedef V4L2VideoDevice::Formats
 * \brief A map of supported V4L2 pixel formats to frame sizes
 */

/**
 * \brief Construct a V4L2VideoDevice
 * \param[in] deviceNode The file-system path to the video device node
 */
V4L2VideoDevice::V4L2VideoDevice(const std::string &deviceNode)
	: V4L2Device(deviceNode), cache_(nullptr), fdBufferNotifier_(nullptr),
	  fdEventNotifier_(nullptr), frameStartEnabled_(false)
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

	if (caps_.version < KERNEL_VERSION(5, 0, 0)) {
		LOG(V4L2, Error)
			<< "V4L2 API v" << (caps_.version >> 16)
			<< "." << ((caps_.version >> 8) & 0xff)
			<< "." << (caps_.version & 0xff)
			<< " too old, v5.0.0 or later is required";
		return -EINVAL;
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
	EventNotifier::Type notifierType;

	if (caps_.isVideoCapture()) {
		notifierType = EventNotifier::Read;
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else if (caps_.isVideoOutput()) {
		notifierType = EventNotifier::Write;
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_OUTPUT;
	} else if (caps_.isMetaCapture()) {
		notifierType = EventNotifier::Read;
		bufferType_ = V4L2_BUF_TYPE_META_CAPTURE;
	} else if (caps_.isMetaOutput()) {
		notifierType = EventNotifier::Write;
		bufferType_ = V4L2_BUF_TYPE_META_OUTPUT;
	} else {
		LOG(V4L2, Error) << "Device is not a supported type";
		return -EINVAL;
	}

	fdBufferNotifier_ = new EventNotifier(fd(), notifierType);
	fdBufferNotifier_->activated.connect(this, &V4L2VideoDevice::bufferAvailable);
	fdBufferNotifier_->setEnabled(false);

	fdEventNotifier_ = new EventNotifier(fd(), EventNotifier::Exception);
	fdEventNotifier_->activated.connect(this, &V4L2VideoDevice::eventAvailable);
	fdEventNotifier_->setEnabled(false);

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
	EventNotifier::Type notifierType;

	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		notifierType = EventNotifier::Write;
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		notifierType = EventNotifier::Read;
		bufferType_ = caps_.isMultiplanar()
			    ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
			    : V4L2_BUF_TYPE_VIDEO_CAPTURE;
		break;
	default:
		LOG(V4L2, Error) << "Unsupported buffer type";
		return -EINVAL;
	}

	fdBufferNotifier_ = new EventNotifier(fd(), notifierType);
	fdBufferNotifier_->activated.connect(this, &V4L2VideoDevice::bufferAvailable);
	fdBufferNotifier_->setEnabled(false);

	fdEventNotifier_ = new EventNotifier(fd(), EventNotifier::Exception);
	fdEventNotifier_->activated.connect(this, &V4L2VideoDevice::eventAvailable);
	fdEventNotifier_->setEnabled(false);

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
	delete fdBufferNotifier_;
	delete fdEventNotifier_;

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

/**
 * \fn V4L2VideoDevice::caps()
 * \brief Retrieve the device V4L2 capabilities
 * \return The device V4L2 capabilities
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
 * \brief Try an image format on the V4L2 video device
 * \param[inout] format The image format to test applicability to the video device
 *
 * Try the supplied \a format on the video device without applying it, returning
 * the format that would be applied. This is equivalent to setFormat(), except
 * that the device configuration is not changed.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::tryFormat(V4L2DeviceFormat *format)
{
	if (caps_.isMeta())
		return trySetFormatMeta(format, false);
	else if (caps_.isMultiplanar())
		return trySetFormatMultiplane(format, false);
	else
		return trySetFormatSingleplane(format, false);
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
		return trySetFormatMeta(format, true);
	else if (caps_.isMultiplanar())
		return trySetFormatMultiplane(format, true);
	else
		return trySetFormatSingleplane(format, true);
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
	format->fourcc = V4L2PixelFormat(pix->dataformat);
	format->planesCount = 1;
	format->planes[0].bpl = pix->buffersize;
	format->planes[0].size = pix->buffersize;

	return 0;
}

int V4L2VideoDevice::trySetFormatMeta(V4L2DeviceFormat *format, bool set)
{
	struct v4l2_format v4l2Format = {};
	struct v4l2_meta_format *pix = &v4l2Format.fmt.meta;
	int ret;

	v4l2Format.type = bufferType_;
	pix->dataformat = format->fourcc;
	pix->buffersize = format->planes[0].size;
	ret = ioctl(set ? VIDIOC_S_FMT : VIDIOC_TRY_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to " << (set ? "set" : "try")
			<< " format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the video device,
	 * which might differ from the requested one.
	 */
	format->size.width = 0;
	format->size.height = 0;
	format->fourcc = V4L2PixelFormat(pix->dataformat);
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
	format->fourcc = V4L2PixelFormat(pix->pixelformat);
	format->planesCount = pix->num_planes;

	for (unsigned int i = 0; i < format->planesCount; ++i) {
		format->planes[i].bpl = pix->plane_fmt[i].bytesperline;
		format->planes[i].size = pix->plane_fmt[i].sizeimage;
	}

	return 0;
}

int V4L2VideoDevice::trySetFormatMultiplane(V4L2DeviceFormat *format, bool set)
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

	ret = ioctl(set ? VIDIOC_S_FMT : VIDIOC_TRY_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to " << (set ? "set" : "try")
			<< " format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the video device,
	 * which might differ from the requested one.
	 */
	format->size.width = pix->width;
	format->size.height = pix->height;
	format->fourcc = V4L2PixelFormat(pix->pixelformat);
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
	format->fourcc = V4L2PixelFormat(pix->pixelformat);
	format->planesCount = 1;
	format->planes[0].bpl = pix->bytesperline;
	format->planes[0].size = pix->sizeimage;

	return 0;
}

int V4L2VideoDevice::trySetFormatSingleplane(V4L2DeviceFormat *format, bool set)
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
	ret = ioctl(set ? VIDIOC_S_FMT : VIDIOC_TRY_FMT, &v4l2Format);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to " << (set ? "set" : "try")
			<< " format: " << strerror(-ret);
		return ret;
	}

	/*
	 * Return to caller the format actually applied on the device,
	 * which might differ from the requested one.
	 */
	format->size.width = pix->width;
	format->size.height = pix->height;
	format->fourcc = V4L2PixelFormat(pix->pixelformat);
	format->planesCount = 1;
	format->planes[0].bpl = pix->bytesperline;
	format->planes[0].size = pix->sizeimage;

	return 0;
}

/**
 * \brief Enumerate all pixel formats and frame sizes
 * \param[in] code Restrict formats to this media bus code.
 *
 * Enumerate all pixel formats and frame sizes supported by the video device.
 * If the \a code argument is not zero, only formats compatible with that media
 * bus code will be enumerated.
 *
 * \return A list of the supported video device formats
 */
V4L2VideoDevice::Formats V4L2VideoDevice::formats(uint32_t code)
{
	Formats formats;

	for (V4L2PixelFormat pixelFormat : enumPixelformats(code)) {
		std::vector<SizeRange> sizes = enumSizes(pixelFormat);
		if (sizes.empty())
			return {};

		if (formats.find(pixelFormat) != formats.end()) {
			LOG(V4L2, Error)
				<< "Could not add sizes for pixel format "
				<< pixelFormat;
			return {};
		}

		formats.emplace(pixelFormat, sizes);
	}

	return formats;
}

std::vector<V4L2PixelFormat> V4L2VideoDevice::enumPixelformats(uint32_t code)
{
	std::vector<V4L2PixelFormat> formats;
	int ret;

	if (code && !(caps_.device_caps() & V4L2_CAP_IO_MC)) {
		LOG(V4L2, Error)
			<< "Media bus code filtering not supported by the device";
		return {};
	}

	for (unsigned int index = 0; ; index++) {
		struct v4l2_fmtdesc pixelformatEnum = {};
		pixelformatEnum.index = index;
		pixelformatEnum.type = bufferType_;
		pixelformatEnum.mbus_code = code;

		ret = ioctl(VIDIOC_ENUM_FMT, &pixelformatEnum);
		if (ret)
			break;

		formats.push_back(V4L2PixelFormat(pixelformatEnum.pixelformat));
	}

	if (ret && ret != -EINVAL) {
		LOG(V4L2, Error)
			<< "Unable to enumerate pixel formats: "
			<< strerror(-ret);
		return {};
	}

	return formats;
}

std::vector<SizeRange> V4L2VideoDevice::enumSizes(V4L2PixelFormat pixelFormat)
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
			sizes.emplace_back(Size{ frameSize.discrete.width,
						 frameSize.discrete.height });
			break;
		case V4L2_FRMSIZE_TYPE_CONTINUOUS:
			sizes.emplace_back(Size{ frameSize.stepwise.min_width,
						 frameSize.stepwise.min_height },
					   Size{ frameSize.stepwise.max_width,
						 frameSize.stepwise.max_height });
			break;
		case V4L2_FRMSIZE_TYPE_STEPWISE:
			sizes.emplace_back(Size{ frameSize.stepwise.min_width,
						 frameSize.stepwise.min_height },
					   Size{ frameSize.stepwise.max_width,
						 frameSize.stepwise.max_height },
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

/**
 * \brief Set a selection rectangle \a rect for \a target
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[inout] rect The selection rectangle to be applied
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::setSelection(unsigned int target, Rectangle *rect)
{
	struct v4l2_selection sel = {};

	sel.type = bufferType_;
	sel.target = target;
	sel.flags = 0;

	sel.r.left = rect->x;
	sel.r.top = rect->y;
	sel.r.width = rect->width;
	sel.r.height = rect->height;

	int ret = ioctl(VIDIOC_S_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error) << "Unable to set rectangle " << target
				 << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}

int V4L2VideoDevice::requestBuffers(unsigned int count,
				    enum v4l2_memory memoryType)
{
	struct v4l2_requestbuffers rb = {};
	int ret;

	rb.count = count;
	rb.type = bufferType_;
	rb.memory = memoryType;

	ret = ioctl(VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to request " << count << " buffers: "
			<< strerror(-ret);
		return ret;
	}

	if (rb.count < count) {
		LOG(V4L2, Error)
			<< "Not enough buffers provided by V4L2VideoDevice";
		requestBuffers(0, memoryType);
		return -ENOMEM;
	}

	LOG(V4L2, Debug) << rb.count << " buffers requested.";

	return 0;
}

/**
 * \brief Allocate and export buffers from the video device
 * \param[in] count Number of buffers to allocate
 * \param[out] buffers Vector to store allocated buffers
 *
 * This function wraps buffer allocation with the V4L2 MMAP memory type. It
 * requests \a count buffers from the driver, allocating the corresponding
 * memory, and exports them as a set of FrameBuffer objects in \a buffers. Upon
 * successful return the driver's internal buffer management is initialized in
 * MMAP mode, and the video device is ready to accept queueBuffer() calls.
 *
 * The number of planes and the plane sizes for the allocation are determined
 * by the currently active format on the device as set by setFormat().
 *
 * Buffers allocated with this function shall later be free with
 * releaseBuffers(). If buffers have already been allocated with
 * allocateBuffers() or imported with importBuffers(), this function returns
 * -EBUSY.
 *
 * \return The number of allocated buffers on success or a negative error code
 * otherwise
 * \retval -EBUSY buffers have already been allocated or imported
 */
int V4L2VideoDevice::allocateBuffers(unsigned int count,
				     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	int ret = createBuffers(count, buffers);
	if (ret < 0)
		return ret;

	cache_ = new V4L2BufferCache(*buffers);
	memoryType_ = V4L2_MEMORY_MMAP;

	return ret;
}

/**
 * \brief Export buffers from the video device
 * \param[in] count Number of buffers to allocate
 * \param[out] buffers Vector to store allocated buffers
 *
 * This function allocates \a count buffer from the video device and exports
 * them as dmabuf objects, stored in \a buffers. Unlike allocateBuffers(), this
 * function leaves the driver's internal buffer management uninitialized. The
 * video device shall be initialized with importBuffers() or allocateBuffers()
 * before it can accept queueBuffer() calls. The exported buffers are directly
 * usable with any V4L2 video device in DMABUF mode, or with other dmabuf
 * importers.
 *
 * The number of planes and the plane sizes for the allocation are determined
 * by the currently active format on the device as set by setFormat().
 *
 * Multiple independent sets of buffers can be allocated with multiple calls to
 * this function. Device-specific limitations may apply regarding the minimum
 * and maximum number of buffers per set, or to total amount of allocated
 * memory. The exported dmabuf lifetime is tied to the returned \a buffers. To
 * free a buffer, the caller shall delete the corresponding FrameBuffer
 * instance. No bookkeeping and automatic free is performed by the
 * V4L2VideoDevice class.
 *
 * If buffers have already been allocated with allocateBuffers() or imported
 * with importBuffers(), this function returns -EBUSY.
 *
 * \return The number of allocated buffers on success or a negative error code
 * otherwise
 * \retval -EBUSY buffers have already been allocated or imported
 */
int V4L2VideoDevice::exportBuffers(unsigned int count,
				   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	int ret = createBuffers(count, buffers);
	if (ret < 0)
		return ret;

	requestBuffers(0, V4L2_MEMORY_MMAP);

	return ret;
}

int V4L2VideoDevice::createBuffers(unsigned int count,
				   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	if (cache_) {
		LOG(V4L2, Error) << "Buffers already allocated";
		return -EINVAL;
	}

	int ret = requestBuffers(count, V4L2_MEMORY_MMAP);
	if (ret < 0)
		return ret;

	for (unsigned i = 0; i < count; ++i) {
		std::unique_ptr<FrameBuffer> buffer = createBuffer(i);
		if (!buffer) {
			LOG(V4L2, Error) << "Unable to create buffer";

			requestBuffers(0, V4L2_MEMORY_MMAP);
			buffers->clear();

			return -EINVAL;
		}

		buffers->push_back(std::move(buffer));
	}

	return count;
}

std::unique_ptr<FrameBuffer> V4L2VideoDevice::createBuffer(unsigned int index)
{
	struct v4l2_plane v4l2Planes[VIDEO_MAX_PLANES] = {};
	struct v4l2_buffer buf = {};

	buf.index = index;
	buf.type = bufferType_;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.length = ARRAY_SIZE(v4l2Planes);
	buf.m.planes = v4l2Planes;

	int ret = ioctl(VIDIOC_QUERYBUF, &buf);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to query buffer " << index << ": "
			<< strerror(-ret);
		return nullptr;
	}

	const bool multiPlanar = V4L2_TYPE_IS_MULTIPLANAR(buf.type);
	const unsigned int numPlanes = multiPlanar ? buf.length : 1;

	if (numPlanes == 0 || numPlanes > VIDEO_MAX_PLANES) {
		LOG(V4L2, Error) << "Invalid number of planes";
		return nullptr;
	}

	std::vector<FrameBuffer::Plane> planes;
	for (unsigned int nplane = 0; nplane < numPlanes; nplane++) {
		FileDescriptor fd = exportDmabufFd(buf.index, nplane);
		if (!fd.isValid())
			return nullptr;

		FrameBuffer::Plane plane;
		plane.fd = std::move(fd);
		plane.length = multiPlanar ?
			buf.m.planes[nplane].length : buf.length;

		planes.push_back(std::move(plane));
	}

	return std::make_unique<FrameBuffer>(std::move(planes));
}

FileDescriptor V4L2VideoDevice::exportDmabufFd(unsigned int index,
					       unsigned int plane)
{
	struct v4l2_exportbuffer expbuf = {};
	int ret;

	expbuf.type = bufferType_;
	expbuf.index = index;
	expbuf.plane = plane;
	expbuf.flags = O_RDWR;

	ret = ioctl(VIDIOC_EXPBUF, &expbuf);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to export buffer: " << strerror(-ret);
		return FileDescriptor();
	}

	return FileDescriptor(std::move(expbuf.fd));
}

/**
 * \brief Prepare the device to import \a count buffers
 * \param[in] count Number of buffers to prepare to import
 *
 * This function initializes the driver's buffer management to import buffers
 * in DMABUF mode. It requests buffers from the driver, but doesn't allocate
 * memory.
 *
 * Upon successful return, the video device is ready to accept queueBuffer()
 * calls. The buffers to be imported are provided to queueBuffer(), and may be
 * supplied externally, or come from a previous exportBuffers() call.
 *
 * Device initialization performed by this function shall later be cleaned up
 * with releaseBuffers(). If buffers have already been allocated with
 * allocateBuffers() or imported with importBuffers(), this function returns
 * -EBUSY.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EBUSY buffers have already been allocated or imported
 */
int V4L2VideoDevice::importBuffers(unsigned int count)
{
	if (cache_) {
		LOG(V4L2, Error) << "Buffers already allocated";
		return -EINVAL;
	}

	memoryType_ = V4L2_MEMORY_DMABUF;

	int ret = requestBuffers(count, V4L2_MEMORY_DMABUF);
	if (ret)
		return ret;

	cache_ = new V4L2BufferCache(count);

	LOG(V4L2, Debug) << "Prepared to import " << count << " buffers";

	return 0;
}

/**
 * \brief Release resources allocated by allocateBuffers() or importBuffers()
 *
 * This function resets the driver's internal buffer management that was
 * initialized by a previous call to allocateBuffers() or importBuffers(). Any
 * memory allocated by allocateBuffers() is freed. Buffer exported by
 * exportBuffers(), if any, are not affected.
 */
int V4L2VideoDevice::releaseBuffers()
{
	LOG(V4L2, Debug) << "Releasing buffers";

	delete cache_;
	cache_ = nullptr;

	return requestBuffers(0, memoryType_);
}

/**
 * \brief Queue a buffer to the video device
 * \param[in] buffer The buffer to be queued
 *
 * For capture video devices the \a buffer will be filled with data by the
 * device. For output video devices the \a buffer shall contain valid data and
 * will be processed by the device. Once the device has finished processing the
 * buffer, it will be available for dequeue.
 *
 * The best available V4L2 buffer is picked for \a buffer using the V4L2 buffer
 * cache.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::queueBuffer(FrameBuffer *buffer)
{
	struct v4l2_plane v4l2Planes[VIDEO_MAX_PLANES] = {};
	struct v4l2_buffer buf = {};
	int ret;

	ret = cache_->get(*buffer);
	if (ret < 0)
		return ret;

	buf.index = ret;
	buf.type = bufferType_;
	buf.memory = memoryType_;
	buf.field = V4L2_FIELD_NONE;

	bool multiPlanar = V4L2_TYPE_IS_MULTIPLANAR(buf.type);
	const std::vector<FrameBuffer::Plane> &planes = buffer->planes();

	if (buf.memory == V4L2_MEMORY_DMABUF) {
		if (multiPlanar) {
			for (unsigned int p = 0; p < planes.size(); ++p)
				v4l2Planes[p].m.fd = planes[p].fd.fd();
		} else {
			buf.m.fd = planes[0].fd.fd();
		}
	}

	if (multiPlanar) {
		buf.length = planes.size();
		buf.m.planes = v4l2Planes;
	}

	if (V4L2_TYPE_IS_OUTPUT(buf.type)) {
		const FrameMetadata &metadata = buffer->metadata();

		if (multiPlanar) {
			unsigned int nplane = 0;
			for (const FrameMetadata::Plane &plane : metadata.planes) {
				v4l2Planes[nplane].bytesused = plane.bytesused;
				v4l2Planes[nplane].length = buffer->planes()[nplane].length;
				nplane++;
			}
		} else {
			if (metadata.planes.size())
				buf.bytesused = metadata.planes[0].bytesused;
		}

		buf.sequence = metadata.sequence;
		buf.timestamp.tv_sec = metadata.timestamp / 1000000000;
		buf.timestamp.tv_usec = (metadata.timestamp / 1000) % 1000000;
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
		fdBufferNotifier_->setEnabled(true);

	queuedBuffers_[buf.index] = buffer;

	return 0;
}

/**
 * \brief Slot to handle completed buffer events from the V4L2 video device
 * \param[in] notifier The event notifier
 *
 * When this slot is called, a Buffer has become available from the device, and
 * will be emitted through the bufferReady Signal.
 *
 * For Capture video devices the FrameBuffer will contain valid data.
 * For Output video devices the FrameBuffer can be considered empty.
 */
void V4L2VideoDevice::bufferAvailable(EventNotifier *notifier)
{
	FrameBuffer *buffer = dequeueBuffer();
	if (!buffer)
		return;

	/* Notify anyone listening to the device. */
	bufferReady.emit(buffer);
}

/**
 * \brief Dequeue the next available buffer from the video device
 *
 * This method dequeues the next available buffer from the device. If no buffer
 * is available to be dequeued it will return nullptr immediately.
 *
 * \return A pointer to the dequeued buffer on success, or nullptr otherwise
 */
FrameBuffer *V4L2VideoDevice::dequeueBuffer()
{
	struct v4l2_buffer buf = {};
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	int ret;

	buf.type = bufferType_;
	buf.memory = memoryType_;

	bool multiPlanar = V4L2_TYPE_IS_MULTIPLANAR(buf.type);

	if (multiPlanar) {
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;
	}

	ret = ioctl(VIDIOC_DQBUF, &buf);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to dequeue buffer: " << strerror(-ret);
		return nullptr;
	}

	LOG(V4L2, Debug) << "Dequeuing buffer " << buf.index;

	cache_->put(buf.index);

	auto it = queuedBuffers_.find(buf.index);
	FrameBuffer *buffer = it->second;
	queuedBuffers_.erase(it);

	if (queuedBuffers_.empty())
		fdBufferNotifier_->setEnabled(false);

	buffer->metadata_.status = buf.flags & V4L2_BUF_FLAG_ERROR
				 ? FrameMetadata::FrameError
				 : FrameMetadata::FrameSuccess;
	buffer->metadata_.sequence = buf.sequence;
	buffer->metadata_.timestamp = buf.timestamp.tv_sec * 1000000000ULL
				    + buf.timestamp.tv_usec * 1000ULL;

	buffer->metadata_.planes.clear();
	if (multiPlanar) {
		for (unsigned int nplane = 0; nplane < buf.length; nplane++)
			buffer->metadata_.planes.push_back({ planes[nplane].bytesused });
	} else {
		buffer->metadata_.planes.push_back({ buf.bytesused });
	}

	return buffer;
}

/**
 * \brief Slot to handle V4L2 events from the V4L2 video device
 * \param[in] notifier The event notifier
 *
 * When this slot is called, a V4L2 event is available to be dequeued from the
 * device.
 */
void V4L2VideoDevice::eventAvailable(EventNotifier *notifier)
{
	struct v4l2_event event{};
	int ret = ioctl(VIDIOC_DQEVENT, &event);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to dequeue event, disabling event notifier";
		fdEventNotifier_->setEnabled(false);
		return;
	}

	if (event.type != V4L2_EVENT_FRAME_SYNC) {
		LOG(V4L2, Error)
			<< "Spurious event (" << event.type
			<< "), disabling event notifier";
		fdEventNotifier_->setEnabled(false);
		return;
	}

	frameStart.emit(event.u.frame_sync.frame_sequence);
}

/**
 * \var V4L2VideoDevice::bufferReady
 * \brief A Signal emitted when a framebuffer completes
 */

/**
 * \brief Enable or disable frame start event notification
 * \param[in] enable True to enable frame start events, false to disable them
 *
 * This function enables or disables generation of frame start events. Once
 * enabled, the events are signalled through the frameStart signal.
 *
 * \return 0 on success, a negative error code otherwise
 */
int V4L2VideoDevice::setFrameStartEnabled(bool enable)
{
	if (frameStartEnabled_ == enable)
		return 0;

	struct v4l2_event_subscription event{};
	event.type = V4L2_EVENT_FRAME_SYNC;

	unsigned long request = enable ? VIDIOC_SUBSCRIBE_EVENT
			      : VIDIOC_UNSUBSCRIBE_EVENT;
	int ret = ioctl(request, &event);
	if (enable && ret)
		return ret;

	fdEventNotifier_->setEnabled(enable);
	frameStartEnabled_ = enable;

	return ret;
}

/**
 * \var V4L2VideoDevice::frameStart
 * \brief A Signal emitted when capture of a frame has started
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
 * immediately dequeued with their status set to FrameMetadata::FrameCancelled,
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
		FrameBuffer *buffer = it.second;

		buffer->metadata_.status = FrameMetadata::FrameCancelled;
		bufferReady.emit(buffer);
	}

	queuedBuffers_.clear();
	fdBufferNotifier_->setEnabled(false);

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
V4L2PixelFormat V4L2VideoDevice::toV4L2PixelFormat(const PixelFormat &pixelFormat)
{
	return V4L2PixelFormat::fromPixelFormat(pixelFormat,
						caps_.isMultiplanar());
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
	fd = syscall(SYS_openat, AT_FDCWD, deviceNode_.c_str(),
		     O_RDWR | O_NONBLOCK);
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
