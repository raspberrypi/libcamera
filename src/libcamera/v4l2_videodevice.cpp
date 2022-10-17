/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_videodevice.cpp - V4L2 Video Device
 */

#include "libcamera/internal/v4l2_videodevice.h"

#include <algorithm>
#include <array>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include <linux/version.h>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/unique_fd.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"

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
 * \fn V4L2Capability::hasMediaController()
 * \brief Determine if the video device uses Media Controller to configure I/O
 * \return True if the video device is controlled by a Media Controller device
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
				    *buffer.get());
}

V4L2BufferCache::~V4L2BufferCache()
{
	if (missCounter_ > cache_.size())
		LOG(V4L2, Debug) << "Cache misses: " << missCounter_;
}

/**
 * \brief Check if all the entries in the cache are unused
 */
bool V4L2BufferCache::isEmpty() const
{
	for (auto const &entry : cache_) {
		if (!entry.free_)
			return false;
	}

	return true;
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

		if (!entry.free_)
			continue;

		/* Try to find a cache hit by comparing the planes. */
		if (entry == buffer) {
			hit = true;
			use = index;
			break;
		}

		if (entry.lastUsed_ < oldest) {
			use = index;
			oldest = entry.lastUsed_;
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
	cache_[index].free_ = true;
}

V4L2BufferCache::Entry::Entry()
	: free_(true), lastUsed_(0)
{
}

V4L2BufferCache::Entry::Entry(bool free, uint64_t lastUsed, const FrameBuffer &buffer)
	: free_(free), lastUsed_(lastUsed)
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
		if (planes_[i].fd != planes[i].fd.get() ||
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
 * \struct V4L2DeviceFormat::Plane
 * \brief Per-plane memory size information
 * \var V4L2DeviceFormat::Plane::size
 * \brief The plane total memory size (in bytes)
 * \var V4L2DeviceFormat::Plane::bpl
 * \brief The plane line stride (in bytes)
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
 * \var V4L2DeviceFormat::colorSpace
 * \brief The color space of the pixels
 *
 * The color space of the image. When setting the format this may be
 * unset, in which case the driver gets to use its default color space.
 * After being set, this value should contain the color space that
 * was actually used. If this value is unset, then the color space chosen
 * by the driver could not be represented by the ColorSpace class (and
 * should probably be added).
 *
 * It is up to the pipeline handler or application to check if the
 * resulting color space is acceptable.
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
	ss << *this;

	return ss.str();
}

/**
 * \brief Insert a text representation of a V4L2DeviceFormat into an output
 * stream
 * \param[in] out The output stream
 * \param[in] f The V4L2DeviceFormat
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const V4L2DeviceFormat &f)
{
	out << f.size << "-" << f.fourcc;
	return out;
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
	: V4L2Device(deviceNode), formatInfo_(nullptr), cache_(nullptr),
	  fdBufferNotifier_(nullptr), state_(State::Stopped),
	  watchdogDuration_(0.0)
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
	watchdog_.timeout.connect(this, &V4L2VideoDevice::watchdogExpired);
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

	LOG(V4L2, Debug)
		<< "Opened device " << caps_.bus_info() << ": "
		<< caps_.driver() << ": " << caps_.card();

	ret = initFormats();
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Open a V4L2 video device from an opened file handle and query its
 * capabilities
 * \param[in] handle The file descriptor to set
 * \param[in] type The device type to operate on
 *
 * This function opens a video device from the existing file descriptor \a
 * handle. Like open(), this function queries the capabilities of the device,
 * but handles it according to the given device \a type instead of determining
 * its type from the capabilities. This can be used to force a given device type
 * for memory-to-memory devices.
 *
 * The file descriptor \a handle is duplicated, no reference to the original
 * handle is kept.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::open(SharedFD handle, enum v4l2_buf_type type)
{
	int ret;

	UniqueFD newFd = handle.dup();
	if (!newFd.isValid()) {
		ret = -errno;
		LOG(V4L2, Error) << "Failed to duplicate file handle: "
				 << strerror(-ret);
		return ret;
	}

	ret = V4L2Device::setFd(std::move(newFd));
	if (ret < 0) {
		LOG(V4L2, Error) << "Failed to set file handle: "
				 << strerror(-ret);
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

	LOG(V4L2, Debug)
		<< "Opened device " << caps_.bus_info() << ": "
		<< caps_.driver() << ": " << caps_.card();

	ret = initFormats();
	if (ret)
		return ret;

	return 0;
}

int V4L2VideoDevice::initFormats()
{
	const std::vector<V4L2PixelFormat> &deviceFormats = enumPixelformats(0);
	if (deviceFormats.empty()) {
		LOG(V4L2, Error) << "Failed to initialize device formats";
		return -EINVAL;
	}

	pixelFormats_ = { deviceFormats.begin(), deviceFormats.end() };

	int ret = getFormat(&format_);
	if (ret) {
		LOG(V4L2, Error) << "Failed to get format";
		return ret;
	}

	formatInfo_ = &PixelFormatInfo::info(format_.fourcc);

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

	formatInfo_ = nullptr;

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
	return deviceNode() + "[" + std::to_string(fd()) +
		(V4L2_TYPE_IS_OUTPUT(bufferType_) ? ":out]" : ":cap]");
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
	int ret = 0;
	if (caps_.isMeta())
		ret = trySetFormatMeta(format, true);
	else if (caps_.isMultiplanar())
		ret = trySetFormatMultiplane(format, true);
	else
		ret = trySetFormatSingleplane(format, true);

	/* Cache the set format on success. */
	if (ret)
		return ret;

	format_ = *format;
	formatInfo_ = &PixelFormatInfo::info(format_.fourcc);

	return 0;
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

template<typename T>
std::optional<ColorSpace> V4L2VideoDevice::toColorSpace(const T &v4l2Format)
{
	V4L2PixelFormat fourcc{ v4l2Format.pixelformat };
	return V4L2Device::toColorSpace(v4l2Format, PixelFormatInfo::info(fourcc).colourEncoding);
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
	format->colorSpace = toColorSpace(*pix);

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
	if (format->colorSpace) {
		fromColorSpace(format->colorSpace, *pix);

		if (caps_.isVideoCapture())
			pix->flags |= V4L2_PIX_FMT_FLAG_SET_CSC;
	}

	ASSERT(pix->num_planes <= std::size(pix->plane_fmt));

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
	format->colorSpace = toColorSpace(*pix);

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
	format->colorSpace = toColorSpace(*pix);

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
	if (format->colorSpace) {
		fromColorSpace(format->colorSpace, *pix);

		if (caps_.isVideoCapture())
			pix->flags |= V4L2_PIX_FMT_FLAG_SET_CSC;
	}

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
	format->colorSpace = toColorSpace(*pix);

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

	if (code && !caps_.hasMediaController()) {
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
 * The number of planes and their offsets and sizes are determined by the
 * currently active format on the device as set by setFormat(). They do not map
 * to the V4L2 buffer planes, but to colour planes of the pixel format. For
 * instance, if the active format is formats::NV12, the allocated FrameBuffer
 * instances will have two planes, for the luma and chroma components,
 * regardless of whether the device uses V4L2_PIX_FMT_NV12 or
 * V4L2_PIX_FMT_NV12M.
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
 * The number of planes and their offsets and sizes are determined by the
 * currently active format on the device as set by setFormat(). They do not map
 * to the V4L2 buffer planes, but to colour planes of the pixel format. For
 * instance, if the active format is formats::NV12, the allocated FrameBuffer
 * instances will have two planes, for the luma and chroma components,
 * regardless of whether the device uses V4L2_PIX_FMT_NV12 or
 * V4L2_PIX_FMT_NV12M.
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
	buf.length = std::size(v4l2Planes);
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
		UniqueFD fd = exportDmabufFd(buf.index, nplane);
		if (!fd.isValid())
			return nullptr;

		FrameBuffer::Plane plane;
		plane.fd = SharedFD(std::move(fd));
		/*
		 * V4L2 API doesn't provide dmabuf offset information of plane.
		 * Set 0 as a placeholder offset.
		 * \todo Set the right offset once V4L2 API provides a way.
		 */
		plane.offset = 0;
		plane.length = multiPlanar ? buf.m.planes[nplane].length : buf.length;

		planes.push_back(std::move(plane));
	}

	/*
	 * If we have a multi-planar format with a V4L2 single-planar buffer,
	 * split the single V4L2 plane into multiple FrameBuffer planes by
	 * computing the offsets manually.
	 *
	 * The format info is not guaranteed to be valid, as there are no
	 * PixelFormatInfo for metadata formats, so check it first.
	 */
	if (formatInfo_->isValid() && formatInfo_->numPlanes() != numPlanes) {
		/*
		 * There's no valid situation where the number of colour planes
		 * differs from the number of V4L2 planes and the V4L2 buffer
		 * has more than one plane.
		 */
		ASSERT(numPlanes == 1u);

		planes.resize(formatInfo_->numPlanes());
		const SharedFD &fd = planes[0].fd;
		size_t offset = 0;

		for (auto [i, plane] : utils::enumerate(planes)) {
			/*
			 * The stride is reported by V4L2 for the first plane
			 * only. Compute the stride of the other planes by
			 * taking the horizontal subsampling factor into
			 * account, which is equal to the bytesPerGroup ratio of
			 * the planes.
			 */
			unsigned int stride = format_.planes[0].bpl
					    * formatInfo_->planes[i].bytesPerGroup
					    / formatInfo_->planes[0].bytesPerGroup;

			plane.fd = fd;
			plane.offset = offset;
			plane.length = formatInfo_->planeSize(format_.size.height,
							      i, stride);
			offset += plane.length;
		}
	}

	return std::make_unique<FrameBuffer>(planes);
}

UniqueFD V4L2VideoDevice::exportDmabufFd(unsigned int index,
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
		return {};
	}

	return UniqueFD(expbuf.fd);
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
	if (!cache_)
		return 0;

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
 * Note that queueBuffer() will fail if the device is in the process of being
 * stopped from a streaming state through streamOff().
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::queueBuffer(FrameBuffer *buffer)
{
	struct v4l2_plane v4l2Planes[VIDEO_MAX_PLANES] = {};
	struct v4l2_buffer buf = {};
	int ret;

	if (state_ == State::Stopping) {
		LOG(V4L2, Error) << "Device is in a stopping state.";
		return -ESHUTDOWN;
	}

	/*
	 * Pipeline handlers should not requeue buffers after releasing the
	 * buffers on the device. Any occurence of this error should be fixed
	 * in the pipeline handler directly.
	 */
	if (!cache_) {
		LOG(V4L2, Fatal) << "No BufferCache available to queue.";
		return -ENOENT;
	}

	ret = cache_->get(*buffer);
	if (ret < 0)
		return ret;

	buf.index = ret;
	buf.type = bufferType_;
	buf.memory = memoryType_;
	buf.field = V4L2_FIELD_NONE;

	bool multiPlanar = V4L2_TYPE_IS_MULTIPLANAR(buf.type);
	const std::vector<FrameBuffer::Plane> &planes = buffer->planes();
	const unsigned int numV4l2Planes = format_.planesCount;

	/*
	 * Ensure that the frame buffer has enough planes, and that they're
	 * contiguous if the V4L2 format requires them to be.
	 */
	if (planes.size() < numV4l2Planes) {
		LOG(V4L2, Error) << "Frame buffer has too few planes";
		return -EINVAL;
	}

	if (planes.size() != numV4l2Planes && !buffer->_d()->isContiguous()) {
		LOG(V4L2, Error) << "Device format requires contiguous buffer";
		return -EINVAL;
	}

	if (buf.memory == V4L2_MEMORY_DMABUF) {
		if (multiPlanar) {
			for (unsigned int p = 0; p < numV4l2Planes; ++p)
				v4l2Planes[p].m.fd = planes[p].fd.get();
		} else {
			buf.m.fd = planes[0].fd.get();
		}
	}

	if (multiPlanar) {
		buf.length = numV4l2Planes;
		buf.m.planes = v4l2Planes;
	}

	if (V4L2_TYPE_IS_OUTPUT(buf.type)) {
		const FrameMetadata &metadata = buffer->metadata();

		for (const auto &plane : metadata.planes()) {
			if (!plane.bytesused)
				LOG(V4L2, Warning) << "byteused == 0 is deprecated";
		}

		if (numV4l2Planes != planes.size()) {
			/*
			 * If we have a multi-planar buffer with a V4L2
			 * single-planar format, coalesce all planes. The length
			 * and number of bytes used may only differ in the last
			 * plane as any other situation can't be represented.
			 */
			unsigned int bytesused = 0;
			unsigned int length = 0;

			for (auto [i, plane] : utils::enumerate(planes)) {
				bytesused += metadata.planes()[i].bytesused;
				length += plane.length;

				if (i != planes.size() - 1 && bytesused != length) {
					LOG(V4L2, Error)
						<< "Holes in multi-planar buffer not supported";
					return -EINVAL;
				}
			}

			if (multiPlanar) {
				v4l2Planes[0].bytesused = bytesused;
				v4l2Planes[0].length = length;
			} else {
				buf.bytesused = bytesused;
				buf.length = length;
			}
		} else if (multiPlanar) {
			/*
			 * If we use the multi-planar API, fill in the planes.
			 * The number of planes in the frame buffer and in the
			 * V4L2 buffer is guaranteed to be equal at this point.
			 */
			for (auto [i, plane] : utils::enumerate(planes)) {
				v4l2Planes[i].bytesused = metadata.planes()[i].bytesused;
				v4l2Planes[i].length = plane.length;
			}
		} else {
			/*
			 * Single-planar API with a single plane in the buffer
			 * is trivial to handle.
			 */
			buf.bytesused = metadata.planes()[0].bytesused;
			buf.length = planes[0].length;
		}

		/*
		 * Timestamps are to be supplied if the device is a mem-to-mem
		 * device. The drivers will have V4L2_BUF_FLAG_TIMESTAMP_COPY
		 * set hence these timestamps will be copied from the output
		 * buffers to capture buffers. If the device is not mem-to-mem,
		 * there is no harm in setting the timestamps as they will be
		 * ignored (and over-written).
		 */
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

	if (queuedBuffers_.empty()) {
		fdBufferNotifier_->setEnabled(true);
		if (watchdogDuration_)
			watchdog_.start(std::chrono::duration_cast<std::chrono::milliseconds>(watchdogDuration_));
	}

	queuedBuffers_[buf.index] = buffer;

	return 0;
}

/**
 * \brief Slot to handle completed buffer events from the V4L2 video device
 *
 * When this slot is called, a Buffer has become available from the device, and
 * will be emitted through the bufferReady Signal.
 *
 * For Capture video devices the FrameBuffer will contain valid data.
 * For Output video devices the FrameBuffer can be considered empty.
 */
void V4L2VideoDevice::bufferAvailable()
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
 * This function dequeues the next available buffer from the device. If no
 * buffer is available to be dequeued it will return nullptr immediately.
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

	/*
	 * If the video node fails to stream-on successfully (which can occur
	 * when queuing a buffer), a vb2 kernel bug can lead to the buffer which
	 * returns a failure upon queuing being mistakenly kept in the kernel.
	 * This leads to the kernel notifying us that a buffer is available to
	 * dequeue, which we have no awareness of being queued, and thus we will
	 * not find it in the queuedBuffers_ list.
	 *
	 * Whilst this kernel bug has been fixed in mainline, ensure that we
	 * safely ignore buffers which are unexpected to prevent crashes on
	 * older kernels.
	 */
	auto it = queuedBuffers_.find(buf.index);
	if (it == queuedBuffers_.end()) {
		LOG(V4L2, Error)
			<< "Dequeued unexpected buffer index " << buf.index;

		return nullptr;
	}

	cache_->put(buf.index);

	FrameBuffer *buffer = it->second;
	queuedBuffers_.erase(it);

	if (queuedBuffers_.empty()) {
		fdBufferNotifier_->setEnabled(false);
		watchdog_.stop();
	} else if (watchdogDuration_) {
		/*
		 * Restart the watchdog timer if there are buffers still queued
		 * in the device.
		 */
		watchdog_.start(std::chrono::duration_cast<std::chrono::milliseconds>(watchdogDuration_));
	}

	FrameMetadata &metadata = buffer->_d()->metadata();

	metadata.status = buf.flags & V4L2_BUF_FLAG_ERROR
			? FrameMetadata::FrameError
			: FrameMetadata::FrameSuccess;
	metadata.sequence = buf.sequence;
	metadata.timestamp = buf.timestamp.tv_sec * 1000000000ULL
			   + buf.timestamp.tv_usec * 1000ULL;

	if (V4L2_TYPE_IS_OUTPUT(buf.type))
		return buffer;

	/*
	 * Detect kernel drivers which do not reset the sequence number to zero
	 * on stream start.
	 */
	if (!firstFrame_) {
		if (buf.sequence)
			LOG(V4L2, Info)
				<< "Zero sequence expected for first frame (got "
				<< buf.sequence << ")";
		firstFrame_ = buf.sequence;
	}
	metadata.sequence -= firstFrame_.value();

	unsigned int numV4l2Planes = multiPlanar ? buf.length : 1;

	if (numV4l2Planes != buffer->planes().size()) {
		/*
		 * If we have a multi-planar buffer with a V4L2
		 * single-planar format, split the V4L2 buffer across
		 * the buffer planes. Only the last plane may have less
		 * bytes used than its length.
		 */
		if (numV4l2Planes != 1) {
			LOG(V4L2, Error)
				<< "Invalid number of planes (" << numV4l2Planes
				<< " != " << buffer->planes().size() << ")";

			metadata.status = FrameMetadata::FrameError;
			return buffer;
		}

		/*
		 * With a V4L2 single-planar format, all the data is stored in
		 * a single memory plane. The number of bytes used is conveyed
		 * through that plane when using the V4L2 multi-planar API, or
		 * set directly in the buffer when using the V4L2 single-planar
		 * API.
		 */
		unsigned int bytesused = multiPlanar ? planes[0].bytesused
				       : buf.bytesused;
		unsigned int remaining = bytesused;

		for (auto [i, plane] : utils::enumerate(buffer->planes())) {
			if (!remaining) {
				LOG(V4L2, Error)
					<< "Dequeued buffer (" << bytesused
					<< " bytes) too small for plane lengths "
					<< utils::join(buffer->planes(), "/",
						       [](const FrameBuffer::Plane &p) {
							       return p.length;
						       });

				metadata.status = FrameMetadata::FrameError;
				return buffer;
			}

			metadata.planes()[i].bytesused =
				std::min(plane.length, remaining);
			remaining -= metadata.planes()[i].bytesused;
		}
	} else if (multiPlanar) {
		/*
		 * If we use the multi-planar API, fill in the planes.
		 * The number of planes in the frame buffer and in the
		 * V4L2 buffer is guaranteed to be equal at this point.
		 */
		for (unsigned int i = 0; i < numV4l2Planes; ++i)
			metadata.planes()[i].bytesused = planes[i].bytesused;
	} else {
		metadata.planes()[0].bytesused = buf.bytesused;
	}

	return buffer;
}

/**
 * \var V4L2VideoDevice::bufferReady
 * \brief A Signal emitted when a framebuffer completes
 */

/**
 * \brief Start the video stream
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::streamOn()
{
	int ret;

	firstFrame_.reset();

	ret = ioctl(VIDIOC_STREAMON, &bufferType_);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to start streaming: " << strerror(-ret);
		return ret;
	}

	state_ = State::Streaming;
	if (watchdogDuration_ && !queuedBuffers_.empty())
		watchdog_.start(std::chrono::duration_cast<std::chrono::milliseconds>(watchdogDuration_));

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
 * This will be a no-op if the stream is not started in the first place and
 * has no queued buffers.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2VideoDevice::streamOff()
{
	int ret;

	if (state_ != State::Streaming && queuedBuffers_.empty())
		return 0;

	if (watchdogDuration_.count())
		watchdog_.stop();

	ret = ioctl(VIDIOC_STREAMOFF, &bufferType_);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Failed to stop streaming: " << strerror(-ret);
		return ret;
	}

	state_ = State::Stopping;

	/* Send back all queued buffers. */
	for (auto it : queuedBuffers_) {
		FrameBuffer *buffer = it.second;
		FrameMetadata &metadata = buffer->_d()->metadata();

		cache_->put(it.first);
		metadata.status = FrameMetadata::FrameCancelled;
		bufferReady.emit(buffer);
	}

	ASSERT(cache_->isEmpty());

	queuedBuffers_.clear();
	fdBufferNotifier_->setEnabled(false);
	state_ = State::Stopped;

	return 0;
}

/**
 * \brief Set the dequeue timeout value
 * \param[in] timeout The timeout value to be used
 *
 * Sets a timeout value, given by \a timeout, that will be used by a watchdog
 * timer to ensure buffer dequeue events are periodically occurring when the
 * device is streaming. The watchdog timer is only active when the device is
 * streaming, so it is not necessary to disable it when the device stops
 * streaming. The timeout value can be safely updated at any time.
 *
 * If the timer expires, the \ref V4L2VideoDevice::dequeueTimeout signal is
 * emitted. This can typically be used by pipeline handlers to be notified of
 * stalled devices.
 *
 * Set \a timeout to 0 to disable the watchdog timer.
 */
void V4L2VideoDevice::setDequeueTimeout(utils::Duration timeout)
{
	watchdogDuration_ = timeout;

	watchdog_.stop();
	if (watchdogDuration_ && state_ == State::Streaming && !queuedBuffers_.empty())
		watchdog_.start(std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
}

/**
 * \var V4L2VideoDevice::dequeueTimeout
 * \brief A Signal emitted when the dequeue watchdog timer expires
 */

/**
 * \brief Slot to handle an expired dequeue timer
 *
 * When this slot is called, the time between successive dequeue events is over
 * the required timeout. Emit the \ref V4L2VideoDevice::dequeueTimeout signal.
 */
void V4L2VideoDevice::watchdogExpired()
{
	LOG(V4L2, Warning)
		<< "Dequeue timer of " << watchdogDuration_ << " has expired!";

	dequeueTimeout.emit();
}

/**
 * \brief Create a new video device instance from \a entity in media device
 * \a media
 * \param[in] media The media device where the entity is registered
 * \param[in] entity The media entity name
 *
 * \return A newly created V4L2VideoDevice on success, nullptr otherwise
 */
std::unique_ptr<V4L2VideoDevice>
V4L2VideoDevice::fromEntityName(const MediaDevice *media,
				const std::string &entity)
{
	MediaEntity *mediaEntity = media->getEntityByName(entity);
	if (!mediaEntity)
		return nullptr;

	return std::make_unique<V4L2VideoDevice>(mediaEntity);
}

/**
 * \brief Convert \a PixelFormat to a V4L2PixelFormat supported by the device
 * \param[in] pixelFormat The PixelFormat to convert
 *
 * Convert \a pixelformat to a V4L2 FourCC that is known to be supported by
 * the video device.
 *
 * A V4L2VideoDevice may support different V4L2 pixel formats that map the same
 * PixelFormat. This is the case of the contiguous and non-contiguous variants
 * of multiplanar formats, and with the V4L2 MJPEG and JPEG pixel formats.
 * Converting a PixelFormat to a V4L2PixelFormat may thus have multiple answers.
 *
 * This function converts the \a pixelFormat using the list of V4L2 pixel
 * formats that the V4L2VideoDevice supports. This guarantees that the returned
 * V4L2PixelFormat will be valid for the device. If multiple matches are still
 * possible, contiguous variants are preferred. If the \a pixelFormat is not
 * supported by the device, the function returns an invalid V4L2PixelFormat.
 *
 * \return The V4L2PixelFormat corresponding to \a pixelFormat if supported by
 * the device, or an invalid V4L2PixelFormat otherwise
 */
V4L2PixelFormat V4L2VideoDevice::toV4L2PixelFormat(const PixelFormat &pixelFormat) const
{
	const std::vector<V4L2PixelFormat> &v4l2PixelFormats =
		V4L2PixelFormat::fromPixelFormat(pixelFormat);

	for (const V4L2PixelFormat &v4l2Format : v4l2PixelFormats) {
		if (pixelFormats_.count(v4l2Format))
			return v4l2Format;
	}

	return {};
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
	int ret;

	/*
	 * The output and capture V4L2VideoDevice instances use the same file
	 * handle for the same device node.
	 */
	SharedFD fd(syscall(SYS_openat, AT_FDCWD, deviceNode_.c_str(),
			    O_RDWR | O_NONBLOCK));
	if (!fd.isValid()) {
		ret = -errno;
		LOG(V4L2, Error) << "Failed to open V4L2 M2M device: "
				 << strerror(-ret);
		return ret;
	}

	ret = output_->open(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (ret)
		goto err;

	ret = capture_->open(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (ret)
		goto err;

	return 0;

err:
	close();

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
