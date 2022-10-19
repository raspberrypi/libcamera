/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * drm.cpp - DRM/KMS Helpers
 */

#include "drm.h"

#include <algorithm>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <set>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include <libdrm/drm_mode.h>

#include "../common/event_loop.h"

namespace DRM {

Object::Object(Device *dev, uint32_t id, Type type)
	: id_(id), dev_(dev), type_(type)
{
	/* Retrieve properties from the objects that support them. */
	if (type != TypeConnector && type != TypeCrtc &&
	    type != TypeEncoder && type != TypePlane)
		return;

	/*
	 * We can't distinguish between failures due to the object having no
	 * property and failures due to other conditions. Assume we use the API
	 * correctly and consider the object has no property.
	 */
	drmModeObjectProperties *properties = drmModeObjectGetProperties(dev->fd(), id, type);
	if (!properties)
		return;

	properties_.reserve(properties->count_props);
	for (uint32_t i = 0; i < properties->count_props; ++i)
		properties_.emplace_back(properties->props[i],
					 properties->prop_values[i]);

	drmModeFreeObjectProperties(properties);
}

Object::~Object()
{
}

const Property *Object::property(const std::string &name) const
{
	for (const PropertyValue &pv : properties_) {
		const Property *property = static_cast<const Property *>(dev_->object(pv.id()));
		if (property && property->name() == name)
			return property;
	}

	return nullptr;
}

const PropertyValue *Object::propertyValue(const std::string &name) const
{
	for (const PropertyValue &pv : properties_) {
		const Property *property = static_cast<const Property *>(dev_->object(pv.id()));
		if (property && property->name() == name)
			return &pv;
	}

	return nullptr;
}

Property::Property(Device *dev, drmModePropertyRes *property)
	: Object(dev, property->prop_id, TypeProperty),
	  name_(property->name), flags_(property->flags),
	  values_(property->values, property->values + property->count_values),
	  blobs_(property->blob_ids, property->blob_ids + property->count_blobs)
{
	if (drm_property_type_is(property, DRM_MODE_PROP_RANGE))
		type_ = TypeRange;
	else if (drm_property_type_is(property, DRM_MODE_PROP_ENUM))
		type_ = TypeEnum;
	else if (drm_property_type_is(property, DRM_MODE_PROP_BLOB))
		type_ = TypeBlob;
	else if (drm_property_type_is(property, DRM_MODE_PROP_BITMASK))
		type_ = TypeBitmask;
	else if (drm_property_type_is(property, DRM_MODE_PROP_OBJECT))
		type_ = TypeObject;
	else if (drm_property_type_is(property, DRM_MODE_PROP_SIGNED_RANGE))
		type_ = TypeSignedRange;
	else
		type_ = TypeUnknown;

	for (int i = 0; i < property->count_enums; ++i)
		enums_[property->enums[i].value] = property->enums[i].name;
}

Blob::Blob(Device *dev, const libcamera::Span<const uint8_t> &data)
	: Object(dev, 0, Object::TypeBlob)
{
	drmModeCreatePropertyBlob(dev->fd(), data.data(), data.size(), &id_);
}

Blob::~Blob()
{
	if (isValid())
		drmModeDestroyPropertyBlob(device()->fd(), id());
}

Mode::Mode(const drmModeModeInfo &mode)
	: drmModeModeInfo(mode)
{
}

std::unique_ptr<Blob> Mode::toBlob(Device *dev) const
{
	libcamera::Span<const uint8_t> data{ reinterpret_cast<const uint8_t *>(this),
					     sizeof(*this) };
	return std::make_unique<Blob>(dev, data);
}

Crtc::Crtc(Device *dev, const drmModeCrtc *crtc, unsigned int index)
	: Object(dev, crtc->crtc_id, Object::TypeCrtc), index_(index)
{
}

Encoder::Encoder(Device *dev, const drmModeEncoder *encoder)
	: Object(dev, encoder->encoder_id, Object::TypeEncoder),
	  type_(encoder->encoder_type)
{
	const std::list<Crtc> &crtcs = dev->crtcs();
	possibleCrtcs_.reserve(crtcs.size());

	for (const Crtc &crtc : crtcs) {
		if (encoder->possible_crtcs & (1 << crtc.index()))
			possibleCrtcs_.push_back(&crtc);
	}

	possibleCrtcs_.shrink_to_fit();
}

namespace {

const std::map<uint32_t, const char *> connectorTypeNames{
	{ DRM_MODE_CONNECTOR_Unknown, "Unknown" },
	{ DRM_MODE_CONNECTOR_VGA, "VGA" },
	{ DRM_MODE_CONNECTOR_DVII, "DVI-I" },
	{ DRM_MODE_CONNECTOR_DVID, "DVI-D" },
	{ DRM_MODE_CONNECTOR_DVIA, "DVI-A" },
	{ DRM_MODE_CONNECTOR_Composite, "Composite" },
	{ DRM_MODE_CONNECTOR_SVIDEO, "S-Video" },
	{ DRM_MODE_CONNECTOR_LVDS, "LVDS" },
	{ DRM_MODE_CONNECTOR_Component, "Component" },
	{ DRM_MODE_CONNECTOR_9PinDIN, "9-Pin-DIN" },
	{ DRM_MODE_CONNECTOR_DisplayPort, "DP" },
	{ DRM_MODE_CONNECTOR_HDMIA, "HDMI-A" },
	{ DRM_MODE_CONNECTOR_HDMIB, "HDMI-B" },
	{ DRM_MODE_CONNECTOR_TV, "TV" },
	{ DRM_MODE_CONNECTOR_eDP, "eDP" },
	{ DRM_MODE_CONNECTOR_VIRTUAL, "Virtual" },
	{ DRM_MODE_CONNECTOR_DSI, "DSI" },
	{ DRM_MODE_CONNECTOR_DPI, "DPI" },
};

} /* namespace */

Connector::Connector(Device *dev, const drmModeConnector *connector)
	: Object(dev, connector->connector_id, Object::TypeConnector),
	  type_(connector->connector_type)
{
	auto typeName = connectorTypeNames.find(connector->connector_type);
	if (typeName == connectorTypeNames.end()) {
		std::cerr
			<< "Invalid connector type "
			<< connector->connector_type << std::endl;
		typeName = connectorTypeNames.find(DRM_MODE_CONNECTOR_Unknown);
	}

	name_ = std::string(typeName->second) + "-"
	      + std::to_string(connector->connector_type_id);

	switch (connector->connection) {
	case DRM_MODE_CONNECTED:
		status_ = Status::Connected;
		break;

	case DRM_MODE_DISCONNECTED:
		status_ = Status::Disconnected;
		break;

	case DRM_MODE_UNKNOWNCONNECTION:
	default:
		status_ = Status::Unknown;
		break;
	}

	const std::list<Encoder> &encoders = dev->encoders();

	encoders_.reserve(connector->count_encoders);

	for (int i = 0; i < connector->count_encoders; ++i) {
		uint32_t encoderId = connector->encoders[i];
		auto encoder = std::find_if(encoders.begin(), encoders.end(),
					    [=](const Encoder &e) {
						    return e.id() == encoderId;
					    });
		if (encoder == encoders.end()) {
			std::cerr
				<< "Encoder " << encoderId << " not found"
				<< std::endl;
			continue;
		}

		encoders_.push_back(&*encoder);
	}

	encoders_.shrink_to_fit();

	modes_ = { connector->modes, connector->modes + connector->count_modes };
}

Plane::Plane(Device *dev, const drmModePlane *plane)
	: Object(dev, plane->plane_id, Object::TypePlane),
	  possibleCrtcsMask_(plane->possible_crtcs)
{
	formats_ = { plane->formats, plane->formats + plane->count_formats };

	const std::list<Crtc> &crtcs = dev->crtcs();
	possibleCrtcs_.reserve(crtcs.size());

	for (const Crtc &crtc : crtcs) {
		if (plane->possible_crtcs & (1 << crtc.index()))
			possibleCrtcs_.push_back(&crtc);
	}

	possibleCrtcs_.shrink_to_fit();
}

bool Plane::supportsFormat(const libcamera::PixelFormat &format) const
{
	return std::find(formats_.begin(), formats_.end(), format.fourcc())
		!= formats_.end();
}

int Plane::setup()
{
	const PropertyValue *pv = propertyValue("type");
	if (!pv)
		return -EINVAL;

	switch (pv->value()) {
	case DRM_PLANE_TYPE_OVERLAY:
		type_ = TypeOverlay;
		break;

	case DRM_PLANE_TYPE_PRIMARY:
		type_ = TypePrimary;
		break;

	case DRM_PLANE_TYPE_CURSOR:
		type_ = TypeCursor;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

FrameBuffer::FrameBuffer(Device *dev)
	: Object(dev, 0, Object::TypeFb)
{
}

FrameBuffer::~FrameBuffer()
{
	for (const auto &plane : planes_) {
		struct drm_gem_close gem_close = {
			.handle = plane.second.handle,
			.pad = 0,
		};
		int ret;

		do {
			ret = ioctl(device()->fd(), DRM_IOCTL_GEM_CLOSE, &gem_close);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));

		if (ret == -1) {
			ret = -errno;
			std::cerr
				<< "Failed to close GEM object: "
				<< strerror(-ret) << std::endl;
		}
	}

	drmModeRmFB(device()->fd(), id());
}

AtomicRequest::AtomicRequest(Device *dev)
	: dev_(dev), valid_(true)
{
	request_ = drmModeAtomicAlloc();
	if (!request_)
		valid_ = false;
}

AtomicRequest::~AtomicRequest()
{
	if (request_)
		drmModeAtomicFree(request_);
}

int AtomicRequest::addProperty(const Object *object, const std::string &property,
			       uint64_t value)
{
	if (!valid_)
		return -EINVAL;

	const Property *prop = object->property(property);
	if (!prop) {
		valid_ = false;
		return -EINVAL;
	}

	return addProperty(object->id(), prop->id(), value);
}

int AtomicRequest::addProperty(const Object *object, const std::string &property,
			       std::unique_ptr<Blob> blob)
{
	if (!valid_)
		return -EINVAL;

	const Property *prop = object->property(property);
	if (!prop) {
		valid_ = false;
		return -EINVAL;
	}

	int ret = addProperty(object->id(), prop->id(), blob->id());
	if (ret < 0)
		return ret;

	blobs_.emplace_back(std::move(blob));

	return 0;
}

int AtomicRequest::addProperty(uint32_t object, uint32_t property, uint64_t value)
{
	int ret = drmModeAtomicAddProperty(request_, object, property, value);
	if (ret < 0) {
		valid_ = false;
		return ret;
	}

	return 0;
}

int AtomicRequest::commit(unsigned int flags)
{
	if (!valid_)
		return -EINVAL;

	uint32_t drmFlags = 0;
	if (flags & FlagAllowModeset)
		drmFlags |= DRM_MODE_ATOMIC_ALLOW_MODESET;
	if (flags & FlagAsync)
		drmFlags |= DRM_MODE_PAGE_FLIP_EVENT | DRM_MODE_ATOMIC_NONBLOCK;
	if (flags & FlagTestOnly)
		drmFlags |= DRM_MODE_ATOMIC_TEST_ONLY;

	return drmModeAtomicCommit(dev_->fd(), request_, drmFlags, this);
}

Device::Device()
	: fd_(-1)
{
}

Device::~Device()
{
	if (fd_ != -1)
		drmClose(fd_);
}

int Device::init()
{
	int ret = openCard();
	if (ret < 0) {
		std::cerr << "Failed to open any DRM/KMS device: "
			  << strerror(-ret) << std::endl;
		return ret;
	}

	/*
	 * Enable the atomic APIs. This also automatically enables the
	 * universal planes API.
	 */
	ret = drmSetClientCap(fd_, DRM_CLIENT_CAP_ATOMIC, 1);
	if (ret < 0) {
		ret = -errno;
		std::cerr
			<< "Failed to enable atomic capability: "
			<< strerror(-ret) << std::endl;
		return ret;
	}

	/* List all the resources. */
	ret = getResources();
	if (ret < 0)
		return ret;

	EventLoop::instance()->addFdEvent(fd_, EventLoop::Read,
					  std::bind(&Device::drmEvent, this));

	return 0;
}

int Device::openCard()
{
	const std::string dirName = "/dev/dri/";
	bool found = false;
	int ret;

	/*
	 * Open the first DRM/KMS device beginning with /dev/dri/card. The
	 * libdrm drmOpen*() functions require either a module name or a bus ID,
	 * which we don't have, so bypass them. The automatic module loading and
	 * device node creation from drmOpen() is of no practical use as any
	 * modern system will handle that through udev or an equivalent
	 * component.
	 */
	DIR *folder = opendir(dirName.c_str());
	if (!folder) {
		ret = -errno;
		std::cerr << "Failed to open " << dirName
			  << " directory: " << strerror(-ret) << std::endl;
		return ret;
	}

	for (struct dirent *res; (res = readdir(folder));) {
		uint64_t cap;

		if (strncmp(res->d_name, "card", 4))
			continue;

		const std::string devName = dirName + res->d_name;
		fd_ = open(devName.c_str(), O_RDWR | O_CLOEXEC);
		if (fd_ < 0) {
			ret = -errno;
			std::cerr << "Failed to open DRM/KMS device " << devName << ": "
				  << strerror(-ret) << std::endl;
			continue;
		}

		/*
		 * Skip devices that don't support the modeset API, to avoid
		 * selecting a DRM device corresponding to a GPU. There is no
		 * modeset capability, but the kernel returns an error for most
		 * caps if mode setting isn't support by the driver. The
		 * DRM_CAP_DUMB_BUFFER capability is one of those, other would
		 * do as well. The capability value itself isn't relevant.
		 */
		ret = drmGetCap(fd_, DRM_CAP_DUMB_BUFFER, &cap);
		if (ret < 0) {
			drmClose(fd_);
			fd_ = -1;
			continue;
		}

		found = true;
		break;
	}

	closedir(folder);

	return found ? 0 : -ENOENT;
}

int Device::getResources()
{
	int ret;

	std::unique_ptr<drmModeRes, decltype(&drmModeFreeResources)> resources{
		drmModeGetResources(fd_),
		&drmModeFreeResources
	};
	if (!resources) {
		ret = -errno;
		std::cerr
			<< "Failed to get DRM/KMS resources: "
			<< strerror(-ret) << std::endl;
		return ret;
	}

	for (int i = 0; i < resources->count_crtcs; ++i) {
		drmModeCrtc *crtc = drmModeGetCrtc(fd_, resources->crtcs[i]);
		if (!crtc) {
			ret = -errno;
			std::cerr
				<< "Failed to get CRTC: " << strerror(-ret)
				<< std::endl;
			return ret;
		}

		crtcs_.emplace_back(this, crtc, i);
		drmModeFreeCrtc(crtc);

		Crtc &obj = crtcs_.back();
		objects_[obj.id()] = &obj;
	}

	for (int i = 0; i < resources->count_encoders; ++i) {
		drmModeEncoder *encoder =
			drmModeGetEncoder(fd_, resources->encoders[i]);
		if (!encoder) {
			ret = -errno;
			std::cerr
				<< "Failed to get encoder: " << strerror(-ret)
				<< std::endl;
			return ret;
		}

		encoders_.emplace_back(this, encoder);
		drmModeFreeEncoder(encoder);

		Encoder &obj = encoders_.back();
		objects_[obj.id()] = &obj;
	}

	for (int i = 0; i < resources->count_connectors; ++i) {
		drmModeConnector *connector =
			drmModeGetConnector(fd_, resources->connectors[i]);
		if (!connector) {
			ret = -errno;
			std::cerr
				<< "Failed to get connector: " << strerror(-ret)
				<< std::endl;
			return ret;
		}

		connectors_.emplace_back(this, connector);
		drmModeFreeConnector(connector);

		Connector &obj = connectors_.back();
		objects_[obj.id()] = &obj;
	}

	std::unique_ptr<drmModePlaneRes, decltype(&drmModeFreePlaneResources)> planes{
		drmModeGetPlaneResources(fd_),
		&drmModeFreePlaneResources
	};
	if (!planes) {
		ret = -errno;
		std::cerr
			<< "Failed to get DRM/KMS planes: "
			<< strerror(-ret) << std::endl;
		return ret;
	}

	for (uint32_t i = 0; i < planes->count_planes; ++i) {
		drmModePlane *plane =
			drmModeGetPlane(fd_, planes->planes[i]);
		if (!plane) {
			ret = -errno;
			std::cerr
				<< "Failed to get plane: " << strerror(-ret)
				<< std::endl;
			return ret;
		}

		planes_.emplace_back(this, plane);
		drmModeFreePlane(plane);

		Plane &obj = planes_.back();
		objects_[obj.id()] = &obj;
	}

	/* Set the possible planes for each CRTC. */
	for (Crtc &crtc : crtcs_) {
		for (const Plane &plane : planes_) {
			if (plane.possibleCrtcsMask_ & (1 << crtc.index()))
				crtc.planes_.push_back(&plane);
		}
	}

	/* Collect all property IDs and create Property instances. */
	std::set<uint32_t> properties;
	for (const auto &object : objects_) {
		for (const PropertyValue &value : object.second->properties())
			properties.insert(value.id());
	}

	for (uint32_t id : properties) {
		drmModePropertyRes *property = drmModeGetProperty(fd_, id);
		if (!property) {
			ret = -errno;
			std::cerr
				<< "Failed to get property: " << strerror(-ret)
				<< std::endl;
			continue;
		}

		properties_.emplace_back(this, property);
		drmModeFreeProperty(property);

		Property &obj = properties_.back();
		objects_[obj.id()] = &obj;
	}

	/* Finally, perform all delayed setup of mode objects. */
	for (auto &object : objects_) {
		ret = object.second->setup();
		if (ret < 0) {
			std::cerr
				<< "Failed to setup object " << object.second->id()
				<< ": " << strerror(-ret) << std::endl;
			return ret;
		}
	}

	return 0;
}

const Object *Device::object(uint32_t id)
{
	const auto iter = objects_.find(id);
	if (iter == objects_.end())
		return nullptr;

	return iter->second;
}

std::unique_ptr<FrameBuffer> Device::createFrameBuffer(
	const libcamera::FrameBuffer &buffer,
	const libcamera::PixelFormat &format,
	const libcamera::Size &size,
	const std::array<uint32_t, 4> &strides)
{
	std::unique_ptr<FrameBuffer> fb{ new FrameBuffer(this) };

	uint32_t handles[4] = {};
	uint32_t offsets[4] = {};
	int ret;

	const std::vector<libcamera::FrameBuffer::Plane> &planes = buffer.planes();

	unsigned int i = 0;
	for (const libcamera::FrameBuffer::Plane &plane : planes) {
		int fd = plane.fd.get();
		uint32_t handle;

		auto iter = fb->planes_.find(fd);
		if (iter == fb->planes_.end()) {
			ret = drmPrimeFDToHandle(fd_, plane.fd.get(), &handle);
			if (ret < 0) {
				ret = -errno;
				std::cerr
					<< "Unable to import framebuffer dmabuf: "
					<< strerror(-ret) << std::endl;
				return nullptr;
			}

			fb->planes_[fd] = { handle };
		} else {
			handle = iter->second.handle;
		}

		handles[i] = handle;
		offsets[i] = plane.offset;
		++i;
	}

	ret = drmModeAddFB2(fd_, size.width, size.height, format.fourcc(), handles,
			    strides.data(), offsets, &fb->id_, 0);
	if (ret < 0) {
		ret = -errno;
		std::cerr
			<< "Failed to add framebuffer: "
			<< strerror(-ret) << std::endl;
		return nullptr;
	}

	return fb;
}

void Device::drmEvent()
{
	drmEventContext ctx{};
	ctx.version = DRM_EVENT_CONTEXT_VERSION;
	ctx.page_flip_handler = &Device::pageFlipComplete;

	drmHandleEvent(fd_, &ctx);
}

void Device::pageFlipComplete([[maybe_unused]] int fd,
			      [[maybe_unused]] unsigned int sequence,
			      [[maybe_unused]] unsigned int tv_sec,
			      [[maybe_unused]] unsigned int tv_usec,
			      void *user_data)
{
	AtomicRequest *request = static_cast<AtomicRequest *>(user_data);
	request->device()->requestComplete.emit(request);
}

} /* namespace DRM */
