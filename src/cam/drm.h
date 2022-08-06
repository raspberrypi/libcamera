/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Ideas on Board Oy
 *
 * drm.h - DRM/KMS Helpers
 */

#pragma once

#include <array>
#include <list>
#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#include <libcamera/base/signal.h>
#include <libcamera/base/span.h>

#include <libdrm/drm.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

namespace libcamera {
class FrameBuffer;
class PixelFormat;
class Size;
} /* namespace libcamera */

namespace DRM {

class Device;
class Plane;
class Property;
class PropertyValue;

class Object
{
public:
	enum Type {
		TypeCrtc = DRM_MODE_OBJECT_CRTC,
		TypeConnector = DRM_MODE_OBJECT_CONNECTOR,
		TypeEncoder = DRM_MODE_OBJECT_ENCODER,
		TypeMode = DRM_MODE_OBJECT_MODE,
		TypeProperty = DRM_MODE_OBJECT_PROPERTY,
		TypeFb = DRM_MODE_OBJECT_FB,
		TypeBlob = DRM_MODE_OBJECT_BLOB,
		TypePlane = DRM_MODE_OBJECT_PLANE,
		TypeAny = DRM_MODE_OBJECT_ANY,
	};

	Object(Device *dev, uint32_t id, Type type);
	virtual ~Object();

	Device *device() const { return dev_; }
	uint32_t id() const { return id_; }
	Type type() const { return type_; }

	const Property *property(const std::string &name) const;
	const PropertyValue *propertyValue(const std::string &name) const;
	const std::vector<PropertyValue> &properties() const { return properties_; }

protected:
	virtual int setup()
	{
		return 0;
	}

	uint32_t id_;

private:
	friend Device;

	Device *dev_;
	Type type_;
	std::vector<PropertyValue> properties_;
};

class Property : public Object
{
public:
	enum Type {
		TypeUnknown = 0,
		TypeRange,
		TypeEnum,
		TypeBlob,
		TypeBitmask,
		TypeObject,
		TypeSignedRange,
	};

	Property(Device *dev, drmModePropertyRes *property);

	Type type() const { return type_; }
	const std::string &name() const { return name_; }

	bool isImmutable() const { return flags_ & DRM_MODE_PROP_IMMUTABLE; }

	const std::vector<uint64_t> values() const { return values_; }
	const std::map<uint32_t, std::string> &enums() const { return enums_; }
	const std::vector<uint32_t> blobs() const { return blobs_; }

private:
	Type type_;
	std::string name_;
	uint32_t flags_;
	std::vector<uint64_t> values_;
	std::map<uint32_t, std::string> enums_;
	std::vector<uint32_t> blobs_;
};

class PropertyValue
{
public:
	PropertyValue(uint32_t id, uint64_t value)
		: id_(id), value_(value)
	{
	}

	uint32_t id() const { return id_; }
	uint32_t value() const { return value_; }

private:
	uint32_t id_;
	uint64_t value_;
};

class Blob : public Object
{
public:
	Blob(Device *dev, const libcamera::Span<const uint8_t> &data);
	~Blob();

	bool isValid() const { return id() != 0; }
};

class Mode : public drmModeModeInfo
{
public:
	Mode(const drmModeModeInfo &mode);

	std::unique_ptr<Blob> toBlob(Device *dev) const;
};

class Crtc : public Object
{
public:
	Crtc(Device *dev, const drmModeCrtc *crtc, unsigned int index);

	unsigned int index() const { return index_; }
	const std::vector<const Plane *> &planes() const { return planes_; }

private:
	friend Device;

	unsigned int index_;
	std::vector<const Plane *> planes_;
};

class Encoder : public Object
{
public:
	Encoder(Device *dev, const drmModeEncoder *encoder);

	uint32_t type() const { return type_; }

	const std::vector<const Crtc *> &possibleCrtcs() const { return possibleCrtcs_; }

private:
	uint32_t type_;
	std::vector<const Crtc *> possibleCrtcs_;
};

class Connector : public Object
{
public:
	enum Status {
		Connected,
		Disconnected,
		Unknown,
	};

	Connector(Device *dev, const drmModeConnector *connector);

	uint32_t type() const { return type_; }
	const std::string &name() const { return name_; }

	Status status() const { return status_; }

	const std::vector<const Encoder *> &encoders() const { return encoders_; }
	const std::vector<Mode> &modes() const { return modes_; }

private:
	uint32_t type_;
	std::string name_;
	Status status_;
	std::vector<const Encoder *> encoders_;
	std::vector<Mode> modes_;
};

class Plane : public Object
{
public:
	enum Type {
		TypeOverlay,
		TypePrimary,
		TypeCursor,
	};

	Plane(Device *dev, const drmModePlane *plane);

	Type type() const { return type_; }
	const std::vector<uint32_t> &formats() const { return formats_; }
	const std::vector<const Crtc *> &possibleCrtcs() const { return possibleCrtcs_; }

	bool supportsFormat(const libcamera::PixelFormat &format) const;

protected:
	int setup() override;

private:
	friend class Device;

	Type type_;
	std::vector<uint32_t> formats_;
	std::vector<const Crtc *> possibleCrtcs_;
	uint32_t possibleCrtcsMask_;
};

class FrameBuffer : public Object
{
public:
	struct Plane {
		uint32_t handle;
	};

	~FrameBuffer();

private:
	friend class Device;

	FrameBuffer(Device *dev);

	std::map<int, Plane> planes_;
};

class AtomicRequest
{
public:
	enum Flags {
		FlagAllowModeset = (1 << 0),
		FlagAsync = (1 << 1),
		FlagTestOnly = (1 << 2),
	};

	AtomicRequest(Device *dev);
	~AtomicRequest();

	Device *device() const { return dev_; }
	bool isValid() const { return valid_; }

	int addProperty(const Object *object, const std::string &property,
			uint64_t value);
	int addProperty(const Object *object, const std::string &property,
			std::unique_ptr<Blob> blob);
	int commit(unsigned int flags = 0);

private:
	AtomicRequest(const AtomicRequest &) = delete;
	AtomicRequest(const AtomicRequest &&) = delete;
	AtomicRequest &operator=(const AtomicRequest &) = delete;
	AtomicRequest &operator=(const AtomicRequest &&) = delete;

	int addProperty(uint32_t object, uint32_t property, uint64_t value);

	Device *dev_;
	bool valid_;
	drmModeAtomicReq *request_;
	std::list<std::unique_ptr<Blob>> blobs_;
};

class Device
{
public:
	Device();
	~Device();

	int init();

	int fd() const { return fd_; }

	const std::list<Crtc> &crtcs() const { return crtcs_; }
	const std::list<Encoder> &encoders() const { return encoders_; }
	const std::list<Connector> &connectors() const { return connectors_; }
	const std::list<Plane> &planes() const { return planes_; }
	const std::list<Property> &properties() const { return properties_; }

	const Object *object(uint32_t id);

	std::unique_ptr<FrameBuffer> createFrameBuffer(
		const libcamera::FrameBuffer &buffer,
		const libcamera::PixelFormat &format,
		const libcamera::Size &size,
		const std::array<uint32_t, 4> &strides);

	libcamera::Signal<AtomicRequest *> requestComplete;

private:
	Device(const Device &) = delete;
	Device(const Device &&) = delete;
	Device &operator=(const Device &) = delete;
	Device &operator=(const Device &&) = delete;

	int openCard();
	int getResources();

	void drmEvent();
	static void pageFlipComplete(int fd, unsigned int sequence,
				     unsigned int tv_sec, unsigned int tv_usec,
				     void *user_data);

	int fd_;

	std::list<Crtc> crtcs_;
	std::list<Encoder> encoders_;
	std::list<Connector> connectors_;
	std::list<Plane> planes_;
	std::list<Property> properties_;

	std::map<uint32_t, Object *> objects_;
};

} /* namespace DRM */
