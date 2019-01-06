/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * device_enumerator.h - API to enumerate and find media devices
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_H__

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <linux/media.h>

namespace libcamera {

class EventNotifier;
class MediaDevice;

class DeviceMatch
{
public:
	DeviceMatch(const std::string &driver);

	void add(const std::string &entity);

	bool match(const MediaDevice *device) const;

private:
	std::string driver_;
	std::vector<std::string> entities_;
};

class DeviceEnumerator
{
public:
	static std::unique_ptr<DeviceEnumerator> create();

	virtual ~DeviceEnumerator();

	virtual int init() = 0;
	virtual int enumerate() = 0;

	std::shared_ptr<MediaDevice> search(const DeviceMatch &dm);

protected:
	int addDevice(const std::string &deviceNode);
	void removeDevice(const std::string &deviceNode);

private:
	std::vector<std::shared_ptr<MediaDevice>> devices_;

	virtual std::string lookupDeviceNode(int major, int minor) = 0;
};

class DeviceEnumeratorUdev: public DeviceEnumerator
{
public:
	DeviceEnumeratorUdev();
	~DeviceEnumeratorUdev();

	int init() final;
	int enumerate() final;

private:
	struct udev *udev_;
	struct udev_monitor *monitor_;
	EventNotifier *notifier_;

	std::string lookupDeviceNode(int major, int minor) final;

	void udevNotify(EventNotifier *notifier);
};

} /* namespace libcamera */

#endif	/* __LIBCAMERA_DEVICE_ENUMERATOR_H__ */
