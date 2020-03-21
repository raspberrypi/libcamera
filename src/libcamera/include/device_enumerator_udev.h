/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018-2019, Google Inc.
 *
 * device_enumerator_udev.h - udev-based device enumerator
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <sys/types.h>

#include "device_enumerator.h"

struct udev;
struct udev_device;
struct udev_monitor;

namespace libcamera {

class EventNotifier;
class MediaDevice;
class MediaEntity;

class DeviceEnumeratorUdev : public DeviceEnumerator
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

	using DependencyMap = std::map<dev_t, std::list<MediaEntity *>>;

	struct MediaDeviceDeps {
		MediaDeviceDeps(std::unique_ptr<MediaDevice> &&media,
				DependencyMap &&deps)
			: media_(std::move(media)), deps_(std::move(deps))
		{
		}

		bool operator==(const MediaDeviceDeps &other) const
		{
			return media_ == other.media_;
		}

		std::unique_ptr<MediaDevice> media_;
		DependencyMap deps_;
	};

	std::set<dev_t> orphans_;
	std::list<MediaDeviceDeps> pending_;
	std::map<dev_t, MediaDeviceDeps *> devMap_;

	int addUdevDevice(struct udev_device *dev);
	int populateMediaDevice(MediaDevice *media, DependencyMap *deps);
	std::string lookupDeviceNode(dev_t devnum);

	int addV4L2Device(dev_t devnum);
	void udevNotify(EventNotifier *notifier);
};

} /* namespace libcamera */

#endif	/* __LIBCAMERA_DEVICE_ENUMERATOR_UDEV_H__ */
