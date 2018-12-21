/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * device_enumerator.h - API to enumerate and find media devices
 */
#ifndef __LIBCAMERA_DEVICE_ENUMERATOR_H__
#define __LIBCAMERA_DEVICE_ENUMERATOR_H__

#include <map>
#include <string>
#include <vector>

#include <linux/media.h>

namespace libcamera {

class DeviceInfo
{
public:
	DeviceInfo(const std::string &devnode, const struct media_device_info &info,
		   const std::map<std::string, std::string> &entities);

	int acquire();
	void release();
	bool busy() const;

	const std::string &devnode() const;
	const struct media_device_info &info() const;
	std::vector<std::string> entities() const;

	int lookup(const std::string &name, std::string &devnode) const;

private:
	bool acquired_;

	std::string devnode_;
	struct media_device_info info_;
	std::map<std::string, std::string> entities_;
};

class DeviceMatch
{
public:
	DeviceMatch(const std::string &driver);

	void add(const std::string &entity);

	bool match(const DeviceInfo *info) const;

private:
	std::string driver_;
	std::vector<std::string> entities_;
};

class DeviceEnumerator
{
public:
	virtual ~DeviceEnumerator();

	virtual int init() = 0;
	virtual int enumerate() = 0;

	DeviceInfo *search(DeviceMatch &dm) const;

protected:
	int addDevice(const std::string &devnode);

private:
	std::vector<DeviceInfo *> devices_;

	int readInfo(int fd, struct media_device_info &info);
	int readTopology(int fd, std::map<std::string, std::string> &entities);

	virtual std::string lookupDevnode(int major, int minor) = 0;
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

	std::string lookupDevnode(int major, int minor) final;
};

} /* namespace libcamera */

#endif	/* __LIBCAMERA_DEVICE_ENUMERATOR_H__ */
