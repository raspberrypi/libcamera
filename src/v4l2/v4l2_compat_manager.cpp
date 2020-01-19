/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_compat_manager.cpp - V4L2 compatibility manager
 */

#include "v4l2_compat_manager.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <map>
#include <stdarg.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include "log.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(V4L2Compat)

namespace {
template<typename T>
void get_symbol(T &func, const char *name)
{
	func = reinterpret_cast<T>(dlsym(RTLD_NEXT, name));
}
} /* namespace */

V4L2CompatManager::V4L2CompatManager()
	: cm_(nullptr)
{
	get_symbol(fops_.openat, "openat");
	get_symbol(fops_.dup, "dup");
	get_symbol(fops_.close, "close");
	get_symbol(fops_.ioctl, "ioctl");
	get_symbol(fops_.mmap, "mmap");
	get_symbol(fops_.munmap, "munmap");
}

V4L2CompatManager::~V4L2CompatManager()
{
	devices_.clear();
	mmaps_.clear();

	if (cm_) {
		proxies_.clear();
		cm_->stop();
		delete cm_;
		cm_ = nullptr;
	}
}

int V4L2CompatManager::start()
{
	cm_ = new CameraManager();

	int ret = cm_->start();
	if (ret) {
		LOG(V4L2Compat, Error) << "Failed to start camera manager: "
				       << strerror(-ret);
		delete cm_;
		cm_ = nullptr;
		return ret;
	}

	LOG(V4L2Compat, Debug) << "Started camera manager";

	/*
	 * For each Camera registered in the system, a V4L2CameraProxy gets
	 * created here to wrap a camera device.
	 */
	unsigned int index = 0;
	for (auto &camera : cm_->cameras()) {
		V4L2CameraProxy *proxy = new V4L2CameraProxy(index, camera);
		proxies_.emplace_back(proxy);
		++index;
	}

	return 0;
}

V4L2CompatManager *V4L2CompatManager::instance()
{
	static V4L2CompatManager instance;
	return &instance;
}

V4L2CameraProxy *V4L2CompatManager::getProxy(int fd)
{
	auto device = devices_.find(fd);
	if (device == devices_.end())
		return nullptr;

	return device->second;
}

int V4L2CompatManager::getCameraIndex(int fd)
{
	struct stat statbuf;
	int ret = fstat(fd, &statbuf);
	if (ret < 0)
		return -1;

	std::shared_ptr<Camera> target = cm_->get(statbuf.st_rdev);
	if (!target)
		return -1;

	unsigned int index = 0;
	for (auto &camera : cm_->cameras()) {
		if (camera == target)
			return index;
		++index;
	}

	return -1;
}

int V4L2CompatManager::openat(int dirfd, const char *path, int oflag, mode_t mode)
{
	int fd = fops_.openat(dirfd, path, oflag, mode);
	if (fd < 0)
		return fd;

	struct stat statbuf;
	int ret = fstat(fd, &statbuf);
	if (ret < 0 || (statbuf.st_mode & S_IFMT) != S_IFCHR ||
	    major(statbuf.st_rdev) != 81)
		return fd;

	if (!cm_)
		start();

	ret = getCameraIndex(fd);
	if (ret < 0) {
		LOG(V4L2Compat, Info) << "No camera found for " << path;
		return fd;
	}

	fops_.close(fd);

	unsigned int camera_index = static_cast<unsigned int>(ret);

	V4L2CameraProxy *proxy = proxies_[camera_index].get();
	ret = proxy->open(oflag & O_NONBLOCK);
	if (ret < 0)
		return ret;

	int efd = eventfd(0, oflag & (O_CLOEXEC | O_NONBLOCK));
	if (efd < 0) {
		proxy->close();
		return efd;
	}

	devices_.emplace(efd, proxy);

	return efd;
}

int V4L2CompatManager::dup(int oldfd)
{
	int newfd = fops_.dup(oldfd);
	if (newfd < 0)
		return newfd;

	auto device = devices_.find(oldfd);
	if (device != devices_.end()) {
		V4L2CameraProxy *proxy = device->second;
		devices_[newfd] = proxy;
		proxy->dup();
	}

	return newfd;
}

int V4L2CompatManager::close(int fd)
{
	V4L2CameraProxy *proxy = getProxy(fd);
	if (proxy) {
		proxy->close();
		devices_.erase(fd);
		return 0;
	}

	return fops_.close(fd);
}

void *V4L2CompatManager::mmap(void *addr, size_t length, int prot, int flags,
			      int fd, off_t offset)
{
	V4L2CameraProxy *proxy = getProxy(fd);
	if (!proxy)
		return fops_.mmap(addr, length, prot, flags, fd, offset);

	void *map = proxy->mmap(addr, length, prot, flags, offset);
	if (map == MAP_FAILED)
		return map;

	mmaps_[map] = proxy;
	return map;
}

int V4L2CompatManager::munmap(void *addr, size_t length)
{
	auto device = mmaps_.find(addr);
	if (device == mmaps_.end())
		return fops_.munmap(addr, length);

	V4L2CameraProxy *proxy = device->second;

	int ret = proxy->munmap(addr, length);
	if (ret < 0)
		return ret;

	mmaps_.erase(device);

	return 0;
}

int V4L2CompatManager::ioctl(int fd, unsigned long request, void *arg)
{
	V4L2CameraProxy *proxy = getProxy(fd);
	if (!proxy)
		return fops_.ioctl(fd, request, arg);

	return proxy->ioctl(request, arg);
}
