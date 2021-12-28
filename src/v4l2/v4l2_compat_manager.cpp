/* SPDX-License-Identifier: LGPL-2.1-or-later */
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

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include "v4l2_camera_file.h"

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
	get_symbol(fops_.openat, "openat64");
	get_symbol(fops_.dup, "dup");
	get_symbol(fops_.close, "close");
	get_symbol(fops_.ioctl, "ioctl");
	get_symbol(fops_.mmap, "mmap64");
	get_symbol(fops_.munmap, "munmap");
}

V4L2CompatManager::~V4L2CompatManager()
{
	files_.clear();
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
	auto cameras = cm_->cameras();
	for (auto [index, camera] : utils::enumerate(cameras)) {
		V4L2CameraProxy *proxy = new V4L2CameraProxy(index, camera);
		proxies_.emplace_back(proxy);
	}

	return 0;
}

V4L2CompatManager *V4L2CompatManager::instance()
{
	static V4L2CompatManager instance;
	return &instance;
}

std::shared_ptr<V4L2CameraFile> V4L2CompatManager::cameraFile(int fd)
{
	auto file = files_.find(fd);
	if (file == files_.end())
		return nullptr;

	return file->second;
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

	auto cameras = cm_->cameras();
	for (auto [index, camera] : utils::enumerate(cameras)) {
		if (camera == target)
			return index;
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
		LOG(V4L2Compat, Debug) << "No camera found for " << path;
		return fd;
	}

	fops_.close(fd);

	int efd = eventfd(0, EFD_SEMAPHORE |
			     ((oflag & O_CLOEXEC) ? EFD_CLOEXEC : 0) |
			     ((oflag & O_NONBLOCK) ? EFD_NONBLOCK : 0));
	if (efd < 0)
		return efd;

	V4L2CameraProxy *proxy = proxies_[ret].get();
	files_.emplace(efd, std::make_shared<V4L2CameraFile>(dirfd, path, efd,
							     oflag & O_NONBLOCK,
							     proxy));

	LOG(V4L2Compat, Debug) << "Opened " << path << " -> fd " << efd;
	return efd;
}

int V4L2CompatManager::dup(int oldfd)
{
	int newfd = fops_.dup(oldfd);
	if (newfd < 0)
		return newfd;

	auto file = files_.find(oldfd);
	if (file != files_.end())
		files_[newfd] = file->second;

	return newfd;
}

int V4L2CompatManager::close(int fd)
{
	auto file = files_.find(fd);
	if (file != files_.end())
		files_.erase(file);

	/* We still need to close the eventfd. */
	return fops_.close(fd);
}

void *V4L2CompatManager::mmap(void *addr, size_t length, int prot, int flags,
			      int fd, off64_t offset)
{
	std::shared_ptr<V4L2CameraFile> file = cameraFile(fd);
	if (!file)
		return fops_.mmap(addr, length, prot, flags, fd, offset);

	void *map = file->proxy()->mmap(file.get(), addr, length, prot, flags,
					offset);
	if (map == MAP_FAILED)
		return map;

	mmaps_[map] = file;
	return map;
}

int V4L2CompatManager::munmap(void *addr, size_t length)
{
	auto device = mmaps_.find(addr);
	if (device == mmaps_.end())
		return fops_.munmap(addr, length);

	V4L2CameraFile *file = device->second.get();

	int ret = file->proxy()->munmap(file, addr, length);
	if (ret < 0)
		return ret;

	mmaps_.erase(device);

	return 0;
}

int V4L2CompatManager::ioctl(int fd, unsigned long request, void *arg)
{
	std::shared_ptr<V4L2CameraFile> file = cameraFile(fd);
	if (!file)
		return fops_.ioctl(fd, request, arg);

	return file->proxy()->ioctl(file.get(), request, arg);
}
