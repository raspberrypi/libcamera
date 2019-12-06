/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_compat_manager.h - V4L2 compatibility manager
 */

#ifndef __V4L2_COMPAT_MANAGER_H__
#define __V4L2_COMPAT_MANAGER_H__

#include <condition_variable>
#include <fcntl.h>
#include <map>
#include <memory>
#include <mutex>
#include <sys/types.h>
#include <vector>

#include <libcamera/camera_manager.h>

#include "thread.h"
#include "v4l2_camera_proxy.h"

using namespace libcamera;

class V4L2CompatManager : public Thread
{
public:
	static V4L2CompatManager *instance();

	int init();

	V4L2CameraProxy *getProxy(int fd);

	int openat(int dirfd, const char *path, int oflag, mode_t mode);

	int dup(int oldfd);
	int close(int fd);
	void *mmap(void *addr, size_t length, int prot, int flags,
		   int fd, off_t offset);
	int munmap(void *addr, size_t length);
	int ioctl(int fd, unsigned long request, void *arg);

private:
	V4L2CompatManager();
	~V4L2CompatManager();

	void run() override;
	int getCameraIndex(int fd);

	typedef int (*openat_func_t)(int dirfd, const char *path, int oflag, ...);
	typedef int (*dup_func_t)(int oldfd);
	typedef int (*close_func_t)(int fd);
	typedef int (*ioctl_func_t)(int fd, unsigned long request, ...);
	typedef void *(*mmap_func_t)(void *addr, size_t length, int prot,
				     int flags, int fd, off_t offset);
	typedef int (*munmap_func_t)(void *addr, size_t length);

	openat_func_t openat_func_;
	dup_func_t    dup_func_;
	close_func_t  close_func_;
	ioctl_func_t  ioctl_func_;
	mmap_func_t   mmap_func_;
	munmap_func_t munmap_func_;

	CameraManager *cm_;

	std::mutex mutex_;
	std::condition_variable cv_;
	bool initialized_;

	std::vector<std::unique_ptr<V4L2CameraProxy>> proxies_;
	std::map<int, V4L2CameraProxy *> devices_;
	std::map<void *, V4L2CameraProxy *> mmaps_;
};

#endif /* __V4L2_COMPAT_MANAGER_H__ */
