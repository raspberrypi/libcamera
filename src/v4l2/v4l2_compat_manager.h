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
	struct FileOperations {
		using openat_func_t = int (*)(int dirfd, const char *path,
					      int oflag, ...);
		using dup_func_t = int (*)(int oldfd);
		using close_func_t = int (*)(int fd);
		using ioctl_func_t = int (*)(int fd, unsigned long request, ...);
		using mmap_func_t = void *(*)(void *addr, size_t length, int prot,
					      int flags, int fd, off_t offset);
		using munmap_func_t = int (*)(void *addr, size_t length);

		openat_func_t openat;
		dup_func_t dup;
		close_func_t close;
		ioctl_func_t ioctl;
		mmap_func_t mmap;
		munmap_func_t munmap;
	};

	static V4L2CompatManager *instance();

	int init();

	V4L2CameraProxy *getProxy(int fd);
	const FileOperations &fops() const { return fops_; }

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

	FileOperations fops_;

	CameraManager *cm_;

	std::mutex mutex_;
	std::condition_variable cv_;
	bool initialized_;

	std::vector<std::unique_ptr<V4L2CameraProxy>> proxies_;
	std::map<int, V4L2CameraProxy *> devices_;
	std::map<void *, V4L2CameraProxy *> mmaps_;
};

#endif /* __V4L2_COMPAT_MANAGER_H__ */
