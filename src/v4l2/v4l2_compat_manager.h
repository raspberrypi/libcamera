/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * V4L2 compatibility manager
 */

#pragma once

#include <fcntl.h>
#include <map>
#include <memory>
#include <sys/types.h>
#include <vector>

#include <libcamera/camera_manager.h>

#include "v4l2_camera_proxy.h"

class V4L2CompatManager
{
public:
	struct FileOperations {
		using openat_func_t = int (*)(int dirfd, const char *path,
					      int oflag, ...);
		using dup_func_t = int (*)(int oldfd);
		using close_func_t = int (*)(int fd);
		using ioctl_func_t = int (*)(int fd, unsigned long request, ...);
		using mmap_func_t = void *(*)(void *addr, size_t length, int prot,
					      int flags, int fd, off64_t offset);
		using munmap_func_t = int (*)(void *addr, size_t length);

		openat_func_t openat;
		dup_func_t dup;
		close_func_t close;
		ioctl_func_t ioctl;
		mmap_func_t mmap;
		munmap_func_t munmap;
	};

	static V4L2CompatManager *instance();

	const FileOperations &fops() const { return fops_; }

	int openat(int dirfd, const char *path, int oflag, mode_t mode);

	int dup(int oldfd);
	int close(int fd);
	void *mmap(void *addr, size_t length, int prot, int flags,
		   int fd, off64_t offset);
	int munmap(void *addr, size_t length);
	int ioctl(int fd, unsigned long request, void *arg);

private:
	V4L2CompatManager();
	~V4L2CompatManager();

	int start();
	int getCameraIndex(int fd);
	std::shared_ptr<V4L2CameraFile> cameraFile(int fd);

	FileOperations fops_;

	libcamera::CameraManager *cm_;

	std::vector<std::unique_ptr<V4L2CameraProxy>> proxies_;
	std::map<int, std::shared_ptr<V4L2CameraFile>> files_;
	std::map<void *, std::shared_ptr<V4L2CameraFile>> mmaps_;
};
