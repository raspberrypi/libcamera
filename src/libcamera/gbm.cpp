/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 * Helper class for managing GBM interactions
 */

#include "libcamera/internal/gbm.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(GBM)

/**
 * \class GBM
 * \brief Helper class for managing GBM interactions
 *
 * The GBM class provides a simplified interface for creating and managing
 * GBM devices. It handles the initialization and teardown of GBM devices
 * used for buffer allocation in graphics and camera pipelines.
 *
 * This class is responsible for opening a DRI render node, creating a GBM
 * device, and providing access to the device and its associated pixel format.
 */

/**
 *\var GBM::fd_
 *\brief file descriptor to DRI device
 */

/**
 *\var GBM::gbmDevice_
 *\brief Pointer to GBM device structure derived from fd_
 */

/**
 *\var GBM::format_
 *\brief Pixel format the GBM surface was created in
 */

/**
 *\brief GBM constructor.
 *
 * Creates a GBM instance with uninitialised state.
 */
GBM::GBM()
{
}

/**
 *\brief GBM destructor
 *
 * Cleans up the GBM device if it was successfully created, and closes
 * the associated file descriptor.
 */
GBM::~GBM()
{
	if (gbmDevice_)
		gbm_device_destroy(gbmDevice_);
}

/**
 * \brief Create and initialize a GBM device
 *
 * \todo Get dri device name from envOption setting
 *
 * Opens the DRI render node (/dev/dri/renderD128) and creates a GBM
 * device using the libgbm library. Sets the default pixel format to
 * ARGB8888.
 *
 * \return 0 on success, or a negative error code on failure
 */
int GBM::createDevice()
{
	const char dri_node[] = "/dev/dri/renderD128";

	int fd = open(dri_node, O_RDWR | O_CLOEXEC | O_NOCTTY);
	if (fd < 0) {
		LOG(GBM, Error) << "Open " << dri_node << " fail " << strerror(errno);
		return -errno;
	}
	fd_ = UniqueFD(fd);

	gbmDevice_ = gbm_create_device(fd_.get());
	if (!gbmDevice_) {
		LOG(GBM, Error) << "gbm_create_device fail" << strerror(errno);
		return -errno;
	}

	format_ = libcamera::formats::ARGB8888;

	return 0;
}

} /* namespace libcamera */
