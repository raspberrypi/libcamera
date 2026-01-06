/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 * Helper class for managing GBM interactions
 */

#pragma once

#include <gbm.h>

#include <libcamera/base/log.h>
#include <libcamera/base/unique_fd.h>

#include <libcamera/formats.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(GBM)

class GBM
{
public:
	GBM();
	~GBM();

	int createDevice();

	/**
	 * \brief Retrieve the GBM device handle
	 *
	 * \return Pointer to the gbm_device structure, or nullptr if the device
	 * has not been created
	 */
	struct gbm_device *device() const { return gbmDevice_; }

	/**
	 * \brief Retrieve the pixel format
	 *
	 * \return The PixelFormat used by this GBM instance (ARGB8888)
	 */
	PixelFormat format() const { return format_; }

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(GBM)

	UniqueFD fd_;
	struct gbm_device *gbmDevice_ = nullptr;
	PixelFormat format_;
};

} /* namespace libcamera */
