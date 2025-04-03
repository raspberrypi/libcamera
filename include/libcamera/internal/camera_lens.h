/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * A camera lens controller
 */
#pragma once

#include <memory>
#include <stdint.h>
#include <string>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>

#include <libcamera/controls.h>

namespace libcamera {

class MediaEntity;
class V4L2Subdevice;

class CameraLens : protected Loggable
{
public:
	explicit CameraLens(const MediaEntity *entity);
	~CameraLens();

	int init();
	int setFocusPosition(int32_t position);

	const std::string &model() const { return model_; }

	const ControlInfoMap &controls() const;

protected:
	std::string logPrefix() const override;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraLens)

	int validateLensDriver();

	const MediaEntity *entity_;
	std::unique_ptr<V4L2Subdevice> subdev_;

	std::string model_;
};

} /* namespace libcamera */
