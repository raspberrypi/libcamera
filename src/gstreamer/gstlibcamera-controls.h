/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * GStreamer Camera Controls
 */

#pragma once

#include <memory>

#include <libcamera/camera.h>
#include <libcamera/controls.h>
#include <libcamera/request.h>

#include "gstlibcamerasrc.h"

namespace libcamera {

class GstCameraControls
{
public:
	static void installProperties(GObjectClass *klass, int lastProp);

	bool getProperty(guint propId, GValue *value, GParamSpec *pspec);
	bool setProperty(guint propId, const GValue *value, GParamSpec *pspec);

	void setCamera(const std::shared_ptr<libcamera::Camera> &cam);

	void applyControls(std::unique_ptr<libcamera::Request> &request);
	void readMetadata(libcamera::Request *request);

private:
	/* Supported controls and limits of camera. */
	ControlInfoMap capabilities_;
	/* Set of user modified controls. */
	ControlList controls_;
	/* Accumulator of all controls ever set and metadata returned by camera */
	ControlList controls_acc_;
};

} /* namespace libcamera */
