/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * IPU3 IPA Context
 *
 */

#pragma once

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include <libipa/agc.h>
#include <libipa/awb.h>
#include <libipa/camera_sensor_helper.h>
#include <libipa/ccm.h>
#include <libipa/fc_queue.h>
#include <libipa/gamma.h>
#include <libipa/lsc.h>

namespace libcamera {

namespace ipa::ipu3 {

struct IPASessionConfiguration {
	struct {
		ipu3_uapi_grid_config bdsGrid;
		Size bdsOutputSize;
		uint32_t stride;
	} grid;

	struct {
		ipu3_uapi_grid_config afGrid;
	} af;

	agc::Session agc;
};

struct IPAActiveState {
	struct {
		uint32_t focus;
		double maxVariance;
		bool stable;
	} af;

	agc::ActiveState agc;
	ipa::awb::ActiveState awb;
	ipa::ccm::ActiveState ccm;
	ipa::gamma::ActiveState gamma;
	ipa::lsc::ActiveState lsc;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
	} sensor;

	agc::FrameContext agc;
	ipa::awb::FrameContext awb;
	ipa::ccm::FrameContext ccm;
	ipa::gamma::FrameContext gamma;
	ipa::lsc::FrameContext lsc;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPASessionConfiguration configuration;
	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	std::unique_ptr<CameraSensorHelper> camHelper;

	ControlInfoMap::Map ctrlMap;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/
