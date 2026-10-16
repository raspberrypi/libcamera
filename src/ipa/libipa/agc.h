/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * Auto exposure/gain algorithm for implementing the IPA-specific AGC algorithms
 */

#pragma once

#include <optional>
#include <stdint.h>
#include <utility>
#include <variant>
#include <vector>

#include <linux/v4l2-controls.h>

#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include "agc_mean_luminance.h"
#include "agc_msv.h"
#include "camera_sensor_helper.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa {

class Histogram;

namespace agc {

struct Session {
	utils::Duration minExposureTime;
	utils::Duration maxExposureTime;
	double minAnalogueGain;
	double maxAnalogueGain;
	double defAnalogueGain;
	utils::Duration minFrameDuration;
	utils::Duration maxFrameDuration;
	utils::Duration lineDuration;

	struct {
		Size outputSize;
	} sensor;

	bool autoAllowed;
};

struct ActiveState {
	struct {
		uint32_t exposure;
		double gain;
	} manual;
	struct {
		uint32_t exposure;
		double gain;
		double quantizationGain;
		double digitalGain;
		double yTarget;
	} automatic;

	bool autoExposureEnabled;
	bool autoGainEnabled;
	double exposureValue;
	controls::AeConstraintModeEnum constraintMode;
	controls::AeExposureModeEnum exposureMode;
	utils::Duration minFrameDuration;
	utils::Duration maxFrameDuration;
};

struct FrameContext {
	uint32_t exposure;
	double gain;
	double quantizationGain;
	double exposureValue;
	double yTarget;
	uint32_t vblank;
	bool autoExposureEnabled;
	bool autoGainEnabled;
	controls::AeConstraintModeEnum constraintMode;
	controls::AeExposureModeEnum exposureMode;
	utils::Duration minFrameDuration;
	utils::Duration maxFrameDuration;
	utils::Duration frameDuration;
	bool autoExposureModeChange;
	bool autoGainModeChange;
};

[[nodiscard]]
inline std::pair<uint32_t, double>
extractControls(const ControlList &controls, const CameraSensorHelper *sensor)
{
	auto exposure = controls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	auto gainCode = controls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();

	return {
		static_cast<uint32_t>(exposure),
		sensor ? sensor->gain(gainCode) : gainCode,
	};
}

inline void
prepareControls(ControlList &controls, const CameraSensorHelper *sensor,
		uint32_t exposure, double gain)
{
	controls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	controls.set(V4L2_CID_ANALOGUE_GAIN,
		     static_cast<int32_t>(sensor
					  ? sensor->gainCode(gain)
					  : static_cast<uint32_t>(gain)));
}

} /* namespace agc */

class AgcAlgorithm
{
public:
	struct ConfigurationParams {
		const IPACameraSensorInfo &sensorInfo;
		const ControlInfoMap &sensorControls;
		ControlInfoMap::Map &ctrlMap;
		bool autoAllowed = true;
	};

	struct ProcessParams {
		const AgcMeanLuminance::Traits &traits;
		const Histogram &yHist;
		uint32_t exposure;
		double gain;
		std::vector<AgcMeanLuminance::AgcConstraint> &&additionalConstraints = {};
		double lux = 0;
	};

	int init(const ValueNode &tuningData, CameraSensorHelper *sensor,
		 const ConfigurationParams &config);

	int configure(agc::Session &session, agc::ActiveState &state,
		      const ConfigurationParams &config);

	void queueRequest(const agc::Session &session, agc::ActiveState &state,
			  agc::FrameContext &frameContext, const ControlList &controls);

	void prepare(agc::ActiveState &state, agc::FrameContext &frameContext);

	void process(const agc::Session &session, agc::ActiveState &state,
		     agc::FrameContext &frameContext, std::optional<ProcessParams> &&params,
		     ControlList &metadata);

private:
	void processFrameDuration(const agc::Session &session,
				  agc::FrameContext &frameContext,
				  utils::Duration frameDuration);
	void fillMetadata(const agc::Session &session,
			  const agc::FrameContext &frameContext,
			  ControlList &metadata);

	std::variant<AgcMSV, AgcMeanLuminance> impl_;
	CameraSensorHelper *sensor_ = nullptr;
};

} /* namespace ipa */

} /* namespace libcamera */
