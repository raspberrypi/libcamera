/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright (C) 2024-2025, Red Hat Inc.
 *
 * Color correction matrix + saturation
 */

#include "ccm.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/matrix.h"

namespace {

constexpr unsigned int kTemperatureThreshold = 100;

}

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftCcm)

int Ccm::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	int ret = ccm_.readYaml(tuningData["ccms"], "ct", "ccm");
	if (ret < 0) {
		LOG(IPASoftCcm, Error)
			<< "Failed to parse 'ccm' parameter from tuning file.";
		return ret;
	}

	context.ccmEnabled = true;
	context.ctrlMap[&controls::Saturation] = ControlInfo(0.0f, 2.0f, 1.0f);

	return 0;
}

int Ccm::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.knobs.saturation = std::optional<double>();

	return 0;
}

void Ccm::queueRequest(typename Module::Context &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] typename Module::FrameContext &frameContext,
		       const ControlList &controls)
{
	const auto &saturation = controls.get(controls::Saturation);
	if (saturation.has_value()) {
		context.activeState.knobs.saturation = saturation;
		LOG(IPASoftCcm, Debug) << "Setting saturation to " << saturation.value();
	}
}

void Ccm::applySaturation(Matrix<float, 3, 3> &ccm, float saturation)
{
	/* https://en.wikipedia.org/wiki/YCbCr#ITU-R_BT.601_conversion */
	const Matrix<float, 3, 3> rgb2ycbcr{
		{ 0.256788235294, 0.504129411765, 0.0979058823529,
		  -0.148223529412, -0.290992156863, 0.439215686275,
		  0.439215686275, -0.367788235294, -0.0714274509804 }
	};
	const Matrix<float, 3, 3> ycbcr2rgb{
		{ 1.16438356164, 0, 1.59602678571,
		  1.16438356164, -0.391762290094, -0.812967647235,
		  1.16438356164, 2.01723214285, 0 }
	};
	const Matrix<float, 3, 3> saturationMatrix{
		{ 1, 0, 0,
		  0, saturation, 0,
		  0, 0, saturation }
	};
	ccm = ycbcr2rgb * saturationMatrix * rgb2ycbcr * ccm;
}

void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, [[maybe_unused]] DebayerParams *params)
{
	auto &saturation = context.activeState.knobs.saturation;

	const unsigned int ct = context.activeState.awb.temperatureK;

	/* Change CCM only on saturation or bigger temperature changes. */
	if (frame > 0 &&
	    utils::abs_diff(ct, lastCt_) < kTemperatureThreshold &&
	    saturation == lastSaturation_) {
		frameContext.ccm.ccm = context.activeState.ccm.ccm;
		context.activeState.ccm.changed = false;
		return;
	}

	lastCt_ = ct;
	lastSaturation_ = saturation;
	Matrix<float, 3, 3> ccm = ccm_.getInterpolated(ct);
	if (saturation)
		applySaturation(ccm, saturation.value());

	context.activeState.ccm.ccm = ccm;
	frameContext.ccm.ccm = ccm;
	frameContext.saturation = saturation;
	context.activeState.ccm.changed = true;
}

void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const SwIspStats *stats,
		  ControlList &metadata)
{
	metadata.set(controls::ColourCorrectionMatrix, frameContext.ccm.ccm.data());

	const auto &saturation = frameContext.saturation;
	metadata.set(controls::Saturation, saturation.value_or(1.0));
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
