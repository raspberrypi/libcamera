/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Common image adjustments
 */

#include "adjust.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/matrix.h"

namespace libcamera {

namespace ipa::soft::algorithms {

constexpr float kDefaultContrast = 1.0f;
constexpr float kDefaultSaturation = 1.0f;

LOG_DEFINE_CATEGORY(IPASoftAdjust)

int Adjust::init(IPAContext &context, [[maybe_unused]] const YamlObject &tuningData)
{
	context.ctrlMap[&controls::Gamma] =
		ControlInfo(0.1f, 10.0f, kDefaultGamma);
	context.ctrlMap[&controls::Contrast] =
		ControlInfo(0.0f, 2.0f, kDefaultContrast);
	if (context.ccmEnabled)
		context.ctrlMap[&controls::Saturation] =
			ControlInfo(0.0f, 2.0f, kDefaultSaturation);
	return 0;
}

int Adjust::configure(IPAContext &context,
		      [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.knobs.gamma = kDefaultGamma;
	context.activeState.knobs.contrast = std::optional<float>();
	context.activeState.knobs.saturation = std::optional<float>();

	return 0;
}

void Adjust::queueRequest(typename Module::Context &context,
			  [[maybe_unused]] const uint32_t frame,
			  [[maybe_unused]] typename Module::FrameContext &frameContext,
			  const ControlList &controls)
{
	const auto &gamma = controls.get(controls::Gamma);
	if (gamma.has_value()) {
		context.activeState.knobs.gamma = gamma.value();
		LOG(IPASoftAdjust, Debug) << "Setting gamma to " << gamma.value();
	}

	const auto &contrast = controls.get(controls::Contrast);
	if (contrast.has_value()) {
		context.activeState.knobs.contrast = contrast;
		LOG(IPASoftAdjust, Debug) << "Setting contrast to " << contrast.value();
	}

	const auto &saturation = controls.get(controls::Saturation);
	if (saturation.has_value()) {
		context.activeState.knobs.saturation = saturation;
		LOG(IPASoftAdjust, Debug) << "Setting saturation to " << saturation.value();
	}
}

void Adjust::applySaturation(Matrix<float, 3, 3> &matrix, float saturation)
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
	matrix =
		ycbcr2rgb * saturationMatrix * rgb2ycbcr * matrix;
}

void Adjust::prepare(IPAContext &context,
		     [[maybe_unused]] const uint32_t frame,
		     IPAFrameContext &frameContext,
		     [[maybe_unused]] DebayerParams *params)
{
	frameContext.gamma = context.activeState.knobs.gamma;
	frameContext.contrast = context.activeState.knobs.contrast;

	if (!context.ccmEnabled)
		return;

	auto &saturation = context.activeState.knobs.saturation;
	frameContext.saturation = saturation;
	if (saturation)
		applySaturation(context.activeState.combinedMatrix, saturation.value());

	if (saturation != lastSaturation_) {
		context.activeState.matrixChanged = true;
		lastSaturation_ = saturation;
	}
}

void Adjust::process([[maybe_unused]] IPAContext &context,
		     [[maybe_unused]] const uint32_t frame,
		     IPAFrameContext &frameContext,
		     [[maybe_unused]] const SwIspStats *stats,
		     ControlList &metadata)
{
	const auto &gamma = frameContext.gamma;
	metadata.set(controls::Gamma, gamma);

	const auto &contrast = frameContext.contrast;
	metadata.set(controls::Contrast, contrast.value_or(kDefaultContrast));

	const auto &saturation = frameContext.saturation;
	metadata.set(controls::Saturation, saturation.value_or(kDefaultSaturation));
}

REGISTER_IPA_ALGORITHM(Adjust, "Adjust")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
