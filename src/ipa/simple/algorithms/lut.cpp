/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Color lookup tables construction
 */

#include "lut.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "simple/ipa_context.h"

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftLut)

namespace ipa::soft::algorithms {

int Lut::init(IPAContext &context,
	      [[maybe_unused]] const YamlObject &tuningData)
{
	context.ctrlMap[&controls::Contrast] = ControlInfo(0.0f, 2.0f, 1.0f);
	return 0;
}

int Lut::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	/* Gamma value is fixed */
	context.configuration.gamma = 0.5;
	context.activeState.knobs.contrast = std::optional<double>();
	updateGammaTable(context);

	return 0;
}

void Lut::queueRequest(typename Module::Context &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] typename Module::FrameContext &frameContext,
		       const ControlList &controls)
{
	const auto &contrast = controls.get(controls::Contrast);
	if (contrast.has_value()) {
		context.activeState.knobs.contrast = contrast;
		LOG(IPASoftLut, Debug) << "Setting contrast to " << contrast.value();
	}
}

void Lut::updateGammaTable(IPAContext &context)
{
	auto &gammaTable = context.activeState.gamma.gammaTable;
	const auto blackLevel = context.activeState.blc.level;
	const unsigned int blackIndex = blackLevel * gammaTable.size() / 256;
	const auto contrast = context.activeState.knobs.contrast.value_or(1.0);

	std::fill(gammaTable.begin(), gammaTable.begin() + blackIndex, 0);
	const float divisor = gammaTable.size() - blackIndex - 1.0;
	for (unsigned int i = blackIndex; i < gammaTable.size(); i++) {
		double normalized = (i - blackIndex) / divisor;
		/* Convert 0..2 to 0..infinity; avoid actual inifinity at tan(pi/2) */
		double contrastExp = tan(std::clamp(contrast * M_PI_4, 0.0, M_PI_2 - 0.00001));
		/* Apply simple S-curve */
		if (normalized < 0.5)
			normalized = 0.5 * std::pow(normalized / 0.5, contrastExp);
		else
			normalized = 1.0 - 0.5 * std::pow((1.0 - normalized) / 0.5, contrastExp);
		gammaTable[i] = UINT8_MAX *
				std::pow(normalized, context.configuration.gamma);
	}

	context.activeState.gamma.blackLevel = blackLevel;
	context.activeState.gamma.contrast = contrast;
}

void Lut::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  [[maybe_unused]] DebayerParams *params)
{
	/*
	 * Update the gamma table if needed. This means if black level changes
	 * and since the black level gets updated only if a lower value is
	 * observed, it's not permanently prone to minor fluctuations or
	 * rounding errors.
	 */
	if (context.activeState.gamma.blackLevel != context.activeState.blc.level ||
	    context.activeState.gamma.contrast != context.activeState.knobs.contrast)
		updateGammaTable(context);

	auto &gains = context.activeState.gains;
	auto &gammaTable = context.activeState.gamma.gammaTable;
	const unsigned int gammaTableSize = gammaTable.size();

	for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
		const double div = static_cast<double>(DebayerParams::kRGBLookupSize) /
				   gammaTableSize;
		/* Apply gamma after gain! */
		unsigned int idx;
		idx = std::min({ static_cast<unsigned int>(i * gains.red / div),
				 gammaTableSize - 1 });
		params->red[i] = gammaTable[idx];
		idx = std::min({ static_cast<unsigned int>(i * gains.green / div),
				 gammaTableSize - 1 });
		params->green[i] = gammaTable[idx];
		idx = std::min({ static_cast<unsigned int>(i * gains.blue / div),
				 gammaTableSize - 1 });
		params->blue[i] = gammaTable[idx];
	}
}

REGISTER_IPA_ALGORITHM(Lut, "Lut")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
