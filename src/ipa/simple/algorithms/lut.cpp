/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Color lookup tables construction
 */

#include "lut.h"

#include <algorithm>
#include <cmath>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "simple/ipa_context.h"

namespace libcamera {

namespace ipa::soft::algorithms {

int Lut::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	/* Gamma value is fixed */
	context.configuration.gamma = 0.5;
	updateGammaTable(context);

	return 0;
}

void Lut::updateGammaTable(IPAContext &context)
{
	auto &gammaTable = context.activeState.gamma.gammaTable;
	auto blackLevel = context.activeState.blc.level;
	const unsigned int blackIndex = blackLevel * gammaTable.size() / 256;

	std::fill(gammaTable.begin(), gammaTable.begin() + blackIndex, 0);
	const float divisor = gammaTable.size() - blackIndex - 1.0;
	for (unsigned int i = blackIndex; i < gammaTable.size(); i++)
		gammaTable[i] = UINT8_MAX * std::pow((i - blackIndex) / divisor,
						     context.configuration.gamma);

	context.activeState.gamma.blackLevel = blackLevel;
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
	if (context.activeState.gamma.blackLevel != context.activeState.blc.level)
		updateGammaTable(context);

	auto &gains = context.activeState.gains;
	auto &gammaTable = context.activeState.gamma.gammaTable;
	const unsigned int gammaTableSize = gammaTable.size();

	for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
		const unsigned int div = static_cast<double>(DebayerParams::kRGBLookupSize) *
					 256 / gammaTableSize;
		/* Apply gamma after gain! */
		unsigned int idx;
		idx = std::min({ i * gains.red / div, gammaTableSize - 1 });
		params->red[i] = gammaTable[idx];
		idx = std::min({ i * gains.green / div, gammaTableSize - 1 });
		params->green[i] = gammaTable[idx];
		idx = std::min({ i * gains.blue / div, gammaTableSize - 1 });
		params->blue[i] = gammaTable[idx];
	}
}

REGISTER_IPA_ALGORITHM(Lut, "Lut")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
