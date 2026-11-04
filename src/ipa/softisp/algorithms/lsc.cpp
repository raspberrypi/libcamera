/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Lens shading correction
 */

#include "lsc.h"

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::softisp::algorithms {

LOG_DEFINE_CATEGORY(IPASoftIspLsc)

int Lsc::init(IPAContext &context, const ValueNode &tuningData)
{
	static constexpr unsigned int kGridSize = DebayerParams::kLscGridSize;

	for (unsigned int i = 0; i < kGridSize; i++)
		gridPos_.push_back(static_cast<double>(i) / (kGridSize - 1));

	int ret = lscAlgo_.init(tuningData, context.ctrlMap,
			     { .keys = { "r", "g", "b" },
			       .numHSamples = kGridSize,
			       .numVSamples = kGridSize,
			       .sensorSize = context.sensorInfo.activeArea.size() });
	if (ret)
		return ret;

	context.lscEnabled = true;

	return 0;
}

int Lsc::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return lscAlgo_.configure(context.activeState.lsc,
				  context.sensorInfo.analogCrop,
				  gridPos_, gridPos_);
}

void Lsc::prepare([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	unsigned int ct = frameContext.awb.colourTemperature;
	constexpr unsigned int minTemperatureChange = 100;

	if (!frameContext.lsc.enabled) {
		if (lastAppliedCt_ != 0) {
			params->lscLut = DebayerParams::identityLscLut;
			params->lscLutVersion++;
			lastAppliedCt_ = 0;

			LOG(IPASoftIspLsc, Debug)
				<< "LSC disabled (version " << params->lscLutVersion << ")";
		}
		return;
	}

	if (utils::abs_diff(ct, lastAppliedCt_) < minTemperatureChange)
		return;

	const auto &set = lscAlgo_.interpolateComponents(ct);

	const auto &red = set.at("r");
	const auto &green = set.at("g");
	const auto &blue = set.at("b");

	DebayerParams::LscLookupTable &lut = params->lscLut;
	constexpr unsigned int gridSize = DebayerParams::kLscGridSize;
	for (unsigned int i = 0, j = 0; i < gridSize * gridSize; i++) {
		lut[j++] = red[i];
		lut[j++] = green[i];
		lut[j++] = blue[i];
		lut[j++] = 0; /* padding */
	}
	params->lscLutVersion++;

	LOG(IPASoftIspLsc, Debug)
		<< "LSC updated to version " << params->lscLutVersion
		<< " and temperature " << ct << " (was " << lastAppliedCt_ << ")";

	lastAppliedCt_ = ct;
}

void Lsc::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext, const ControlList &controls)
{
	lscAlgo_.queueRequest(context.activeState.lsc, frameContext.lsc,
			      controls);
}

void Lsc::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const SwIspStats *stats,
		  ControlList &metadata)
{
	lscAlgo_.process(frameContext.lsc, metadata);
}

REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
