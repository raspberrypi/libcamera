/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <libcamera/base/log.h>

#include <libipa/histogram.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftIspExposure)

namespace ipa::softisp::algorithms {

namespace {

class AgcTraits : public AgcMeanLuminance::Traits
{
public:
	AgcTraits(const Histogram &yHist)
		: yHist_(yHist)
	{
	}

	double estimateLuminance(double gain) const override
	{
		/*
		 * \todo Improve by asking the weight of saturating and
		 * non-saturating bins directly from the histogram.
		 */
		double sum = 0;

		for (size_t i = 0; i < yHist_.bins(); i++)
			sum += std::min<double>(yHist_.bins(), i * gain) * yHist_[i];

		return sum / yHist_.total() / yHist_.bins();
	}

private:
	const Histogram &yHist_;
};

} /* namespace */

int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	return agc_.init(tuningData, context.camHelper.get(), {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
}

int Agc::configure(IPAContext &context, [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	return agc_.configure(context.configuration.agc, context.activeState.agc, {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
}

void Agc::queueRequest(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext, const ControlList &controls)
{
	agc_.queueRequest(context.configuration.agc, context.activeState.agc, frameContext.agc, controls);
}

void Agc::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, [[maybe_unused]] DebayerParams *params)
{
	agc_.prepare(context.activeState.agc, frameContext.agc);
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	if (stats->valid) {
		/* \todo The histogram should come already adjusted. */
		auto histogram = stats->yHistogram;

		const unsigned int blackLevelHistIdx =
			context.activeState.blc.level * histogram.size() / 256;
		for (unsigned int i = 0; i < blackLevelHistIdx; i++)
			histogram[blackLevelHistIdx] += histogram[i];

		Histogram yHist({ histogram.begin() + blackLevelHistIdx, histogram.end() });

		agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {{
			.traits = AgcTraits(yHist),
			.yHist = yHist,
			.exposure = frameContext.sensor.exposure,
			.gain = frameContext.sensor.gain,
		}}, metadata);
	} else {
		agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {}, metadata);
	}
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::softisp::algorithms */

} /* namespace libcamera */
