/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Color correction matrix
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

	return 0;
}

void Ccm::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, [[maybe_unused]] DebayerParams *params)
{
	const unsigned int ct = context.activeState.awb.temperatureK;

	/* Change CCM only on bigger temperature changes. */
	if (!currentCcm_ ||
	    utils::abs_diff(ct, lastCt_) >= kTemperatureThreshold) {
		currentCcm_ = ccm_.getInterpolated(ct);
		lastCt_ = ct;
		context.activeState.matrixChanged = true;
	}

	context.activeState.combinedMatrix =
		currentCcm_.value() * context.activeState.combinedMatrix;
	frameContext.ccm = currentCcm_.value();
}

void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const SwIspStats *stats,
		  ControlList &metadata)
{
	metadata.set(controls::ColourCorrectionMatrix, frameContext.ccm.data());
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
