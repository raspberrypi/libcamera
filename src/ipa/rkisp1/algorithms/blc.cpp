/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Black Level Correction control
 */

#include "blc.h"

#include <linux/videodev2.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

/**
 * \file blc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class BlackLevelCorrection
 * \brief RkISP1 Black Level Correction control
 *
 * The pixels output by the camera normally include a black level, because
 * sensors do not always report a signal level of '0' for black. Pixels at or
 * below this level should be considered black. To achieve that, the RkISP BLC
 * algorithm subtracts a configurable offset from all pixels.
 *
 * The black level can be measured at runtime from an optical dark region of the
 * camera sensor, or measured during the camera tuning process. The first option
 * isn't currently supported.
 */

LOG_DEFINE_CATEGORY(RkISP1Blc)

BlackLevelCorrection::BlackLevelCorrection()
{
	/*
	 * This is a bit of a hack. In raw mode no black level correction
	 * happens. This flag is used to ensure the metadata gets populated with
	 * the black level which is needed to capture proper raw images for
	 * tuning.
	 */
	supportsRaw_ = true;
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int BlackLevelCorrection::init(IPAContext &context, const YamlObject &tuningData)
{
	std::optional<int16_t> levelRed = tuningData["R"].get<int16_t>();
	std::optional<int16_t> levelGreenR = tuningData["Gr"].get<int16_t>();
	std::optional<int16_t> levelGreenB = tuningData["Gb"].get<int16_t>();
	std::optional<int16_t> levelBlue = tuningData["B"].get<int16_t>();
	bool tuningHasLevels = levelRed && levelGreenR && levelGreenB && levelBlue;

	auto blackLevel = context.camHelper->blackLevel();
	if (!blackLevel) {
		/*
		 * Not all camera sensor helpers have been updated with black
		 * levels. Print a warning and fall back to the levels from the
		 * tuning data to preserve backward compatibility. This should
		 * be removed once all helpers provide the data.
		 */
		LOG(RkISP1Blc, Warning)
			<< "No black levels provided by camera sensor helper"
			<< ", please fix";

		blackLevelRed_ = levelRed.value_or(4096);
		blackLevelGreenR_ = levelGreenR.value_or(4096);
		blackLevelGreenB_ = levelGreenB.value_or(4096);
		blackLevelBlue_ = levelBlue.value_or(4096);
	} else if (tuningHasLevels) {
		/*
		 * If black levels are provided in the tuning file, use them to
		 * avoid breaking existing camera tuning. This is deprecated and
		 * will be removed.
		 */
		LOG(RkISP1Blc, Warning)
			<< "Deprecated: black levels overwritten by tuning file";

		blackLevelRed_ = *levelRed;
		blackLevelGreenR_ = *levelGreenR;
		blackLevelGreenB_ = *levelGreenB;
		blackLevelBlue_ = *levelBlue;
	} else {
		blackLevelRed_ = *blackLevel;
		blackLevelGreenR_ = *blackLevel;
		blackLevelGreenB_ = *blackLevel;
		blackLevelBlue_ = *blackLevel;
	}

	LOG(RkISP1Blc, Debug)
		<< "Black levels: red " << blackLevelRed_
		<< ", green (red) " << blackLevelGreenR_
		<< ", green (blue) " << blackLevelGreenB_
		<< ", blue " << blackLevelBlue_;

	return 0;
}

int BlackLevelCorrection::configure(IPAContext &context,
				    [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/*
	 * BLC on ISP versions that include the companding block requires usage
	 * of the extensible parameters format.
	 */
	supported_ = context.configuration.paramFormat == V4L2_META_FMT_RK_ISP1_EXT_PARAMS ||
		     !context.hw->compand;

	if (!supported_)
		LOG(RkISP1Blc, Warning)
			<< "BLC in companding block requires extensible parameters";

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare(IPAContext &context,
				   const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   RkISP1Params *params)
{
	if (context.configuration.raw)
		return;

	if (frame > 0)
		return;

	if (!supported_)
		return;

	if (context.hw->compand) {
		auto config = params->block<BlockType::CompandBls>();
		config.setEnabled(true);

		/*
		 * Scale up to the 20-bit black levels used by the companding
		 * block.
		 */
		config->r = blackLevelRed_ << 4;
		config->gr = blackLevelGreenR_ << 4;
		config->gb = blackLevelGreenB_ << 4;
		config->b = blackLevelBlue_ << 4;
	} else {
		auto config = params->block<BlockType::Bls>();
		config.setEnabled(true);

		config->enable_auto = 0;

		/* Scale down to the 12-bit black levels used by the BLS block. */
		config->fixed_val.r = blackLevelRed_ >> 4;
		config->fixed_val.gr = blackLevelGreenR_ >> 4;
		config->fixed_val.gb = blackLevelGreenB_ >> 4;
		config->fixed_val.b = blackLevelBlue_ >> 4;
	}
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void BlackLevelCorrection::process([[maybe_unused]] IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   [[maybe_unused]] const rkisp1_stat_buffer *stats,
				   ControlList &metadata)
{
	metadata.set(controls::SensorBlackLevels,
		     { static_cast<int32_t>(blackLevelRed_),
		       static_cast<int32_t>(blackLevelGreenR_),
		       static_cast<int32_t>(blackLevelGreenB_),
		       static_cast<int32_t>(blackLevelBlue_) });
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
