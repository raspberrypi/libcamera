/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Color Processing control
 */

#include "cproc.h"

#include <algorithm>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

/**
 * \file cproc.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class ColorProcessing
 * \brief RkISP1 Color Processing control
 *
 * The ColorProcessing algorithm is responsible for applying brightness,
 * contrast and saturation corrections. The values are directly provided
 * through requests by the corresponding controls.
 */

LOG_DEFINE_CATEGORY(RkISP1CProc)

namespace {

constexpr float kDefaultBrightness = 0.0f;
constexpr float kDefaultContrast = 1.0f;
constexpr float kDefaultSaturation = 1.0f;

int convertBrightness(const float v)
{
	return std::clamp<int>(std::lround(v * 128), -128, 127);
}

int convertContrastOrSaturation(const float v)
{
	return std::clamp<int>(std::lround(v * 128), 0, 255);
}

} /* namespace */

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int ColorProcessing::init(IPAContext &context,
			  [[maybe_unused]] const YamlObject &tuningData)
{
	auto &cmap = context.ctrlMap;

	cmap[&controls::Brightness] = ControlInfo(-1.0f, 0.993f, kDefaultBrightness);
	cmap[&controls::Contrast] = ControlInfo(0.0f, 1.993f, kDefaultContrast);
	cmap[&controls::Saturation] = ControlInfo(0.0f, 1.993f, kDefaultSaturation);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int ColorProcessing::configure(IPAContext &context,
			       [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	auto &cproc = context.activeState.cproc;

	cproc.brightness = convertBrightness(kDefaultBrightness);
	cproc.contrast = convertContrastOrSaturation(kDefaultContrast);
	cproc.saturation = convertContrastOrSaturation(kDefaultSaturation);

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void ColorProcessing::queueRequest(IPAContext &context,
				   const uint32_t frame,
				   IPAFrameContext &frameContext,
				   const ControlList &controls)
{
	auto &cproc = context.activeState.cproc;
	bool update = false;

	if (frame == 0)
		update = true;

	const auto &brightness = controls.get(controls::Brightness);
	if (brightness) {
		int value = convertBrightness(*brightness);
		if (cproc.brightness != value) {
			cproc.brightness = value;
			update = true;
		}

		LOG(RkISP1CProc, Debug) << "Set brightness to " << value;
	}

	const auto &contrast = controls.get(controls::Contrast);
	if (contrast) {
		int value = convertContrastOrSaturation(*contrast);
		if (cproc.contrast != value) {
			cproc.contrast = value;
			update = true;
		}

		LOG(RkISP1CProc, Debug) << "Set contrast to " << value;
	}

	const auto saturation = controls.get(controls::Saturation);
	if (saturation) {
		int value = convertContrastOrSaturation(*saturation);
		if (cproc.saturation != value) {
			cproc.saturation = value;
			update = true;
		}

		LOG(RkISP1CProc, Debug) << "Set saturation to " << value;
	}

	frameContext.cproc.brightness = cproc.brightness;
	frameContext.cproc.contrast = cproc.contrast;
	frameContext.cproc.saturation = cproc.saturation;
	frameContext.cproc.update = update;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void ColorProcessing::prepare([[maybe_unused]] IPAContext &context,
			      [[maybe_unused]] const uint32_t frame,
			      IPAFrameContext &frameContext,
			      RkISP1Params *params)
{
	/* Check if the algorithm configuration has been updated. */
	if (!frameContext.cproc.update)
		return;

	auto config = params->block<BlockType::Cproc>();
	config.setEnabled(true);
	config->brightness = frameContext.cproc.brightness;
	config->contrast = frameContext.cproc.contrast;
	config->sat = frameContext.cproc.saturation;
}

REGISTER_IPA_ALGORITHM(ColorProcessing, "ColorProcessing")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
