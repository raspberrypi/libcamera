/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * RkISP1 Filter control
 */

#include "filter.h"

#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

/**
 * \file filter.h
 */

namespace libcamera {

namespace ipa::rkisp1::algorithms {

/**
 * \class Filter
 * \brief RkISP1 Filter control
 *
 * Denoise and Sharpness filters will be applied by RkISP1 during the
 * demosaicing step. The denoise filter is responsible for removing noise from
 * the image, while the sharpness filter will enhance its acutance.
 *
 * \todo In current version the denoise and sharpness control is based on user
 * controls. In a future version it should be controlled automatically by the
 * algorithm.
 */

LOG_DEFINE_CATEGORY(RkISP1Filter)

static constexpr uint32_t kFiltLumWeightDefault = 0x00022040;
static constexpr uint32_t kFiltModeDefault = 0x000004f2;

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Filter::init(IPAContext &context,
		 [[maybe_unused]] const YamlObject &tuningData)
{
	auto &cmap = context.ctrlMap;
	cmap[&controls::Sharpness] = ControlInfo(0.0f, 10.0f, 1.0f);

	return 0;
}
/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Filter::queueRequest(IPAContext &context,
			  [[maybe_unused]] const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls)
{
	auto &filter = context.activeState.filter;
	bool update = false;

	const auto &sharpness = controls.get(controls::Sharpness);
	if (sharpness) {
		unsigned int value = std::round(std::clamp(*sharpness, 0.0f, 10.0f));

		if (filter.sharpness != value) {
			filter.sharpness = value;
			update = true;
		}

		LOG(RkISP1Filter, Debug) << "Set sharpness to " << *sharpness;
	}

	const auto &denoise = controls.get(controls::draft::NoiseReductionMode);
	if (denoise) {
		LOG(RkISP1Filter, Debug) << "Set denoise to " << *denoise;

		switch (*denoise) {
		case controls::draft::NoiseReductionModeOff:
			if (filter.denoise != 0) {
				filter.denoise = 0;
				update = true;
			}
			break;
		case controls::draft::NoiseReductionModeMinimal:
			if (filter.denoise != 1) {
				filter.denoise = 1;
				update = true;
			}
			break;
		case controls::draft::NoiseReductionModeHighQuality:
		case controls::draft::NoiseReductionModeFast:
			if (filter.denoise != 3) {
				filter.denoise = 3;
				update = true;
			}
			break;
		default:
			LOG(RkISP1Filter, Error)
				<< "Unsupported denoise value "
				<< *denoise;
			break;
		}
	}

	frameContext.filter.denoise = filter.denoise;
	frameContext.filter.sharpness = filter.sharpness;
	frameContext.filter.update = update;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Filter::prepare([[maybe_unused]] IPAContext &context,
		     [[maybe_unused]] const uint32_t frame,
		     IPAFrameContext &frameContext, RkISP1Params *params)
{
	/* Check if the algorithm configuration has been updated. */
	if (!frameContext.filter.update)
		return;

	static constexpr uint16_t filt_fac_sh0[] = {
		0x04, 0x07, 0x0a, 0x0c, 0x10, 0x14, 0x1a, 0x1e, 0x24, 0x2a, 0x30
	};

	static constexpr uint16_t filt_fac_sh1[] = {
		0x04, 0x08, 0x0c, 0x10, 0x16, 0x1b, 0x20, 0x26, 0x2c, 0x30, 0x3f
	};

	static constexpr uint16_t filt_fac_mid[] = {
		0x04, 0x06, 0x08, 0x0a, 0x0c, 0x10, 0x13, 0x17, 0x1d, 0x22, 0x28
	};

	static constexpr uint16_t filt_fac_bl0[] = {
		0x02, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x10, 0x15, 0x1a, 0x24
	};

	static constexpr uint16_t filt_fac_bl1[] = {
		0x00, 0x00, 0x00, 0x02, 0x04, 0x04, 0x06, 0x08, 0x0d, 0x14, 0x20
	};

	static constexpr uint16_t filt_thresh_sh0[] = {
		0, 18, 26, 36, 41, 75, 90, 120, 170, 250, 1023
	};

	static constexpr uint16_t filt_thresh_sh1[] = {
		0, 33, 44, 51, 67, 100, 120, 150, 200, 300, 1023
	};

	static constexpr uint16_t filt_thresh_bl0[] = {
		0, 8, 13, 23, 26, 50, 60, 80, 140, 180, 1023
	};

	static constexpr uint16_t filt_thresh_bl1[] = {
		0, 2, 5, 10, 15, 20, 26, 51, 100, 150, 1023
	};

	static constexpr uint16_t stage1_select[] = {
		6, 6, 4, 4, 3, 3, 2, 2, 2, 1, 0
	};

	static constexpr uint16_t filt_chr_v_mode[] = {
		1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
	};

	static constexpr uint16_t filt_chr_h_mode[] = {
		0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
	};

	uint8_t denoise = frameContext.filter.denoise;
	uint8_t sharpness = frameContext.filter.sharpness;

	auto config = params->block<BlockType::Flt>();
	config.setEnabled(true);

	config->fac_sh0 = filt_fac_sh0[sharpness];
	config->fac_sh1 = filt_fac_sh1[sharpness];
	config->fac_mid = filt_fac_mid[sharpness];
	config->fac_bl0 = filt_fac_bl0[sharpness];
	config->fac_bl1 = filt_fac_bl1[sharpness];

	config->lum_weight = kFiltLumWeightDefault;
	config->mode = kFiltModeDefault;
	config->thresh_sh0 = filt_thresh_sh0[denoise];
	config->thresh_sh1 = filt_thresh_sh1[denoise];
	config->thresh_bl0 = filt_thresh_bl0[denoise];
	config->thresh_bl1 = filt_thresh_bl1[denoise];
	config->grn_stage1 = stage1_select[denoise];
	config->chr_v_mode = filt_chr_v_mode[denoise];
	config->chr_h_mode = filt_chr_h_mode[denoise];

	/*
	 * Combined high denoising and high sharpening requires some
	 * adjustments to the configuration of the filters. A first stage
	 * filter with a lower strength must be selected, and the blur factors
	 * must be decreased.
	 */
	if (denoise == 9) {
		if (sharpness > 3)
			config->grn_stage1 = 2;
	} else if (denoise == 10) {
		if (sharpness > 5)
			config->grn_stage1 = 2;
		else if (sharpness > 3)
			config->grn_stage1 = 1;
	}

	if (denoise > 7) {
		if (sharpness > 7) {
			config->fac_bl0 /= 2;
			config->fac_bl1 /= 4;
		} else if (sharpness > 4) {
			config->fac_bl0 = config->fac_bl0 * 3 / 4;
			config->fac_bl1 /= 2;
		}
	}
}

REGISTER_IPA_ALGORITHM(Filter, "Filter")

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
