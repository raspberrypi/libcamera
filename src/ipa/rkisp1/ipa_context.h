/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021-2022, Ideas On Board
 *
 * ipa_context.h - RkISP1 IPA Context
 *
 */

#pragma once

#include <linux/rkisp1-config.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::rkisp1 {

struct IPASessionConfiguration {
	struct {
		struct rkisp1_cif_isp_window measureWindow;
	} agc;

	struct {
		struct rkisp1_cif_isp_window measureWindow;
		bool enabled;
	} awb;

	struct {
		bool enabled;
	} lsc;

	struct {
		utils::Duration minShutterSpeed;
		utils::Duration maxShutterSpeed;
		double minAnalogueGain;
		double maxAnalogueGain;

		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
	} sensor;

	struct {
		rkisp1_cif_isp_version revision;
	} hw;

	bool raw;
};

struct IPAActiveState {
	struct {
		struct {
			uint32_t exposure;
			double gain;
		} manual;
		struct {
			uint32_t exposure;
			double gain;
		} automatic;

		bool autoEnabled;
	} agc;

	struct {
		struct {
			struct {
				double red;
				double green;
				double blue;
			} manual;
			struct {
				double red;
				double green;
				double blue;
			} automatic;
		} gains;

		unsigned int temperatureK;
		bool autoEnabled;
	} awb;

	struct {
		int8_t brightness;
		uint8_t contrast;
		uint8_t saturation;
	} cproc;

	struct {
		bool denoise;
	} dpf;

	struct {
		uint8_t denoise;
		uint8_t sharpness;
	} filter;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
		bool autoEnabled;
	} agc;

	struct {
		struct {
			double red;
			double green;
			double blue;
		} gains;

		unsigned int temperatureK;
		bool autoEnabled;
	} awb;

	struct {
		int8_t brightness;
		uint8_t contrast;
		uint8_t saturation;
		bool update;
	} cproc;

	struct {
		bool denoise;
		bool update;
	} dpf;

	struct {
		uint8_t denoise;
		uint8_t sharpness;
		bool update;
	} filter;

	struct {
		uint32_t exposure;
		double gain;
	} sensor;
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
