/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Denoise.cpp - Denoise (spatial, colour, temporal) control algorithm
 */
#include "denoise.h"

#include <libcamera/base/log.h>

#include "denoise_status.h"
#include "noise_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiDenoise)

// Calculate settings for the denoise blocks using the noise profile in
// the image metadata.

#define NAME "rpi.denoise"

Denoise::Denoise(Controller *controller)
	: DenoiseAlgorithm(controller), mode_(DenoiseMode::ColourHighQuality)
{
}

char const *Denoise::name() const
{
	return NAME;
}

int Denoise::read(const libcamera::YamlObject &params)
{
	sdnEnable_ = params.contains("sdn");
	if (sdnEnable_) {
		auto &sdnParams = params["sdn"];
		sdnDeviation_ = sdnParams["deviation"].get<double>(3.2);
		sdnStrength_ = sdnParams["strength"].get<double>(0.25);
		sdnDeviation2_ = sdnParams["deviation2"].get<double>(sdnDeviation_);
		sdnDeviationNoTdn_ = sdnParams["deviation_no_tdn"].get<double>(sdnDeviation_);
		sdnStrengthNoTdn_ = sdnParams["strength_no_tdn"].get<double>(sdnStrength_);
		sdnTdnBackoff_ = sdnParams["backoff"].get<double>(0.75);
	}

	cdnEnable_ = params.contains("cdn");
	if (cdnEnable_) {
		auto &cdnParams = params["cdn"];
		cdnDeviation_ = cdnParams["deviation"].get<double>(120);
		cdnStrength_ = cdnParams["strength"].get<double>(0.2);
	}

	tdnEnable_ = params.contains("tdn");
	if (tdnEnable_) {
		auto &tdnParams = params["tdn"];
		tdnDeviation_ = tdnParams["deviation"].get<double>(0.5);
		tdnThreshold_ = tdnParams["threshold"].get<double>(0.75);
	} else if (sdnEnable_) {
		/*
		 * If SDN is enabled but TDN isn't, overwrite all the SDN settings
		 * with the "no TDN" versions. This makes it easier to enable or
		 * disable TDN in the tuning file without editing all the other
		 * parameters.
		 */
		sdnDeviation_ = sdnDeviation2_ = sdnDeviationNoTdn_;
		sdnStrength_ = sdnStrengthNoTdn_;
	}

	return 0;
}

void Denoise::initialise()
{
}

void Denoise::switchMode([[maybe_unused]] CameraMode const &cameraMode,
			 [[maybe_unused]] Metadata *metadata)
{
	/* A mode switch effectively resets temporal denoise and it has to start over. */
	currentSdnDeviation_ = sdnDeviationNoTdn_;
	currentSdnStrength_ = sdnStrengthNoTdn_;
	currentSdnDeviation2_ = sdnDeviationNoTdn_;
}

void Denoise::prepare(Metadata *imageMetadata)
{
	struct NoiseStatus noiseStatus = {};
	noiseStatus.noiseSlope = 3.0; // in case no metadata
	if (imageMetadata->get("noise.status", noiseStatus) != 0)
		LOG(RPiDenoise, Warning) << "no noise profile found";

	LOG(RPiDenoise, Debug)
		<< "Noise profile: constant " << noiseStatus.noiseConstant
		<< " slope " << noiseStatus.noiseSlope;

	if (mode_ == DenoiseMode::Off)
		return;

	if (sdnEnable_) {
		struct SdnStatus sdn;
		sdn.noiseConstant = noiseStatus.noiseConstant * currentSdnDeviation_;
		sdn.noiseSlope = noiseStatus.noiseSlope * currentSdnDeviation_;
		sdn.noiseConstant2 = noiseStatus.noiseConstant * sdnDeviation2_;
		sdn.noiseSlope2 = noiseStatus.noiseSlope * currentSdnDeviation2_;
		sdn.strength = currentSdnStrength_;
		imageMetadata->set("sdn.status", sdn);
		LOG(RPiDenoise, Debug)
			<< "const " << sdn.noiseConstant
			<< " slope " << sdn.noiseSlope
			<< " str " << sdn.strength
			<< " const2 " << sdn.noiseConstant2
			<< " slope2 " << sdn.noiseSlope2;

		/* For the next frame, we back off the SDN parameters as TDN ramps up. */
		double f = sdnTdnBackoff_;
		currentSdnDeviation_ = f * currentSdnDeviation_ + (1 - f) * sdnDeviation_;
		currentSdnStrength_ = f * currentSdnStrength_ + (1 - f) * sdnStrength_;
		currentSdnDeviation2_ = f * currentSdnDeviation2_ + (1 - f) * sdnDeviation2_;
	}

	if (tdnEnable_) {
		struct TdnStatus tdn;
		tdn.noiseConstant = noiseStatus.noiseConstant * tdnDeviation_;
		tdn.noiseSlope = noiseStatus.noiseSlope * tdnDeviation_;
		tdn.threshold = tdnThreshold_;
		imageMetadata->set("tdn.status", tdn);
		LOG(RPiDenoise, Debug)
			<< "programmed tdn threshold " << tdn.threshold
			<< " constant " << tdn.noiseConstant
			<< " slope " << tdn.noiseSlope;
	}

	if (cdnEnable_ && mode_ != DenoiseMode::ColourOff) {
		struct CdnStatus cdn;
		cdn.threshold = cdnDeviation_ * noiseStatus.noiseSlope + noiseStatus.noiseConstant;
		cdn.strength = cdnStrength_;
		imageMetadata->set("cdn.status", cdn);
		LOG(RPiDenoise, Debug)
			<< "programmed cdn threshold " << cdn.threshold
			<< " strength " << cdn.strength;
	}
}

void Denoise::setMode(DenoiseMode mode)
{
	// We only distinguish between off and all other modes.
	mode_ = mode;
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Denoise(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
