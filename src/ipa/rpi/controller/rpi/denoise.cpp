/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * Denoise (spatial, colour, temporal) control algorithm
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

int DenoiseConfig::read(const libcamera::YamlObject &params)
{
	sdnEnable = params.contains("sdn");
	if (sdnEnable) {
		auto &sdnParams = params["sdn"];
		sdnDeviation = sdnParams["deviation"].get<double>(3.2);
		sdnStrength = sdnParams["strength"].get<double>(0.25);
		sdnDeviation2 = sdnParams["deviation2"].get<double>(sdnDeviation);
		sdnDeviationNoTdn = sdnParams["deviation_no_tdn"].get<double>(sdnDeviation);
		sdnStrengthNoTdn = sdnParams["strength_no_tdn"].get<double>(sdnStrength);
		sdnTdnBackoff = sdnParams["backoff"].get<double>(0.75);
	}

	cdnEnable = params.contains("cdn");
	if (cdnEnable) {
		auto &cdnParams = params["cdn"];
		cdnDeviation = cdnParams["deviation"].get<double>(120);
		cdnStrength = cdnParams["strength"].get<double>(0.2);
	}

	tdnEnable = params.contains("tdn");
	if (tdnEnable) {
		auto &tdnParams = params["tdn"];
		tdnDeviation = tdnParams["deviation"].get<double>(0.5);
		tdnThreshold = tdnParams["threshold"].get<double>(0.75);
	} else if (sdnEnable) {
		/*
		 * If SDN is enabled but TDN isn't, overwrite all the SDN settings
		 * with the "no TDN" versions. This makes it easier to enable or
		 * disable TDN in the tuning file without editing all the other
		 * parameters.
		 */
		sdnDeviation = sdnDeviation2 = sdnDeviationNoTdn;
		sdnStrength = sdnStrengthNoTdn;
	}

	return 0;
}

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
	if (!params.contains("normal")) {
		configs_["normal"].read(params);
		currentConfig_ = &configs_["normal"];

		return 0;
	}

	for (const auto &[key, value] : params.asDict()) {
		if (configs_[key].read(value)) {
			LOG(RPiDenoise, Error) << "Failed to read denoise config " << key;
			return -EINVAL;
		}
	}

	auto it = configs_.find("normal");
	if (it == configs_.end()) {
		LOG(RPiDenoise, Error) << "No normal denoise settings found";
		return -EINVAL;
	}
	currentConfig_ = &it->second;

	return 0;
}

void Denoise::initialise()
{
}

void Denoise::switchMode([[maybe_unused]] CameraMode const &cameraMode,
			 [[maybe_unused]] Metadata *metadata)
{
	/* A mode switch effectively resets temporal denoise and it has to start over. */
	currentSdnDeviation_ = currentConfig_->sdnDeviationNoTdn;
	currentSdnStrength_ = currentConfig_->sdnStrengthNoTdn;
	currentSdnDeviation2_ = currentConfig_->sdnDeviationNoTdn;
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

	if (currentConfig_->sdnEnable) {
		struct SdnStatus sdn;
		sdn.noiseConstant = noiseStatus.noiseConstant * currentSdnDeviation_;
		sdn.noiseSlope = noiseStatus.noiseSlope * currentSdnDeviation_;
		sdn.noiseConstant2 = noiseStatus.noiseConstant * currentConfig_->sdnDeviation2;
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
		double f = currentConfig_->sdnTdnBackoff;
		currentSdnDeviation_ = f * currentSdnDeviation_ + (1 - f) * currentConfig_->sdnDeviation;
		currentSdnStrength_ = f * currentSdnStrength_ + (1 - f) * currentConfig_->sdnStrength;
		currentSdnDeviation2_ = f * currentSdnDeviation2_ + (1 - f) * currentConfig_->sdnDeviation2;
	}

	if (currentConfig_->tdnEnable) {
		struct TdnStatus tdn;
		tdn.noiseConstant = noiseStatus.noiseConstant * currentConfig_->tdnDeviation;
		tdn.noiseSlope = noiseStatus.noiseSlope * currentConfig_->tdnDeviation;
		tdn.threshold = currentConfig_->tdnThreshold;
		imageMetadata->set("tdn.status", tdn);
		LOG(RPiDenoise, Debug)
			<< "programmed tdn threshold " << tdn.threshold
			<< " constant " << tdn.noiseConstant
			<< " slope " << tdn.noiseSlope;
	}

	if (currentConfig_->cdnEnable && mode_ != DenoiseMode::ColourOff) {
		struct CdnStatus cdn;
		cdn.threshold = currentConfig_->cdnDeviation * noiseStatus.noiseSlope + noiseStatus.noiseConstant;
		cdn.strength = currentConfig_->cdnStrength;
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

void Denoise::setConfig(std::string const &name)
{
	auto it = configs_.find(name);
	if (it == configs_.end()) {
		/*
		 * Some platforms may have no need for different denoise settings, so we only issue
		 * a warning if there clearly are several configurations.
		 */
		if (configs_.size() > 1)
			LOG(RPiDenoise, Warning) << "No denoise config found for " << name;
		else
			LOG(RPiDenoise, Debug) << "No denoise config found for " << name;
	} else
		currentConfig_ = &it->second;
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Denoise(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
