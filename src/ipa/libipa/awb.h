/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * Generic AWB algorithms
 */

#pragma once

#include <map>
#include <optional>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "libcamera/internal/vector.h"
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace ipa {

struct AwbResult {
	RGB<double> gains;
	double colourTemperature;
};

struct AwbStats {
	virtual double computeColourError(const RGB<double> &gains) const = 0;
	virtual RGB<double> rgbMeans() const = 0;

protected:
	~AwbStats() = default;
};

class AwbAlgorithm
{
public:
	virtual ~AwbAlgorithm() = default;

	virtual int init(const YamlObject &tuningData) = 0;
	virtual AwbResult calculateAwb(const AwbStats &stats, unsigned int lux) = 0;
	virtual std::optional<RGB<double>> gainsFromColourTemperature(double colourTemperature) = 0;

	const ControlInfoMap::Map &controls() const
	{
		return controls_;
	}

	virtual void handleControls([[maybe_unused]] const ControlList &controls) {}

protected:
	int parseModeConfigs(const YamlObject &tuningData,
			     const ControlValue &def = {});

	struct ModeConfig {
		double ctHi;
		double ctLo;
	};

	ControlInfoMap::Map controls_;
	std::map<controls::AwbModeEnum, AwbAlgorithm::ModeConfig> modes_;
};

} /* namespace ipa */

} /* namespace libcamera */
