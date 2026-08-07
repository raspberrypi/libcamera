/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Ideas on Board Oy
 *
 * libIPA AWB algorithms
 */

#pragma once

#include <array>
#include <map>
#include <optional>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

namespace ipa {

namespace awb {

struct Context {
	RGB<double> gains;
	unsigned int colourTemperature;
};

struct ActiveState {
	Context manual;
	Context automatic;

	bool autoEnabled;
};

struct FrameContext : public Context {
	bool autoEnabled;
};

} /* namespace awb */

struct AwbStats {
	virtual double computeColourError(const RGB<double> &gains) const = 0;
	virtual bool valid() const = 0;
	virtual RGB<double> rgbMeans() const = 0;

protected:
	~AwbStats() = default;
};

class AwbImplementation
{
public:
	struct Result {
		RGB<double> gains;
		unsigned int colourTemperature;
	};

	virtual ~AwbImplementation() = default;
	virtual int init(const ValueNode &tuningData) = 0;
	virtual Result calculateAwb(const AwbStats &stats, unsigned int lux,
				    std::array<double, 2> range) = 0;
	virtual std::optional<RGB<double>>
	gainsFromColourTemperature(double colourTemperature) = 0;
};

class AwbAlgorithmBase
{
public:
	int init(const ValueNode &tuningData);

	int configure(awb::ActiveState &state);

	void queueRequest(awb::ActiveState &state,
			  const uint32_t frame,
			  awb::FrameContext &frameContext,
			  const ControlList &controls);

	void prepare(awb::ActiveState &state, awb::FrameContext &frameContext);

	void process(awb::ActiveState &state, awb::FrameContext &frameContext,
		     const AwbStats &stats, unsigned int lux,
		     ControlList &metadata);

protected:
	AwbAlgorithmBase() = default;

	ControlInfoMap::Map controls_;
	float gainMin_;
	float gainMax_;

private:
	struct ModeConfig {
		double ctHi;
		double ctLo;
	};

	/* AwbGrey does not support modes; */
	static constexpr ModeConfig AwbGreyMode = { 0.0, 0.0 };

	int parseModeConfigs(const ValueNode &tuningData,
			     const ControlValue &def = {});

	std::map<controls::AwbModeEnum, AwbAlgorithmBase::ModeConfig> modes_;
	const ModeConfig *currentMode_ = nullptr;
	std::unique_ptr<AwbImplementation> impl_;
	bool bayes_ = false;
};

template<typename Q>
class AwbAlgorithm : public AwbAlgorithmBase
{
public:
	int init(const ValueNode &tuningData, ControlInfoMap::Map &controls)
	{
		AwbAlgorithmBase::init(tuningData);

		gainMin_ = std::max(Q::TraitsType::min, 1.0f);
		gainMax_ = Q::TraitsType::max;

		controls_[&controls::ColourGains] =
			ControlInfo(gainMin_, gainMax_,
				    Span<const float, 2>{ { 1.0f, 1.0f } });

		controls.insert(controls_.begin(), controls_.end());

		return 0;
	}
};

} /* namespace ipa */

} /* namespace libcamera */
