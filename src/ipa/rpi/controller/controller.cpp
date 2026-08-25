/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ISP controller
 */

#include <assert.h>
#include <stdlib.h>	/* strtod */

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>	/* secure_getenv */

#include "libcamera/internal/yaml_parser.h"

#include "algorithm.h"
#include "controller.h"

using namespace RPiController;
using namespace libcamera;
using namespace std::literals::chrono_literals;

LOG_DEFINE_CATEGORY(RPiController)

static const std::map<std::string, Controller::HardwareConfig> HardwareConfigMap = {
	{
		"bcm2835",
		{
			/*
			 * There are only ever 15 AGC regions computed by the firmware
			 * due to zoning, but the HW defines AGC_REGIONS == 16!
			 */
			.agcRegions = { 15 , 1 },
			.agcZoneWeights = { 15 , 1 },
			.awbRegions = { 16, 12 },
			.cacRegions = { 0, 0 },
			.focusRegions = { 4, 3 },
			.numHistogramBins = 128,
			.numGammaPoints = 33,
			.pipelineWidth = 13,
			.statsInline = false,
			.minPixelProcessingTime = 0s,
			.dataBufferStrided = true,
		}
	},
	{
		"pisp",
		{
			.agcRegions = { 0, 0 },
			.agcZoneWeights = { 15, 15 },
			.awbRegions = { 32, 32 },
			.cacRegions = { 8, 8 },
			.focusRegions = { 8, 8 },
			.numHistogramBins = 1024,
			.numGammaPoints = 64,
			.pipelineWidth = 16,
			.statsInline = true,

			/*
			 * The constraint below is on the rate of pixels going
			 * from CSI2 peripheral to ISP-FE (400Mpix/s, plus tiny
			 * overheads per scanline, for which 380Mpix/s is a
			 * conservative bound).
			 *
			 * There is a 64kbit data FIFO before the bottleneck,
			 * which means that in all reasonable cases the
			 * constraint applies at a timescale >= 1 scanline, so
			 * adding horizontal blanking can prevent loss.
			 *
			 * If the backlog were to grow beyond 64kbit during a
			 * single scanline, there could still be loss. This
			 * could happen using 4 lanes at 1.5Gbps at 10bpp with
			 * frames wider than ~16,000 pixels.
			 *
			 * This is the stock-clock value and only the default:
			 * the bottleneck scales with the RP1 clock, which the
			 * rp1-overclock overlay raises. getHardwareConfig()
			 * replaces it with whatever the caller passed in via
			 * LIBCAMERA_RPI_MAX_PIXEL_RATE. See maxPixelRate().
			 */
			.minPixelProcessingTime = 1.0us / 380,
			.dataBufferStrided = false,
		}
	},
};

namespace {

/*
 * The PiSP CSI2-to-ISP-FE bottleneck scales with the RP1 clock, which Will
 * Whang's rp1-overclock overlay raises. The rate is supplied explicitly rather
 * than probed, because probing was tried and does not work on this hardware:
 *
 *   - /proc/device-tree has no rp1 node at all on a CM5 running 6.12.93, so
 *     there is no assigned-clock-rates array to read. The walk always failed
 *     and always fell back to the stock rate, silently discarding the
 *     overclock the operator had enabled.
 *   - The overlay asks for 300MHz and the clock driver delivers 333.33MHz
 *     (measured: pll_sys and clk_sys both 333333333Hz). Any table keyed on the
 *     requested 300MHz misses, so even a working read would have fallen back.
 *
 * cinepi-raw passes the rate down instead: one explicit value, set from the
 * same settings switch that enables the overlay, so the two cannot disagree.
 *
 * The default remains the stock bound. Under-stating it costs frame rate and
 * nothing else, because the IPA simply pads the line length further.
 * Over-stating it corrupts every mode wide enough for this bound to be what
 * limits the line time, and does so silently: the only warning on this path
 * (ipa_base.cpp, "THIS WILL CAUSE IMAGE CORRUPTION") fires when the *sensor*
 * cannot supply enough blanking to meet the bound, which a too-high rate makes
 * less likely to trigger, not more.
 */
constexpr double pixelRateStock = 380.0;

constexpr const char *maxPixelRateEnv = "LIBCAMERA_RPI_MAX_PIXEL_RATE";

double maxPixelRate()
{
	const char *env = utils::secure_getenv(maxPixelRateEnv);
	if (!env || !*env) {
		LOG(RPiController, Info)
			<< maxPixelRateEnv << " not set, assuming stock clocks ("
			<< pixelRateStock << "MPix/s)";
		return pixelRateStock;
	}

	char *end;
	double rate = strtod(env, &end);
	if (*end != '\0' || !(rate > 0.0)) {
		LOG(RPiController, Warning)
			<< "Ignoring malformed " << maxPixelRateEnv << " \"" << env
			<< "\", assuming stock clocks (" << pixelRateStock << "MPix/s)";
		return pixelRateStock;
	}

	LOG(RPiController, Info)
		<< "Pixel rate limited to " << rate << "MPix/s (" << maxPixelRateEnv << ")";

	return rate;
}

} /* namespace */

Controller::Controller()
	: switchModeCalled_(false)
{
}

Controller::~Controller() {}

int Controller::read(char const *filename)
{
	File file(filename);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPiController, Warning)
			<< "Failed to open tuning file '" << filename << "'";
		return -EINVAL;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	if (!root)
		return -EINVAL;

	double version = (*root)["version"].get<double>(1.0);
	target_ = (*root)["target"].get<std::string>("bcm2835");

	if (version < 2.0) {
		LOG(RPiController, Warning)
			<< "This format of the tuning file will be deprecated soon!"
			<< " Please use the convert_tuning.py utility to update to version 2.0.";

		for (auto const &[key, value] : root->asDict()) {
			int ret = createAlgorithm(key, value);
			if (ret)
				return ret;
		}
	} else if (version < 3.0) {
		if (!root->contains("algorithms")) {
			LOG(RPiController, Error)
				<< "Tuning file " << filename
				<< " does not have an \"algorithms\" list!";
			return -EINVAL;
		}

		for (auto const &rootAlgo : (*root)["algorithms"].asList())
			for (auto const &[key, value] : rootAlgo.asDict()) {
				int ret = createAlgorithm(key, value);
				if (ret)
					return ret;
			}
	} else {
		LOG(RPiController, Error)
			<< "Unrecognised version " << version
			<< " for the tuning file " << filename;
		return -EINVAL;
	}

	return 0;
}

int Controller::createAlgorithm(const std::string &name, const YamlObject &params)
{
	auto it = getAlgorithms().find(name);
	if (it == getAlgorithms().end()) {
		LOG(RPiController, Warning)
			<< "No algorithm found for \"" << name << "\"";
		return 0;
	}

	Algorithm *algo = (*it->second)(this);
	int ret = algo->read(params);
	if (ret)
		return ret;

	algorithms_.push_back(AlgorithmPtr(algo));
	return 0;
}

void Controller::initialise()
{
	for (auto &algo : algorithms_)
		algo->initialise();
}

void Controller::switchMode(CameraMode const &cameraMode, Metadata *metadata)
{
	for (auto &algo : algorithms_)
		algo->switchMode(cameraMode, metadata);
	switchModeCalled_ = true;
}

void Controller::prepare(Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		algo->prepare(imageMetadata);
}

void Controller::process(StatisticsPtr stats, Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		algo->process(stats, imageMetadata);
}

Metadata &Controller::getGlobalMetadata()
{
	return globalMetadata_;
}

Algorithm *Controller::getAlgorithm(std::string const &name) const
{
	/*
	 * The passed name must be the entire algorithm name, or must match the
	 * last part of it with a period (.) just before.
	 */
	size_t nameLen = name.length();
	for (auto &algo : algorithms_) {
		char const *algoName = algo->name();
		size_t algoNameLen = strlen(algoName);
		if (algoNameLen >= nameLen &&
		    strcasecmp(name.c_str(),
			       algoName + algoNameLen - nameLen) == 0 &&
		    (nameLen == algoNameLen ||
		     algoName[algoNameLen - nameLen - 1] == '.'))
			return algo.get();
	}
	return nullptr;
}

const std::string &Controller::getTarget() const
{
	return target_;
}

const Controller::HardwareConfig &Controller::getHardwareConfig() const
{
	if (hardwareConfig_)
		return *hardwareConfig_;

	auto cfg = HardwareConfigMap.find(getTarget());

	/*
	 * This really should not happen, the IPA ought to validate the target
	 * on initialisation.
	 */
	ASSERT(cfg != HardwareConfigMap.end());

	HardwareConfig config = cfg->second;

	/*
	 * Only PiSP carries a pixel-rate bound, and only PiSP has an RP1 whose
	 * clock it tracks. The vc4 platforms leave it zero (unconstrained), and
	 * probing the device tree for them would be meaningless.
	 */
	if (config.minPixelProcessingTime)
		config.minPixelProcessingTime = 1.0us / maxPixelRate();

	hardwareConfig_ = config;

	return *hardwareConfig_;
}
