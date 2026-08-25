/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ISP controller
 */

#include <assert.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <optional>
#include <string>
#include <vector>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

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
			 * the bottleneck scales with RP1_CLK_SYS, which the
			 * rp1-overclock overlay raises. getHardwareConfig()
			 * replaces it with the rate that matches the clock the
			 * board is actually running. See maxPixelRate() below.
			 */
			.minPixelProcessingTime = 1.0us / 380,
			.dataBufferStrided = false,
		}
	},
};

namespace {

/*
 * The PiSP CSI2-to-ISP-FE bottleneck scales with RP1_CLK_SYS. Will Whang's
 * rp1-overclock overlay raises that clock from 200 to 300MHz, and the pixel
 * rate the pipeline may advertise moves with it.
 *
 * Only these two points have been measured on hardware, so only these two are
 * honoured; an unrecognised clock falls back to the stock rate rather than
 * interpolating. That asymmetry is deliberate. Under-stating the rate costs
 * frame rate and nothing else, because the IPA simply pads the line length
 * further. Over-stating it corrupts every mode wide enough for the bound to be
 * what limits the line time -- and does so silently, since the only warning on
 * this path (ipa_base.cpp, "THIS WILL CAUSE IMAGE CORRUPTION") fires when the
 * *sensor* cannot supply enough blanking to meet the bound, which a too-high
 * rate makes less likely to trigger, not more.
 */
constexpr unsigned int rp1ClkSysStock = 200000000;
constexpr unsigned int rp1ClkSysOverclocked = 300000000;
constexpr double pixelRateStock = 380.0;
constexpr double pixelRateOverclocked = 580.0;

/*
 * RP1_CLK_SYS is index 7 of the rp1_clocks assigned-clock-rates array. The
 * index is only trusted as far as the value it yields: anything that is not
 * one of the two rates above is discarded, so a future kernel that reorders
 * the array degrades to the stock default instead of misreading a neighbour.
 */
constexpr size_t rp1ClkSysIndex = 7;

constexpr const char *deviceTreeRoot = "/proc/device-tree";
constexpr const char *clockRatesProperty = "assigned-clock-rates";
constexpr const char *maxPixelRateEnv = "LIBCAMERA_RPI_MAX_PIXEL_RATE";

/* Device tree properties are big endian regardless of the CPU. */
std::optional<unsigned int> readBeU32(const std::string &filename, size_t index)
{
	File file(filename);
	if (!file.open(File::OpenModeFlag::ReadOnly))
		return {};

	ssize_t size = file.size();
	if (size <= 0 || static_cast<size_t>(size) < (index + 1) * sizeof(uint32_t))
		return {};

	std::vector<uint8_t> buffer(size);
	if (file.read({ buffer.data(), buffer.size() }) != size)
		return {};

	const uint8_t *p = buffer.data() + index * sizeof(uint32_t);
	return (static_cast<unsigned int>(p[0]) << 24) |
	       (static_cast<unsigned int>(p[1]) << 16) |
	       (static_cast<unsigned int>(p[2]) << 8) |
	       static_cast<unsigned int>(p[3]);
}

/*
 * Walk the live device tree for the RP1 clock node's rate array. The node is
 * matched by having an ancestor whose name contains "rp1", rather than by an
 * absolute path, because the unit address moves between kernels. Applied
 * overlays are reflected here, which is the point: this reads what the board
 * booted with, not what it was built for.
 */
std::optional<unsigned int> findRp1ClkSys(const std::string &dir, bool underRp1,
					  unsigned int depth)
{
	if (depth > 8)
		return {};

	DIR *dp = opendir(dir.c_str());
	if (!dp)
		return {};

	std::optional<unsigned int> result;
	const struct dirent *ent;

	while (!result && (ent = readdir(dp)) != nullptr) {
		std::string name = ent->d_name;
		if (name == "." || name == "..")
			continue;

		std::string path = dir + "/" + name;
		bool rp1 = underRp1 || name.find("rp1") != std::string::npos;

		struct stat st;
		if (stat(path.c_str(), &st))
			continue;

		if (S_ISDIR(st.st_mode))
			result = findRp1ClkSys(path, rp1, depth + 1);
		else if (rp1 && name == clockRatesProperty)
			result = readBeU32(path, rp1ClkSysIndex);
	}

	closedir(dp);

	return result;
}

double maxPixelRate()
{
	const char *env = utils::secure_getenv(maxPixelRateEnv);
	if (env && *env) {
		char *end;
		double rate = strtod(env, &end);
		if (*end == '\0' && rate > 0.0) {
			LOG(RPiController, Info)
				<< maxPixelRateEnv << " set, using " << rate
				<< "MPix/s";
			return rate;
		}

		LOG(RPiController, Warning)
			<< "Ignoring malformed " << maxPixelRateEnv << " \""
			<< env << "\"";
	}

	std::optional<unsigned int> clk = findRp1ClkSys(deviceTreeRoot, false, 0);
	if (!clk) {
		LOG(RPiController, Info)
			<< "Could not read RP1_CLK_SYS from the device tree,"
			<< " assuming stock clocks (" << pixelRateStock << "MPix/s)";
		return pixelRateStock;
	}

	switch (*clk) {
	case rp1ClkSysStock:
		LOG(RPiController, Info)
			<< "RP1_CLK_SYS at 200MHz (stock), limiting to "
			<< pixelRateStock << "MPix/s";
		return pixelRateStock;

	case rp1ClkSysOverclocked:
		LOG(RPiController, Info)
			<< "RP1_CLK_SYS at 300MHz (rp1-overclock), allowing "
			<< pixelRateOverclocked << "MPix/s";
		return pixelRateOverclocked;

	default:
		LOG(RPiController, Warning)
			<< "Unrecognised RP1_CLK_SYS rate " << *clk
			<< "Hz, assuming stock clocks (" << pixelRateStock
			<< "MPix/s)";
		return pixelRateStock;
	}
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
