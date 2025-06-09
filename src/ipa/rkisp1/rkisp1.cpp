/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * RkISP1 Image Processing Algorithms
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <stdint.h>
#include <string.h>

#include <linux/rkisp1-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/request.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"

#include "ipa_context.h"
#include "params.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPARkISP1)

using namespace std::literals::chrono_literals;

namespace ipa::rkisp1 {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPARkISP1 : public IPARkISP1Interface, public Module
{
public:
	IPARkISP1();

	int init(const IPASettings &settings, unsigned int hwRevision,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;

	int configure(const IPAConfigInfo &ipaConfig,
		      const std::map<uint32_t, IPAStream> &streamConfig,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void computeParams(const uint32_t frame, const uint32_t bufferId) override;
	void processStats(const uint32_t frame, const uint32_t bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void setControls(unsigned int frame);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap sensorControls_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

const IPAHwSettings ipaHwSettingsV10{
	RKISP1_CIF_ISP_AE_MEAN_MAX_V10,
	RKISP1_CIF_ISP_HIST_BIN_N_MAX_V10,
	RKISP1_CIF_ISP_HISTOGRAM_WEIGHT_GRIDS_SIZE_V10,
	RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10,
	false,
};

const IPAHwSettings ipaHwSettingsIMX8MP{
	RKISP1_CIF_ISP_AE_MEAN_MAX_V10,
	RKISP1_CIF_ISP_HIST_BIN_N_MAX_V10,
	RKISP1_CIF_ISP_HISTOGRAM_WEIGHT_GRIDS_SIZE_V10,
	RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10,
	true,
};

const IPAHwSettings ipaHwSettingsV12{
	RKISP1_CIF_ISP_AE_MEAN_MAX_V12,
	RKISP1_CIF_ISP_HIST_BIN_N_MAX_V12,
	RKISP1_CIF_ISP_HISTOGRAM_WEIGHT_GRIDS_SIZE_V12,
	RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V12,
	false,
};

/* List of controls handled by the RkISP1 IPA */
const ControlInfoMap::Map rkisp1Controls{
	{ &controls::DebugMetadataEnable, ControlInfo(false, true, false) },
	{ &controls::draft::NoiseReductionMode, ControlInfo(controls::draft::NoiseReductionModeValues) },
};

} /* namespace */

IPARkISP1::IPARkISP1()
	: context_(kMaxFrameContexts)
{
}

std::string IPARkISP1::logPrefix() const
{
	return "rkisp1";
}

int IPARkISP1::init(const IPASettings &settings, unsigned int hwRevision,
		    const IPACameraSensorInfo &sensorInfo,
		    const ControlInfoMap &sensorControls,
		    ControlInfoMap *ipaControls)
{
	/* \todo Add support for other revisions */
	switch (hwRevision) {
	case RKISP1_V10:
		context_.hw = &ipaHwSettingsV10;
		break;
	case RKISP1_V_IMX8MP:
		context_.hw = &ipaHwSettingsIMX8MP;
		break;
	case RKISP1_V12:
		context_.hw = &ipaHwSettingsV12;
		break;
	default:
		LOG(IPARkISP1, Error)
			<< "Hardware revision " << hwRevision
			<< " is currently not supported";
		return -ENODEV;
	}

	LOG(IPARkISP1, Debug) << "Hardware revision is " << hwRevision;

	context_.sensorInfo = sensorInfo;

	context_.camHelper = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!context_.camHelper) {
		LOG(IPARkISP1, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	context_.configuration.sensor.lineDuration =
		sensorInfo.minLineLength * 1.0s / sensorInfo.pixelRate;

	/* Load the tuning data file. */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPARkISP1, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	unsigned int version = (*data)["version"].get<uint32_t>(0);
	if (version != 1) {
		LOG(IPARkISP1, Error)
			<< "Invalid tuning file version " << version;
		return -EINVAL;
	}

	if (!data->contains("algorithms")) {
		LOG(IPARkISP1, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	/* Initialize controls. */
	updateControls(sensorInfo, sensorControls, ipaControls);

	return 0;
}

int IPARkISP1::start()
{
	/* \todo Properly handle startup controls. */
	return 0;
}

void IPARkISP1::stop()
{
	context_.frameContexts.clear();
}

int IPARkISP1::configure(const IPAConfigInfo &ipaConfig,
			 const std::map<uint32_t, IPAStream> &streamConfig,
			 ControlInfoMap *ipaControls)
{
	sensorControls_ = ipaConfig.sensorControls;

	const auto itExp = sensorControls_.find(V4L2_CID_EXPOSURE);
	int32_t minExposure = itExp->second.min().get<int32_t>();
	int32_t maxExposure = itExp->second.max().get<int32_t>();

	const auto itGain = sensorControls_.find(V4L2_CID_ANALOGUE_GAIN);
	int32_t minGain = itGain->second.min().get<int32_t>();
	int32_t maxGain = itGain->second.max().get<int32_t>();

	LOG(IPARkISP1, Debug)
		<< "Exposure: [" << minExposure << ", " << maxExposure
		<< "], gain: [" << minGain << ", " << maxGain << "]";

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	context_.configuration.paramFormat = ipaConfig.paramFormat;

	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;
	const ControlInfo vBlank = sensorControls_.find(V4L2_CID_VBLANK)->second;
	context_.configuration.sensor.defVBlank = vBlank.def().get<int32_t>();
	context_.configuration.sensor.size = info.outputSize;
	context_.configuration.sensor.lineDuration = info.minLineLength * 1.0s / info.pixelRate;

	/* Update the camera controls using the new sensor settings. */
	updateControls(info, sensorControls_, ipaControls);

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for exposure time and analogue gain. As it depends
	 * on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum exposure time
	 */
	context_.configuration.sensor.minExposureTime =
		minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.sensor.maxExposureTime =
		maxExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.sensor.minAnalogueGain =
		context_.camHelper->gain(minGain);
	context_.configuration.sensor.maxAnalogueGain =
		context_.camHelper->gain(maxGain);

	context_.configuration.raw = std::any_of(streamConfig.begin(), streamConfig.end(),
		[](auto &cfg) -> bool {
			PixelFormat pixelFormat{ cfg.second.pixelFormat };
			const PixelFormatInfo &format = PixelFormatInfo::info(pixelFormat);
			return format.colourEncoding == PixelFormatInfo::ColourEncodingRAW;
		});

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		/* Disable algorithms that don't support raw formats. */
		algo->disabled_ = context_.configuration.raw && !algo->supportsRaw_;
		if (algo->disabled_)
			continue;

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	return 0;
}

void IPARkISP1::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		MappedFrameBuffer mappedBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite);
		if (!mappedBuffer.isValid()) {
			LOG(IPARkISP1, Fatal) << "Failed to mmap buffer: "
					      << strerror(mappedBuffer.error());
		}

		mappedBuffers_.emplace(buffer.id, std::move(mappedBuffer));
	}
}

void IPARkISP1::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		const auto fb = buffers_.find(id);
		if (fb == buffers_.end())
			continue;

		mappedBuffers_.erase(id);
		buffers_.erase(id);
	}
}

void IPARkISP1::queueRequest(const uint32_t frame, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(frame);
	context_.debugMetadata.enableByControl(controls);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());
		if (algo->disabled_)
			continue;
		algo->queueRequest(context_, frame, frameContext, controls);
	}
}

void IPARkISP1::computeParams(const uint32_t frame, const uint32_t bufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	RkISP1Params params(context_.configuration.paramFormat,
			    mappedBuffers_.at(bufferId).planes()[0]);

	for (auto const &algo : algorithms())
		algo->prepare(context_, frame, frameContext, &params);

	paramsComputed.emit(frame, params.size());
}

void IPARkISP1::processStats(const uint32_t frame, const uint32_t bufferId,
			     const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	/*
	 * In raw capture mode, the ISP is bypassed and no statistics buffer is
	 * provided.
	 */
	const rkisp1_stat_buffer *stats = nullptr;
	if (!context_.configuration.raw)
		stats = reinterpret_cast<rkisp1_stat_buffer *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	frameContext.sensor.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	frameContext.sensor.gain =
		context_.camHelper->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

	ControlList metadata(controls::controls);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());
		if (algo->disabled_)
			continue;
		algo->process(context_, frame, frameContext, stats, metadata);
	}

	setControls(frame);

	context_.debugMetadata.moveEntries(metadata);
	metadataReady.emit(frame, metadata);
}

void IPARkISP1::updateControls(const IPACameraSensorInfo &sensorInfo,
			       const ControlInfoMap &sensorControls,
			       ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap = rkisp1Controls;

	/*
	 * Compute exposure time limits from the V4L2_CID_EXPOSURE control
	 * limits and the line duration.
	 */
	double lineDuration = context_.configuration.sensor.lineDuration.get<std::micro>();
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration;
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration;
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration;
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::ExposureTime),
			std::forward_as_tuple(minExposure, maxExposure, defExposure));

	/* Compute the analogue gain limits. */
	const ControlInfo &v4l2Gain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	float minGain = context_.camHelper->gain(v4l2Gain.min().get<int32_t>());
	float maxGain = context_.camHelper->gain(v4l2Gain.max().get<int32_t>());
	float defGain = context_.camHelper->gain(v4l2Gain.def().get<int32_t>());
	ctrlMap.emplace(std::piecewise_construct,
			std::forward_as_tuple(&controls::AnalogueGain),
			std::forward_as_tuple(minGain, maxGain, defGain));

	/*
	 * Compute the frame duration limits.
	 *
	 * The frame length is computed assuming a fixed line length combined
	 * with the vertical frame sizes.
	 */
	const ControlInfo &v4l2HBlank = sensorControls.find(V4L2_CID_HBLANK)->second;
	uint32_t hblank = v4l2HBlank.def().get<int32_t>();
	uint32_t lineLength = sensorInfo.outputSize.width + hblank;

	const ControlInfo &v4l2VBlank = sensorControls.find(V4L2_CID_VBLANK)->second;
	std::array<uint32_t, 3> frameHeights{
		v4l2VBlank.min().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.max().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.def().get<int32_t>() + sensorInfo.outputSize.height,
	};

	std::array<int64_t, 3> frameDurations;
	for (unsigned int i = 0; i < frameHeights.size(); ++i) {
		uint64_t frameSize = lineLength * frameHeights[i];
		frameDurations[i] = frameSize / (sensorInfo.pixelRate / 1000000U);
	}

	/* \todo Move this (and other agc-related controls) to agc */
	context_.ctrlMap[&controls::FrameDurationLimits] =
		ControlInfo(frameDurations[0], frameDurations[1], frameDurations[2]);

	ctrlMap.insert(context_.ctrlMap.begin(), context_.ctrlMap.end());
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

void IPARkISP1::setControls(unsigned int frame)
{
	/*
	 * \todo The frame number is most likely wrong here, we need to take
	 * internal sensor delays and other timing parameters into account.
	 */

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);
	uint32_t exposure = frameContext.agc.exposure;
	uint32_t gain = context_.camHelper->gainCode(frameContext.agc.gain);
	uint32_t vblank = frameContext.agc.vblank;

	LOG(IPARkISP1, Debug)
		<< "Set controls for frame " << frame << ": exposure " << exposure
		<< ", gain " << frameContext.agc.gain << ", vblank " << vblank;

	ControlList ctrls(sensorControls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));
	ctrls.set(V4L2_CID_VBLANK, static_cast<int32_t>(vblank));

	setSensorControls.emit(frame, ctrls);
}

} /* namespace ipa::rkisp1 */

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"rkisp1",
	"rkisp1",
};

IPAInterface *ipaCreate()
{
	return new ipa::rkisp1::IPARkISP1();
}
}

} /* namespace libcamera */
