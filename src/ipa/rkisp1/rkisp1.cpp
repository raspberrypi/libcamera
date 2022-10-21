/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * rkisp1.cpp - RkISP1 Image Processing Algorithms
 */

#include <algorithm>
#include <math.h>
#include <queue>
#include <stdint.h>
#include <string.h>

#include <linux/rkisp1-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>
#include <libcamera/request.h>

#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

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
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;

	int configure(const IPACameraSensorInfo &info,
		      const std::map<uint32_t, IPAStream> &streamConfig,
		      const std::map<uint32_t, ControlInfoMap> &entityControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void fillParamsBuffer(const uint32_t frame, const uint32_t bufferId) override;
	void processStatsBuffer(const uint32_t frame, const uint32_t bufferId,
				const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void setControls(unsigned int frame);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap ctrls_;

	/* revision-specific data */
	rkisp1_cif_isp_version hwRevision_;
	unsigned int hwHistBinNMax_;
	unsigned int hwGammaOutMaxSamples_;
	unsigned int hwHistogramWeightGridsSize_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

/* List of controls handled by the RkISP1 IPA */
const ControlInfoMap::Map rkisp1Controls{
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::ColourGains, ControlInfo(0.0f, 3.996f, 1.0f) },
	{ &controls::Brightness, ControlInfo(-1.0f, 0.993f) },
	{ &controls::Contrast, ControlInfo(0.0f, 1.993f) },
	{ &controls::Saturation, ControlInfo(0.0f, 1.993f) },
	{ &controls::Sharpness, ControlInfo(0.0f, 10.0f, 1.0f) },
	{ &controls::draft::NoiseReductionMode, ControlInfo(controls::draft::NoiseReductionModeValues) },
};

} /* namespace */

IPARkISP1::IPARkISP1()
	: context_({ {}, {}, { kMaxFrameContexts } })
{
}

std::string IPARkISP1::logPrefix() const
{
	return "rkisp1";
}

int IPARkISP1::init(const IPASettings &settings, unsigned int hwRevision,
		    ControlInfoMap *ipaControls)
{
	/* \todo Add support for other revisions */
	switch (hwRevision) {
	case RKISP1_V10:
		hwHistBinNMax_ = RKISP1_CIF_ISP_HIST_BIN_N_MAX_V10;
		hwGammaOutMaxSamples_ = RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V10;
		hwHistogramWeightGridsSize_ = RKISP1_CIF_ISP_HISTOGRAM_WEIGHT_GRIDS_SIZE_V10;
		break;
	case RKISP1_V12:
		hwHistBinNMax_ = RKISP1_CIF_ISP_HIST_BIN_N_MAX_V12;
		hwGammaOutMaxSamples_ = RKISP1_CIF_ISP_GAMMA_OUT_MAX_SAMPLES_V12;
		hwHistogramWeightGridsSize_ = RKISP1_CIF_ISP_HISTOGRAM_WEIGHT_GRIDS_SIZE_V12;
		break;
	default:
		LOG(IPARkISP1, Error)
			<< "Hardware revision " << hwRevision
			<< " is currently not supported";
		return -ENODEV;
	}

	LOG(IPARkISP1, Debug) << "Hardware revision is " << hwRevision;

	/* Cache the value to set it in configure. */
	hwRevision_ = static_cast<rkisp1_cif_isp_version>(hwRevision);

	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(IPARkISP1, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

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

	/* Return the controls handled by the IPA. */
	ControlInfoMap::Map ctrlMap = rkisp1Controls;
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);

	return 0;
}

int IPARkISP1::start()
{
	setControls(0);

	return 0;
}

void IPARkISP1::stop()
{
	context_.frameContexts.clear();
}

/**
 * \todo The RkISP1 pipeline currently provides an empty IPACameraSensorInfo
 * if the connected sensor does not provide enough information to properly
 * assemble one. Make sure the reported sensor information are relevant
 * before accessing them.
 */
int IPARkISP1::configure([[maybe_unused]] const IPACameraSensorInfo &info,
			 [[maybe_unused]] const std::map<uint32_t, IPAStream> &streamConfig,
			 const std::map<uint32_t, ControlInfoMap> &entityControls)
{
	if (entityControls.empty())
		return -EINVAL;

	ctrls_ = entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPARkISP1, Error) << "Can't find exposure control";
		return -EINVAL;
	}

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPARkISP1, Error) << "Can't find gain control";
		return -EINVAL;
	}

	int32_t minExposure = itExp->second.min().get<int32_t>();
	int32_t maxExposure = itExp->second.max().get<int32_t>();

	int32_t minGain = itGain->second.min().get<int32_t>();
	int32_t maxGain = itGain->second.max().get<int32_t>();

	LOG(IPARkISP1, Debug)
		<< "Exposure: [" << minExposure << ", " << maxExposure
		<< "], gain: [" << minGain << ", " << maxGain << "]";

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	/* Set the hardware revision for the algorithms. */
	context_.configuration.hw.revision = hwRevision_;

	const ControlInfo vBlank = ctrls_.find(V4L2_CID_VBLANK)->second;
	context_.configuration.sensor.defVBlank = vBlank.def().get<int32_t>();
	context_.configuration.sensor.size = info.outputSize;
	context_.configuration.sensor.lineDuration = info.minLineLength * 1.0s / info.pixelRate;

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for shutter speed and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum shutter speed
	 */
	context_.configuration.sensor.minShutterSpeed =
		minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.sensor.maxShutterSpeed =
		maxExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.sensor.minAnalogueGain = camHelper_->gain(minGain);
	context_.configuration.sensor.maxAnalogueGain = camHelper_->gain(maxGain);

	for (auto const &algo : algorithms()) {
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

	for (auto const &algo : algorithms())
		algo->queueRequest(context_, frame, frameContext, controls);
}

void IPARkISP1::fillParamsBuffer(const uint32_t frame, const uint32_t bufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	rkisp1_params_cfg *params =
		reinterpret_cast<rkisp1_params_cfg *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	for (auto const &algo : algorithms())
		algo->prepare(context_, frame, frameContext, params);

	paramsBufferReady.emit(frame);
}

void IPARkISP1::processStatsBuffer(const uint32_t frame, const uint32_t bufferId,
				   const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	const rkisp1_stat_buffer *stats =
		reinterpret_cast<rkisp1_stat_buffer *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	frameContext.sensor.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	frameContext.sensor.gain =
		camHelper_->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

	ControlList metadata(controls::controls);

	for (auto const &algo : algorithms())
		algo->process(context_, frame, frameContext, stats, metadata);

	setControls(frame);

	metadataReady.emit(frame, metadata);
}

void IPARkISP1::setControls(unsigned int frame)
{
	/*
	 * \todo The frame number is most likely wrong here, we need to take
	 * internal sensor delays and other timing parameters into account.
	 */

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);
	uint32_t exposure = frameContext.agc.exposure;
	uint32_t gain = camHelper_->gainCode(frameContext.agc.gain);

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));

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
	"PipelineHandlerRkISP1",
	"rkisp1",
};

IPAInterface *ipaCreate()
{
	return new ipa::rkisp1::IPARkISP1();
}
}

} /* namespace libcamera */
