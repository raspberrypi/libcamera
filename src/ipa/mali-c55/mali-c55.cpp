/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Ideas on Board Oy
 *
 * mali-c55.cpp - Mali-C55 ISP image processing algorithms
 */

#include <map>
#include <string.h>
#include <vector>

#include <linux/mali-c55-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/mali-c55_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAMaliC55)

using namespace std::literals::chrono_literals;

namespace ipa::mali_c55 {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPAMaliC55 : public IPAMaliC55Interface, public Module
{
public:
	IPAMaliC55();

	int init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;
	int configure(const IPAConfigInfo &ipaConfig, uint8_t bayerOrder,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers, bool readOnly) override;
	void unmapBuffers(const std::vector<IPABuffer> &buffers) override;
	void queueRequest(const uint32_t request, const ControlList &controls) override;
	void fillParams(unsigned int request, uint32_t bufferId) override;
	void processStats(unsigned int request, unsigned int bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateSessionConfiguration(const IPACameraSensorInfo &info,
					const ControlInfoMap &sensorControls,
					BayerFormat::Order bayerOrder);
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void setControls();

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap sensorControls_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

} /* namespace */

IPAMaliC55::IPAMaliC55()
	: context_(kMaxFrameContexts)
{
}

std::string IPAMaliC55::logPrefix() const
{
	return "mali-c55";
}

int IPAMaliC55::init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		     ControlInfoMap *ipaControls)
{
	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(IPAMaliC55, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPAMaliC55, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	if (!data->contains("algorithms")) {
		LOG(IPAMaliC55, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	updateControls(ipaConfig.sensorInfo, ipaConfig.sensorControls, ipaControls);

	return 0;
}

void IPAMaliC55::setControls()
{
	IPAActiveState &activeState = context_.activeState;
	uint32_t exposure;
	uint32_t gain;

	if (activeState.agc.autoEnabled) {
		exposure = activeState.agc.automatic.exposure;
		gain = camHelper_->gainCode(activeState.agc.automatic.sensorGain);
	} else {
		exposure = activeState.agc.manual.exposure;
		gain = camHelper_->gainCode(activeState.agc.manual.sensorGain);
	}

	ControlList ctrls(sensorControls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));

	setSensorControls.emit(ctrls);
}

int IPAMaliC55::start()
{
	return 0;
}

void IPAMaliC55::stop()
{
	context_.frameContexts.clear();
}

void IPAMaliC55::updateSessionConfiguration(const IPACameraSensorInfo &info,
					    const ControlInfoMap &sensorControls,
					    BayerFormat::Order bayerOrder)
{
	context_.configuration.sensor.bayerOrder = bayerOrder;

	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>();
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>();
	int32_t defExposure = v4l2Exposure.def().get<int32_t>();

	const ControlInfo &v4l2Gain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	int32_t minGain = v4l2Gain.min().get<int32_t>();
	int32_t maxGain = v4l2Gain.max().get<int32_t>();

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for shutter speed and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum shutter speed
	 */
	context_.configuration.sensor.lineDuration = info.minLineLength * 1.0s / info.pixelRate;
	context_.configuration.agc.minShutterSpeed = minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.maxShutterSpeed = maxExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.defaultExposure = defExposure;
	context_.configuration.agc.minAnalogueGain = camHelper_->gain(minGain);
	context_.configuration.agc.maxAnalogueGain = camHelper_->gain(maxGain);

	if (camHelper_->blackLevel().has_value()) {
		/*
		 * The black level from CameraSensorHelper is a 16-bit value.
		 * The Mali-C55 ISP expects 20-bit settings, so we shift it to
		 * the appropriate width
		 */
		context_.configuration.sensor.blackLevel =
			camHelper_->blackLevel().value() << 4;
	}
}

void IPAMaliC55::updateControls(const IPACameraSensorInfo &sensorInfo,
				const ControlInfoMap &sensorControls,
				ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap;

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

	ctrlMap[&controls::FrameDurationLimits] = ControlInfo(frameDurations[0],
							      frameDurations[1],
							      frameDurations[2]);

	/*
	 * Compute exposure time limits from the V4L2_CID_EXPOSURE control
	 * limits and the line duration.
	 */
	double lineDuration = sensorInfo.minLineLength / sensorInfo.pixelRate;

	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration;
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration;
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration;
	ctrlMap[&controls::ExposureTime] = ControlInfo(minExposure, maxExposure, defExposure);

	/* Compute the analogue gain limits. */
	const ControlInfo &v4l2Gain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	float minGain = camHelper_->gain(v4l2Gain.min().get<int32_t>());
	float maxGain = camHelper_->gain(v4l2Gain.max().get<int32_t>());
	float defGain = camHelper_->gain(v4l2Gain.def().get<int32_t>());
	ctrlMap[&controls::AnalogueGain] = ControlInfo(minGain, maxGain, defGain);

	/*
	 * Merge in any controls that we support either statically or from the
	 * algorithms.
	 */
	ctrlMap.merge(context_.ctrlMap);

	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

int IPAMaliC55::configure(const IPAConfigInfo &ipaConfig, uint8_t bayerOrder,
			  ControlInfoMap *ipaControls)
{
	sensorControls_ = ipaConfig.sensorControls;

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;

	updateSessionConfiguration(info, ipaConfig.sensorControls,
				   static_cast<BayerFormat::Order>(bayerOrder));
	updateControls(info, ipaConfig.sensorControls, ipaControls);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	return 0;
}

void IPAMaliC55::mapBuffers(const std::vector<IPABuffer> &buffers, bool readOnly)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(
			buffer.id,
			MappedFrameBuffer(
				&fb,
				readOnly ? MappedFrameBuffer::MapFlag::Read
					 : MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

void IPAMaliC55::unmapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto it = buffers_.find(buffer.id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(buffer.id);
	}
}

void IPAMaliC55::queueRequest(const uint32_t request, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(request);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->queueRequest(context_, request, frameContext, controls);
	}
}

void IPAMaliC55::fillParams(unsigned int request,
			    [[maybe_unused]] uint32_t bufferId)
{
	struct mali_c55_params_buffer *params;
	IPAFrameContext &frameContext = context_.frameContexts.get(request);

	params = reinterpret_cast<mali_c55_params_buffer *>(
		buffers_.at(bufferId).planes()[0].data());
	memset(params, 0, sizeof(mali_c55_params_buffer));

	params->version = MALI_C55_PARAM_BUFFER_V1;

	for (auto const &algo : algorithms()) {
		algo->prepare(context_, request, frameContext, params);

		ASSERT(params->total_size <= MALI_C55_PARAMS_MAX_SIZE);
	}

	paramsComputed.emit(request);
}

void IPAMaliC55::processStats(unsigned int request, unsigned int bufferId,
			      const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(request);
	const mali_c55_stats_buffer *stats = nullptr;

	stats = reinterpret_cast<mali_c55_stats_buffer *>(
		buffers_.at(bufferId).planes()[0].data());

	frameContext.agc.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	frameContext.agc.sensorGain =
		camHelper_->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

	ControlList metadata(controls::controls);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->process(context_, request, frameContext, stats, metadata);
	}

	setControls();

	statsProcessed.emit(request, metadata);
}

} /* namespace ipa::mali_c55 */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"mali-c55",
	"mali-c55",
};

IPAInterface *ipaCreate()
{
	return new ipa::mali_c55::IPAMaliC55();
}

} /* extern "C" */

} /* namespace libcamera */
