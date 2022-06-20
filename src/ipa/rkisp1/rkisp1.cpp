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

#include "algorithms/agc.h"
#include "algorithms/algorithm.h"
#include "algorithms/awb.h"
#include "algorithms/blc.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPARkISP1)

using namespace std::literals::chrono_literals;

namespace ipa::rkisp1 {

class IPARkISP1 : public IPARkISP1Interface, public Module
{
public:
	int init(const IPASettings &settings, unsigned int hwRevision) override;
	int start() override;
	void stop() override {}

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
	void prepareMetadata(unsigned int frame, unsigned int aeState);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	bool autoExposure_;

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

std::string IPARkISP1::logPrefix() const
{
	return "rkisp1";
}

int IPARkISP1::init(const IPASettings &settings, unsigned int hwRevision)
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

	camHelper_ = CameraSensorHelperFactory::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(IPARkISP1, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	/* Load the tuning data file. */
	File file(settings.configurationFile.c_str());
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

	return createAlgorithms(context_, (*data)["algorithms"]);
}

int IPARkISP1::start()
{
	setControls(0);

	return 0;
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

	autoExposure_ = true;

	int32_t minExposure = itExp->second.min().get<int32_t>();
	int32_t maxExposure = itExp->second.max().get<int32_t>();

	int32_t minGain = itGain->second.min().get<int32_t>();
	int32_t maxGain = itGain->second.max().get<int32_t>();

	LOG(IPARkISP1, Info)
		<< "Exposure: " << minExposure << "-" << maxExposure
		<< " Gain: " << minGain << "-" << maxGain;

	/* Clean context at configuration */
	context_ = {};

	/* Set the hardware revision for the algorithms. */
	context_.configuration.hw.revision = hwRevision_;

	context_.configuration.sensor.lineDuration = info.lineLength * 1.0s / info.pixelRate;

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for shutter speed and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum shutter speed
	 */
	context_.configuration.agc.minShutterSpeed = minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.maxShutterSpeed = maxExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.minAnalogueGain = camHelper_->gain(minGain);
	context_.configuration.agc.maxAnalogueGain = camHelper_->gain(maxGain);

	context_.frameContext.frameCount = 0;

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

void IPARkISP1::queueRequest([[maybe_unused]] const uint32_t frame,
			     [[maybe_unused]] const ControlList &controls)
{
	/* \todo Start processing for 'frame' based on 'controls'. */
}

void IPARkISP1::fillParamsBuffer(const uint32_t frame, const uint32_t bufferId)
{
	rkisp1_params_cfg *params =
		reinterpret_cast<rkisp1_params_cfg *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	for (auto const &algo : algorithms())
		algo->prepare(context_, params);

	paramsBufferReady.emit(frame);
	context_.frameContext.frameCount++;
}

void IPARkISP1::processStatsBuffer(const uint32_t frame, const uint32_t bufferId,
				   const ControlList &sensorControls)
{
	const rkisp1_stat_buffer *stats =
		reinterpret_cast<rkisp1_stat_buffer *>(
			mappedBuffers_.at(bufferId).planes()[0].data());

	context_.frameContext.sensor.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	context_.frameContext.sensor.gain =
		camHelper_->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

	unsigned int aeState = 0;

	for (auto const &algo : algorithms())
		algo->process(context_, nullptr, stats);

	setControls(frame);

	prepareMetadata(frame, aeState);
}

void IPARkISP1::setControls(unsigned int frame)
{
	uint32_t exposure = context_.frameContext.agc.exposure;
	uint32_t gain = camHelper_->gainCode(context_.frameContext.agc.gain);

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));

	setSensorControls.emit(frame, ctrls);
}

void IPARkISP1::prepareMetadata(unsigned int frame, unsigned int aeState)
{
	ControlList ctrls(controls::controls);

	if (aeState)
		ctrls.set(controls::AeLocked, aeState == 2);

	metadataReady.emit(frame, ctrls);
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
