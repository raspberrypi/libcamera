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

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>
#include <libcamera/request.h>

#include <libcamera/internal/mapped_framebuffer.h>

#include "algorithms/agc.h"
#include "algorithms/algorithm.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPARkISP1)

using namespace std::literals::chrono_literals;

namespace ipa::rkisp1 {

class IPARkISP1 : public IPARkISP1Interface
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
	void processEvent(const RkISP1Event &event) override;

private:
	void queueRequest(unsigned int frame, rkisp1_params_cfg *params,
			  const ControlList &controls);
	void updateStatistics(unsigned int frame,
			      const rkisp1_stat_buffer *stats);

	void setControls(unsigned int frame);
	void metadataReady(unsigned int frame, unsigned int aeState);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, MappedFrameBuffer> mappedBuffers_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	bool autoExposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t minGain_;
	uint32_t maxGain_;

	/* revision-specific data */
	rkisp1_cif_isp_version hwRevision_;
	unsigned int hwHistBinNMax_;
	unsigned int hwGammaOutMaxSamples_;
	unsigned int hwHistogramWeightGridsSize_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;

	/* Maintain the algorithms used by the IPA */
	std::list<std::unique_ptr<ipa::rkisp1::Algorithm>> algorithms_;
};

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

	/* Construct our Algorithms */
	algorithms_.push_back(std::make_unique<algorithms::Agc>());

	return 0;
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

	minExposure_ = itExp->second.min().get<int32_t>();
	maxExposure_ = itExp->second.max().get<int32_t>();

	minGain_ = itGain->second.min().get<int32_t>();
	maxGain_ = itGain->second.max().get<int32_t>();

	LOG(IPARkISP1, Info)
		<< "Exposure: " << minExposure_ << "-" << maxExposure_
		<< " Gain: " << minGain_ << "-" << maxGain_;

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
	context_.configuration.agc.minShutterSpeed = minExposure_ * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.maxShutterSpeed = maxExposure_ * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.minAnalogueGain = camHelper_->gain(minGain_);
	context_.configuration.agc.maxAnalogueGain = camHelper_->gain(maxGain_);

	for (auto const &algo : algorithms_) {
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

void IPARkISP1::processEvent(const RkISP1Event &event)
{
	switch (event.op) {
	case EventSignalStatBuffer: {
		unsigned int frame = event.frame;
		unsigned int bufferId = event.bufferId;

		const rkisp1_stat_buffer *stats =
			reinterpret_cast<rkisp1_stat_buffer *>(
				mappedBuffers_.at(bufferId).planes()[0].data());

		context_.frameContext.sensor.exposure =
			event.sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
		context_.frameContext.sensor.gain =
			camHelper_->gain(event.sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

		updateStatistics(frame, stats);
		break;
	}
	case EventQueueRequest: {
		unsigned int frame = event.frame;
		unsigned int bufferId = event.bufferId;

		rkisp1_params_cfg *params =
			reinterpret_cast<rkisp1_params_cfg *>(
				mappedBuffers_.at(bufferId).planes()[0].data());

		queueRequest(frame, params, event.controls);
		break;
	}
	default:
		LOG(IPARkISP1, Error) << "Unknown event " << event.op;
		break;
	}
}

void IPARkISP1::queueRequest(unsigned int frame, rkisp1_params_cfg *params,
			     [[maybe_unused]] const ControlList &controls)
{
	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	for (auto const &algo : algorithms_)
		algo->prepare(context_, params);

	RkISP1Action op;
	op.op = ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

void IPARkISP1::updateStatistics(unsigned int frame,
				 const rkisp1_stat_buffer *stats)
{
	unsigned int aeState = 0;

	for (auto const &algo : algorithms_)
		algo->process(context_, stats);

	setControls(frame);

	metadataReady(frame, aeState);
}

void IPARkISP1::setControls(unsigned int frame)
{
	RkISP1Action op;
	op.op = ActionV4L2Set;

	uint32_t exposure = context_.frameContext.agc.exposure;
	uint32_t gain = camHelper_->gainCode(context_.frameContext.agc.gain);

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));
	op.sensorControls = ctrls;

	queueFrameAction.emit(frame, op);
}

void IPARkISP1::metadataReady(unsigned int frame, unsigned int aeState)
{
	ControlList ctrls(controls::controls);

	if (aeState)
		ctrls.set(controls::AeLocked, aeState == 2);

	RkISP1Action op;
	op.op = ActionMetadata;
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
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
