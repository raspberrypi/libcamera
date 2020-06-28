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
#include <sys/mman.h>

#include <linux/rkisp1-config.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp1.h>
#include <libcamera/request.h>

#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/log.h"
#include "libcamera/internal/utils.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPARkISP1)

class IPARkISP1 : public IPAInterface
{
public:
	int init(const IPASettings &settings) override { return 0; }
	int start() override { return 0; }
	void stop() override {}

	void configure(const CameraSensorInfo &info,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *response) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPAOperationData &event) override;

private:
	void queueRequest(unsigned int frame, rkisp1_isp_params_cfg *params,
			  const ControlList &controls);
	void updateStatistics(unsigned int frame,
			      const rkisp1_stat_buffer *stats);

	void setControls(unsigned int frame);
	void metadataReady(unsigned int frame, unsigned int aeState);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, void *> buffersMemory_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	bool autoExposure_;
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;
};

/**
 * \todo The RkISP1 pipeline currently provides an empty CameraSensorInfo
 * if the connected sensor does not provide enough information to properly
 * assemble one. Make sure the reported sensor information are relevant
 * before accessing them.
 */
void IPARkISP1::configure(const CameraSensorInfo &info,
			  const std::map<unsigned int, IPAStream> &streamConfig,
			  const std::map<unsigned int, const ControlInfoMap &> &entityControls,
			  const IPAOperationData &ipaConfig,
			  IPAOperationData *result)
{
	if (entityControls.empty())
		return;

	ctrls_ = entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPARkISP1, Error) << "Can't find exposure control";
		return;
	}

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPARkISP1, Error) << "Can't find gain control";
		return;
	}

	autoExposure_ = true;

	minExposure_ = std::max<uint32_t>(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = minExposure_;

	minGain_ = std::max<uint32_t>(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = minGain_;

	LOG(IPARkISP1, Info)
		<< "Exposure: " << minExposure_ << "-" << maxExposure_
		<< " Gain: " << minGain_ << "-" << maxGain_;

	setControls(0);
}

void IPARkISP1::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		/*
		 * \todo Provide a helper to mmap() buffers (possibly exposed
		 * to applications).
		 */
		buffersMemory_[buffer.id] = mmap(NULL,
						 fb.planes()[0].length,
						 PROT_READ | PROT_WRITE,
						 MAP_SHARED,
						 fb.planes()[0].fd.fd(),
						 0);

		if (buffersMemory_[buffer.id] == MAP_FAILED) {
			int ret = -errno;
			LOG(IPARkISP1, Fatal) << "Failed to mmap buffer: "
					      << strerror(-ret);
		}
	}
}

void IPARkISP1::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		const auto fb = buffers_.find(id);
		if (fb == buffers_.end())
			continue;

		munmap(buffersMemory_[id], fb->second.planes()[0].length);
		buffersMemory_.erase(id);
		buffers_.erase(id);
	}
}

void IPARkISP1::processEvent(const IPAOperationData &event)
{
	switch (event.operation) {
	case RKISP1_IPA_EVENT_SIGNAL_STAT_BUFFER: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		const rkisp1_stat_buffer *stats =
			static_cast<rkisp1_stat_buffer *>(buffersMemory_[bufferId]);

		updateStatistics(frame, stats);
		break;
	}
	case RKISP1_IPA_EVENT_QUEUE_REQUEST: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		rkisp1_isp_params_cfg *params =
			static_cast<rkisp1_isp_params_cfg *>(buffersMemory_[bufferId]);

		queueRequest(frame, params, event.controls[0]);
		break;
	}
	default:
		LOG(IPARkISP1, Error) << "Unknown event " << event.operation;
		break;
	}
}

void IPARkISP1::queueRequest(unsigned int frame, rkisp1_isp_params_cfg *params,
			     const ControlList &controls)
{
	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	/* Auto Exposure on/off. */
	if (controls.contains(controls::AeEnable)) {
		autoExposure_ = controls.get(controls::AeEnable);
		if (autoExposure_)
			params->module_ens = CIFISP_MODULE_AEC;

		params->module_en_update = CIFISP_MODULE_AEC;
	}

	IPAOperationData op;
	op.operation = RKISP1_IPA_ACTION_PARAM_FILLED;

	queueFrameAction.emit(frame, op);
}

void IPARkISP1::updateStatistics(unsigned int frame,
				 const rkisp1_stat_buffer *stats)
{
	const cifisp_stat *params = &stats->params;
	unsigned int aeState = 0;

	if (stats->meas_type & CIFISP_STAT_AUTOEXP) {
		const cifisp_ae_stat *ae = &params->ae;

		const unsigned int target = 60;

		unsigned int value = 0;
		unsigned int num = 0;
		for (int i = 0; i < CIFISP_AE_MEAN_MAX; i++) {
			if (ae->exp_mean[i] <= 15)
				continue;

			value += ae->exp_mean[i];
			num++;
		}
		value /= num;

		double factor = (double)target / value;

		if (frame % 3 == 0) {
			double exposure;

			exposure = factor * exposure_ * gain_ / minGain_;
			exposure_ = utils::clamp<uint64_t>((uint64_t)exposure,
							   minExposure_,
							   maxExposure_);

			exposure = exposure / exposure_ * minGain_;
			gain_ = utils::clamp<uint64_t>((uint64_t)exposure,
						       minGain_, maxGain_);

			setControls(frame + 1);
		}

		aeState = fabs(factor - 1.0f) < 0.05f ? 2 : 1;
	}

	metadataReady(frame, aeState);
}

void IPARkISP1::setControls(unsigned int frame)
{
	IPAOperationData op;
	op.operation = RKISP1_IPA_ACTION_V4L2_SET;

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	op.controls.push_back(ctrls);

	queueFrameAction.emit(frame, op);
}

void IPARkISP1::metadataReady(unsigned int frame, unsigned int aeState)
{
	ControlList ctrls(controls::controls);

	if (aeState)
		ctrls.set(controls::AeLocked, aeState == 2);

	IPAOperationData op;
	op.operation = RKISP1_IPA_ACTION_METADATA;
	op.controls.push_back(ctrls);

	queueFrameAction.emit(frame, op);
}

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

struct ipa_context *ipaCreate()
{
	return new IPAInterfaceWrapper(std::make_unique<IPARkISP1>());
}
}

} /* namespace libcamera */
