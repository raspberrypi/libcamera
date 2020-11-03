/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.cpp - IPU3 Image Processing Algorithms
 */

#include <libcamera/ipa/ipu3.h>

#include <stdint.h>
#include <sys/mman.h>

#include <linux/intel-ipu3.h>
#include <linux/v4l2-controls.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/request.h>

#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/buffer.h"
#include "libcamera/internal/log.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU3)

class IPAIPU3 : public IPAInterface
{
public:
	int init([[maybe_unused]] const IPASettings &settings) override
	{
		return 0;
	}
	int start([[maybe_unused]] const IPAOperationData &data,
		  [[maybe_unused]] IPAOperationData *result) override { return 0; }
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
	void processControls(unsigned int frame, const ControlList &controls);
	void fillParams(unsigned int frame, ipu3_uapi_params *params);
	void parseStatistics(unsigned int frame,
			     const ipu3_uapi_stats_3a *stats);

	void setControls(unsigned int frame);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;
};

void IPAIPU3::configure([[maybe_unused]] const CameraSensorInfo &info,
			[[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
			const std::map<unsigned int, const ControlInfoMap &> &entityControls,
			[[maybe_unused]] const IPAOperationData &ipaConfig,
			[[maybe_unused]] IPAOperationData *result)
{
	if (entityControls.empty())
		return;

	ctrls_ = entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find exposure control";
		return;
	}

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find gain control";
		return;
	}

	minExposure_ = std::max(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = maxExposure_;

	minGain_ = std::max(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = maxGain_;

	setControls(0);
}

void IPAIPU3::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, PROT_READ | PROT_WRITE));
	}
}

void IPAIPU3::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(it);
	}
}

void IPAIPU3::processEvent(const IPAOperationData &event)
{
	switch (event.operation) {
	case IPU3_IPA_EVENT_PROCESS_CONTROLS: {
		unsigned int frame = event.data[0];
		processControls(frame, event.controls[0]);
		break;
	}
	case IPU3_IPA_EVENT_STAT_READY: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		auto it = buffers_.find(bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find stats buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		const ipu3_uapi_stats_3a *stats =
			reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

		parseStatistics(frame, stats);
		break;
	}
	case IPU3_IPA_EVENT_FILL_PARAMS: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		auto it = buffers_.find(bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find param buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		ipu3_uapi_params *params =
			reinterpret_cast<ipu3_uapi_params *>(mem.data());

		fillParams(frame, params);
		break;
	}
	default:
		LOG(IPAIPU3, Error) << "Unknown event " << event.operation;
		break;
	}
}

void IPAIPU3::processControls([[maybe_unused]] unsigned int frame,
			      [[maybe_unused]] const ControlList &controls)
{
	/* \todo Start processing for 'frame' based on 'controls'. */
}

void IPAIPU3::fillParams(unsigned int frame, ipu3_uapi_params *params)
{
	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	/* \todo Fill in parameters buffer. */

	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_PARAM_FILLED;

	queueFrameAction.emit(frame, op);

	/* \todo Calculate new values for exposure_ and gain_. */
	setControls(frame);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	/* \todo React to statistics and update internal state machine. */
	/* \todo Add meta-data information to ctrls. */

	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_METADATA_READY;
	op.controls.push_back(ctrls);

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::setControls(unsigned int frame)
{
	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_SET_SENSOR_CONTROLS;

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
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
	"PipelineHandlerIPU3",
	"ipu3",
};

struct ipa_context *ipaCreate()
{
	return new IPAInterfaceWrapper(std::make_unique<IPAIPU3>());
}
}

} /* namespace libcamera */
