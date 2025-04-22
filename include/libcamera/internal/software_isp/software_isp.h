/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * Simple software ISP implementation
 */

#pragma once

#include <deque>
#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <stdint.h>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/log.h>
#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include <libcamera/ipa/soft_ipa_interface.h>
#include <libcamera/ipa/soft_ipa_proxy.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/debayer_params.h"

namespace libcamera {

class DebayerCpu;
class FrameBuffer;
class PixelFormat;
class Stream;
struct StreamConfiguration;

LOG_DECLARE_CATEGORY(SoftwareIsp)

class SoftwareIsp : public Object
{
public:
	SoftwareIsp(PipelineHandler *pipe, const CameraSensor *sensor,
		    ControlInfoMap *ipaControls);
	~SoftwareIsp();

	int loadConfiguration([[maybe_unused]] const std::string &filename) { return 0; }

	bool isValid() const;

	std::vector<PixelFormat> formats(PixelFormat input);

	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize);

	std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &outputFormat, const Size &size);

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
		      const ipa::soft::IPAConfigInfo &configInfo);

	int exportBuffers(const Stream *stream, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);

	void processStats(const uint32_t frame, const uint32_t bufferId,
			  const ControlList &sensorControls);

	int start();
	void stop();

	void queueRequest(const uint32_t frame, const ControlList &controls);
	int queueBuffers(uint32_t frame, FrameBuffer *input,
			 const std::map<const Stream *, FrameBuffer *> &outputs);

	void process(uint32_t frame, FrameBuffer *input, FrameBuffer *output);

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;
	Signal<uint32_t, uint32_t> ispStatsReady;
	Signal<uint32_t, const ControlList &> metadataReady;
	Signal<const ControlList &> setSensorControls;

private:
	void saveIspParams();
	void setSensorCtrls(const ControlList &sensorControls);
	void statsReady(uint32_t frame, uint32_t bufferId);
	void inputReady(FrameBuffer *input);
	void outputReady(FrameBuffer *output);

	std::unique_ptr<DebayerCpu> debayer_;
	Thread ispWorkerThread_;
	SharedMemObject<DebayerParams> sharedParams_;
	DebayerParams debayerParams_;
	DmaBufAllocator dmaHeap_;
	bool ccmEnabled_;

	std::unique_ptr<ipa::soft::IPAProxySoft> ipa_;
	std::deque<FrameBuffer *> queuedInputBuffers_;
	std::deque<FrameBuffer *> queuedOutputBuffers_;
};

} /* namespace libcamera */
