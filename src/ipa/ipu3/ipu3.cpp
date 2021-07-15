/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.cpp - IPU3 Image Processing Algorithms
 */

#include <array>
#include <stdint.h>
#include <utility>

#include <linux/intel-ipu3.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/request.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "ipu3_agc.h"
#include "ipu3_awb.h"
#include "libipa/camera_sensor_helper.h"

static constexpr uint32_t kMaxCellWidthPerSet = 160;
static constexpr uint32_t kMaxCellHeightPerSet = 56;

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU3)

namespace ipa::ipu3 {

class IPAIPU3 : public IPAIPU3Interface
{
public:
	int init(const IPASettings &settings,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
		 ControlInfoMap *ipaControls) override;

	int start() override;
	void stop() override {}

	int configure(const IPAConfigInfo &configInfo) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPU3Event &event) override;

private:
	void processControls(unsigned int frame, const ControlList &controls);
	void fillParams(unsigned int frame, ipu3_uapi_params *params);
	void parseStatistics(unsigned int frame,
			     int64_t frameTimestamp,
			     const ipu3_uapi_stats_3a *stats);

	void setControls(unsigned int frame);
	void calculateBdsGrid(const Size &bdsOutputSize);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap ctrls_;

	IPACameraSensorInfo sensorInfo_;

	/* Camera sensor controls. */
	uint32_t defVBlank_;
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;

	/* Interface to the AWB algorithm */
	std::unique_ptr<IPU3Awb> awbAlgo_;
	/* Interface to the AEC/AGC algorithm */
	std::unique_ptr<IPU3Agc> agcAlgo_;
	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct ipu3_uapi_params params_;

	struct ipu3_uapi_grid_config bdsGrid_;
};

/**
 * Initialize the IPA module and its controls.
 *
 * This function receives the camera sensor information from the pipeline
 * handler, computes the limits of the controls it handles and returns
 * them in the \a ipaControls output parameter.
 */
int IPAIPU3::init(const IPASettings &settings,
		  const IPACameraSensorInfo &sensorInfo,
		  const ControlInfoMap &sensorControls,
		  ControlInfoMap *ipaControls)
{
	camHelper_ = CameraSensorHelperFactory::create(settings.sensorModel);
	if (camHelper_ == nullptr) {
		LOG(IPAIPU3, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	/* Initialize Controls. */
	ControlInfoMap::Map controls{};

	/*
	 * Compute exposure time limits.
	 *
	 * Initialize the control using the line length and pixel rate of the
	 * current configuration converted to microseconds. Use the
	 * V4L2_CID_EXPOSURE control to get exposure min, max and default and
	 * convert it from lines to microseconds.
	 */
	double lineDuration = sensorInfo.lineLength / (sensorInfo.pixelRate / 1e6);
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration;
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration;
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration;
	controls[&controls::ExposureTime] = ControlInfo(minExposure, maxExposure,
							defExposure);

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

	controls[&controls::FrameDurationLimits] = ControlInfo(frameDurations[0],
							       frameDurations[1],
							       frameDurations[2]);

	*ipaControls = ControlInfoMap(std::move(controls), controls::controls);

	return 0;
}

int IPAIPU3::start()
{
	setControls(0);

	return 0;
}

/**
 * This function calculates a grid for the AWB algorithm in the IPU3 firmware.
 * Its input is the BDS output size calculated in the ImgU.
 * It is limited for now to the simplest method: find the lesser error
 * with the width/height and respective log2 width/height of the cells.
 *
 * \todo The frame is divided into cells which can be 8x8 => 128x128.
 * As a smaller cell improves the algorithm precision, adapting the
 * x_start and y_start parameters of the grid would provoke a loss of
 * some pixels but would also result in more accurate algorithms.
 */
void IPAIPU3::calculateBdsGrid(const Size &bdsOutputSize)
{
	uint32_t minError = std::numeric_limits<uint32_t>::max();
	Size best;
	Size bestLog2;
	bdsGrid_ = {};

	for (uint32_t widthShift = 3; widthShift <= 7; ++widthShift) {
		uint32_t width = std::min(kMaxCellWidthPerSet,
					  bdsOutputSize.width >> widthShift);
		width = width << widthShift;
		for (uint32_t heightShift = 3; heightShift <= 7; ++heightShift) {
			int32_t height = std::min(kMaxCellHeightPerSet,
						  bdsOutputSize.height >> heightShift);
			height = height << heightShift;
			uint32_t error  = std::abs(static_cast<int>(width - bdsOutputSize.width))
							+ std::abs(static_cast<int>(height - bdsOutputSize.height));

			if (error > minError)
				continue;

			minError = error;
			best.width = width;
			best.height = height;
			bestLog2.width = widthShift;
			bestLog2.height = heightShift;
		}
	}

	bdsGrid_.width = best.width >> bestLog2.width;
	bdsGrid_.block_width_log2 = bestLog2.width;
	bdsGrid_.height = best.height >> bestLog2.height;
	bdsGrid_.block_height_log2 = bestLog2.height;

	LOG(IPAIPU3, Debug) << "Best grid found is: ("
			    << (int)bdsGrid_.width << " << " << (int)bdsGrid_.block_width_log2 << ") x ("
			    << (int)bdsGrid_.height << " << " << (int)bdsGrid_.block_height_log2 << ")";
}

int IPAIPU3::configure(const IPAConfigInfo &configInfo)
{
	if (configInfo.entityControls.empty()) {
		LOG(IPAIPU3, Error) << "No controls provided";
		return -ENODATA;
	}

	sensorInfo_ = configInfo.sensorInfo;

	ctrls_ = configInfo.entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find exposure control";
		return -EINVAL;
	}

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find gain control";
		return -EINVAL;
	}

	const auto itVBlank = ctrls_.find(V4L2_CID_VBLANK);
	if (itVBlank == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find VBLANK control";
		return -EINVAL;
	}

	minExposure_ = std::max(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = minExposure_;

	minGain_ = std::max(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = minGain_;

	defVBlank_ = itVBlank->second.def().get<int32_t>();

	params_ = {};

	calculateBdsGrid(configInfo.bdsOutputSize);

	awbAlgo_ = std::make_unique<IPU3Awb>();
	awbAlgo_->initialise(params_, configInfo.bdsOutputSize, bdsGrid_);

	agcAlgo_ = std::make_unique<IPU3Agc>();
	agcAlgo_->initialise(bdsGrid_, sensorInfo_);

	return 0;
}

void IPAIPU3::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
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

void IPAIPU3::processEvent(const IPU3Event &event)
{
	switch (event.op) {
	case EventProcessControls: {
		processControls(event.frame, event.controls);
		break;
	}
	case EventStatReady: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find stats buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		const ipu3_uapi_stats_3a *stats =
			reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

		parseStatistics(event.frame, event.frameTimestamp, stats);
		break;
	}
	case EventFillParams: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find param buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		ipu3_uapi_params *params =
			reinterpret_cast<ipu3_uapi_params *>(mem.data());

		fillParams(event.frame, params);
		break;
	}
	default:
		LOG(IPAIPU3, Error) << "Unknown event " << event.op;
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
	if (agcAlgo_->updateControls())
		awbAlgo_->updateWbParameters(params_, agcAlgo_->gamma());

	*params = params_;

	IPU3Action op;
	op.op = ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] int64_t frameTimestamp,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	double gain = camHelper_->gain(gain_);
	agcAlgo_->process(stats, exposure_, gain);
	gain_ = camHelper_->gainCode(gain);

	awbAlgo_->calculateWBGains(stats);

	if (agcAlgo_->updateControls())
		setControls(frame);

	/* \todo Use VBlank value calculated from each frame exposure. */
	int64_t frameDuration = sensorInfo_.lineLength * (defVBlank_ + sensorInfo_.outputSize.height) /
				(sensorInfo_.pixelRate / 1e6);
	ctrls.set(controls::FrameDuration, frameDuration);

	IPU3Action op;
	op.op = ActionMetadataReady;
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::setControls(unsigned int frame)
{
	IPU3Action op;
	op.op = ActionSetSensorControls;

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

} /* namespace ipa::ipu3 */

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

IPAInterface *ipaCreate()
{
	return new ipa::ipu3::IPAIPU3();
}
}

} /* namespace libcamera */
