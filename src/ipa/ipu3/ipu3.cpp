/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.cpp - IPU3 Image Processing Algorithms
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdint.h>
#include <utility>
#include <vector>

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

#include "algorithms/agc.h"
#include "algorithms/algorithm.h"
#include "algorithms/awb.h"
#include "algorithms/tone_mapping.h"
#include "libipa/camera_sensor_helper.h"

/**
 * \file ipa_context.h
 * \brief Context and state information shared between the algorithms
 */

/**
 * \struct IPASessionConfiguration
 * \brief Session configuration for the IPA module
 *
 * The session configuration contains all IPA configuration parameters that
 * remain constant during the capture session, from IPA module start to stop.
 * It is typically set during the configure() operation of the IPA module, but
 * may also be updated in the start() operation.
 */

/**
 * \struct IPAFrameContext
 * \brief Per-frame context for algorithms
 *
 * The frame context stores data specific to a single frame processed by the
 * IPA. Each frame processed by the IPA has a context associated with it,
 * accessible through the IPAContext structure.
 *
 * \todo Detail how to access contexts for a particular frame
 *
 * Each of the fields in the frame context belongs to either a specific
 * algorithm, or to the top-level IPA module. A field may be read by any
 * algorithm, but should only be written by its owner.
 */

/**
 * \struct IPAContext
 * \brief Global IPA context data shared between all algorithms
 *
 * \var IPAContext::configuration
 * \brief The IPA session configuration, immutable during the session
 *
 * \var IPAContext::frameContext
 * \brief The frame context for the frame being processed
 *
 * \todo While the frame context is supposed to be per-frame, this
 * single frame context stores data related to both the current frame
 * and the previous frames, with fields being updated as the algorithms
 * are run. This needs to be turned into real per-frame data storage.
 */

/**
 * \struct IPASessionConfiguration::grid
 * \brief Grid configuration of the IPA
 *
 * \var IPASessionConfiguration::grid::bdsGrid
 * \brief Bayer Down Scaler grid plane config used by the kernel
 *
 * \var IPASessionConfiguration::grid::bdsOutputSize
 * \brief BDS output size configured by the pipeline handler
 */

/**
 * \struct IPAFrameContext::agc
 * \brief Context for the Automatic Gain Control algorithm
 *
 * The exposure and gain determined are expected to be applied to the sensor
 * at the earliest opportunity.
 *
 * \var IPAFrameContext::agc::exposure
 * \brief Exposure time expressed as a number of lines
 *
 * \var IPAFrameContext::agc::gain
 * \brief Analogue gain multiplier
 *
 * The gain should be adapted to the sensor specific gain code before applying.
 */

/**
 * \struct IPAFrameContext::awb
 * \brief Context for the Automatic White Balance algorithm
 *
 * \struct IPAFrameContext::awb::gains
 * \brief White balance gains
 *
 * \var IPAFrameContext::awb::gains::red
 * \brief White balance gain for R channel
 *
 * \var IPAFrameContext::awb::gains::green
 * \brief White balance gain for G channel
 *
 * \var IPAFrameContext::awb::gains::blue
 * \brief White balance gain for B channel
 */

/**
 * \struct IPAFrameContext::toneMapping
 * \brief Context for ToneMapping and Gamma control
 *
 * \var IPAFrameContext::toneMapping::gammaCorrection
 * \brief Per-pixel tone mapping implemented as a LUT
 *
 * The LUT structure is defined by the IPU3 kernel interface. See
 * <linux/intel-ipu3.h> struct ipu3_uapi_gamma_corr_lut for further details.
 */

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

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Maintain the algorithms used by the IPA */
	std::list<std::unique_ptr<ipa::ipu3::Algorithm>> algorithms_;

	/* Local parameter storage */
	struct IPAContext context_;
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

	/* Construct our Algorithms */
	algorithms_.push_back(std::make_unique<algorithms::Agc>());
	algorithms_.push_back(std::make_unique<algorithms::Awb>());
	algorithms_.push_back(std::make_unique<algorithms::ToneMapping>());

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

	/* Set the BDS output size in the IPAConfiguration structure */
	context_.configuration.grid.bdsOutputSize = bdsOutputSize;

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

	struct ipu3_uapi_grid_config &bdsGrid = context_.configuration.grid.bdsGrid;
	bdsGrid.x_start = 0;
	bdsGrid.y_start = 0;
	bdsGrid.width = best.width >> bestLog2.width;
	bdsGrid.block_width_log2 = bestLog2.width;
	bdsGrid.height = best.height >> bestLog2.height;
	bdsGrid.block_height_log2 = bestLog2.height;

	LOG(IPAIPU3, Debug) << "Best grid found is: ("
			    << (int)bdsGrid.width << " << " << (int)bdsGrid.block_width_log2 << ") x ("
			    << (int)bdsGrid.height << " << " << (int)bdsGrid.block_height_log2 << ")";
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

	/* Clean context at configuration */
	context_ = {};

	calculateBdsGrid(configInfo.bdsOutputSize);

	for (auto const &algo : algorithms_) {
		int ret = algo->configure(context_, configInfo);
		if (ret)
			return ret;
	}

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
	for (auto const &algo : algorithms_)
		algo->prepare(context_, params);

	IPU3Action op;
	op.op = ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] int64_t frameTimestamp,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	/* \todo These fields should not be written by the IPAIPU3 layer */
	context_.frameContext.agc.gain = camHelper_->gain(gain_);
	context_.frameContext.agc.exposure = exposure_;

	for (auto const &algo : algorithms_)
		algo->process(context_, stats);

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

	exposure_ = context_.frameContext.agc.exposure;
	gain_ = camHelper_->gainCode(context_.frameContext.agc.gain);

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
