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
#include <libcamera/base/utils.h>

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
#include "algorithms/blc.h"
#include "algorithms/tone_mapping.h"
#include "libipa/camera_sensor_helper.h"

/* Minimum grid width, expressed as a number of cells */
static constexpr uint32_t kMinGridWidth = 16;
/* Maximum grid width, expressed as a number of cells */
static constexpr uint32_t kMaxGridWidth = 80;
/* Minimum grid height, expressed as a number of cells */
static constexpr uint32_t kMinGridHeight = 16;
/* Maximum grid height, expressed as a number of cells */
static constexpr uint32_t kMaxGridHeight = 60;
/* log2 of the minimum grid cell width and height, in pixels */
static constexpr uint32_t kMinCellSizeLog2 = 3;
/* log2 of the maximum grid cell width and height, in pixels */
static constexpr uint32_t kMaxCellSizeLog2 = 6;

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU3)

using namespace std::literals::chrono_literals;

namespace ipa::ipu3 {

/**
 * \brief The IPU3 IPA implementation
 *
 * The IPU3 Pipeline defines an IPU3-specific interface for communication
 * between the PipelineHandler and the IPA module.
 *
 * We extend the IPAIPU3Interface to implement our algorithms and handle events
 * from the IPU3 PipelineHandler to satisfy requests from the application.
 *
 * At initialisation time, a CameraSensorHelper is instantiated to support
 * camera-specific calculations, while the default controls are computed, and
 * the algorithms are constructed and placed in an ordered list.
 *
 * The IPU3 ImgU operates with a grid layout to divide the overall frame into
 * rectangular cells of pixels. When the IPA is configured, we determine the
 * best grid for the statistics based on the pipeline handler Bayer Down Scaler
 * output size.
 *
 * Two main events are then handled to operate the IPU3 ImgU by populating its
 * parameter buffer, and adapting the settings of the sensor attached to the
 * IPU3 CIO2 through sensor-specific V4L2 controls.
 *
 * When the event \a EventFillParams occurs we populate the ImgU parameter
 * buffer with settings to configure the device in preparation for handling the
 * frame queued in the Request.
 *
 * When the frame has completed processing, the ImgU will generate a statistics
 * buffer which is given to the IPA as part of the \a EventStatReady event. At
 * this event we run the algorithms to parse the statistics and cache any
 * results for the next \a EventFillParams event.
 *
 * The individual algorithms are split into modular components that are called
 * iteratively to allow them to process statistics from the ImgU in a defined
 * order.
 *
 * The current implementation supports three core algorithms:
 * - Automatic white balance (AWB)
 * - Automatic gain and exposure control (AGC)
 * - Black level correction (BLC)
 * - Tone mapping (Gamma)
 *
 * AWB is implemented using a Greyworld algorithm, and calculates the red and
 * blue gains to apply to generate a neutral grey frame overall.
 *
 * AGC is handled by calculating a histogram of the green channel to estimate an
 * analogue gain and shutter time which will provide a well exposed frame. A
 * low-pass IIR filter is used to smooth the changes to the sensor to reduce
 * perceivable steps.
 *
 * The tone mapping algorithm provides a gamma correction table to improve the
 * contrast of the scene.
 *
 * The black level compensation algorithm subtracts a hardcoded black level from
 * all pixels.
 *
 * The IPU3 ImgU has further processing blocks to support image quality
 * improvements through bayer and temporal noise reductions, however those are
 * not supported in the current implementation, and will use default settings as
 * provided by the kernel driver.
 *
 * Demosaicing is operating with the default parameters and could be further
 * optimised to provide improved sharpening coefficients, checker artifact
 * removal, and false color correction.
 *
 * Additional image enhancements can be made by providing lens and
 * sensor-specific tuning to adapt for Black Level compensation (BLC), Lens
 * shading correction (SHD) and Color correction (CCM).
 */
class IPAIPU3 : public IPAIPU3Interface
{
public:
	int init(const IPASettings &settings,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
		 ControlInfoMap *ipaControls) override;

	int start() override;
	void stop() override;

	int configure(const IPAConfigInfo &configInfo,
		      ControlInfoMap *ipaControls) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPU3Event &event) override;

private:
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void updateSessionConfiguration(const ControlInfoMap &sensorControls);
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

	utils::Duration lineDuration_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Maintain the algorithms used by the IPA */
	std::list<std::unique_ptr<ipa::ipu3::Algorithm>> algorithms_;

	/* Local parameter storage */
	struct IPAContext context_;
};

/**
 * \brief Compute IPASessionConfiguration using the sensor information and the
 * sensor V4L2 controls
 */
void IPAIPU3::updateSessionConfiguration(const ControlInfoMap &sensorControls)
{
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>();
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>();

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
	context_.configuration.agc.minShutterSpeed = minExposure * lineDuration_;
	context_.configuration.agc.maxShutterSpeed = maxExposure * lineDuration_;
	context_.configuration.agc.minAnalogueGain = camHelper_->gain(minGain);
	context_.configuration.agc.maxAnalogueGain = camHelper_->gain(maxGain);
}

/**
 * \brief Compute camera controls using the sensor information and the sensor
 * V4L2 controls
 *
 * Some of the camera controls are computed by the pipeline handler, some others
 * by the IPA module which is in charge of handling, for example, the exposure
 * time and the frame duration.
 *
 * This function computes:
 * - controls::ExposureTime
 * - controls::FrameDurationLimits
 */
void IPAIPU3::updateControls(const IPACameraSensorInfo &sensorInfo,
			     const ControlInfoMap &sensorControls,
			     ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map controls{};

	/*
	 * Compute exposure time limits by using line length and pixel rate
	 * converted to microseconds. Use the V4L2_CID_EXPOSURE control to get
	 * exposure min, max and default and convert it from lines to
	 * microseconds.
	 */
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration_.get<std::micro>();
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration_.get<std::micro>();
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration_.get<std::micro>();
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
}

/**
 * \brief Initialize the IPA module and its controls
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

	/* Construct our Algorithms */
	algorithms_.push_back(std::make_unique<algorithms::Agc>());
	algorithms_.push_back(std::make_unique<algorithms::Awb>());
	algorithms_.push_back(std::make_unique<algorithms::BlackLevelCorrection>());
	algorithms_.push_back(std::make_unique<algorithms::ToneMapping>());

	/* Initialize controls. */
	updateControls(sensorInfo, sensorControls, ipaControls);

	return 0;
}

/**
 * \brief Perform any processing required before the first frame
 */
int IPAIPU3::start()
{
	/*
	 * Set the sensors V4L2 controls before the first frame to ensure that
	 * we have an expected and known configuration from the start.
	 */
	setControls(0);

	return 0;
}

/**
 * \brief Ensure that all processing has completed
 */
void IPAIPU3::stop()
{
}

/**
 * \brief Calculate a grid for the AWB statistics
 *
 * This function calculates a grid for the AWB algorithm in the IPU3 firmware.
 * Its input is the BDS output size calculated in the ImgU.
 * It is limited for now to the simplest method: find the lesser error
 * with the width/height and respective log2 width/height of the cells.
 *
 * \todo The frame is divided into cells which can be 8x8 => 64x64.
 * As a smaller cell improves the algorithm precision, adapting the
 * x_start and y_start parameters of the grid would provoke a loss of
 * some pixels but would also result in more accurate algorithms.
 */
void IPAIPU3::calculateBdsGrid(const Size &bdsOutputSize)
{
	Size best;
	Size bestLog2;

	/* Set the BDS output size in the IPAConfiguration structure */
	context_.configuration.grid.bdsOutputSize = bdsOutputSize;

	uint32_t minError = std::numeric_limits<uint32_t>::max();
	for (uint32_t shift = kMinCellSizeLog2; shift <= kMaxCellSizeLog2; ++shift) {
		uint32_t width = std::clamp(bdsOutputSize.width >> shift,
					    kMinGridWidth,
					    kMaxGridWidth);

		width = width << shift;
		uint32_t error = utils::abs_diff(width, bdsOutputSize.width);
		if (error >= minError)
			continue;

		minError = error;
		best.width = width;
		bestLog2.width = shift;
	}

	minError = std::numeric_limits<uint32_t>::max();
	for (uint32_t shift = kMinCellSizeLog2; shift <= kMaxCellSizeLog2; ++shift) {
		uint32_t height = std::clamp(bdsOutputSize.height >> shift,
					     kMinGridHeight,
					     kMaxGridHeight);

		height = height << shift;
		uint32_t error = utils::abs_diff(height, bdsOutputSize.height);
		if (error >= minError)
			continue;

		minError = error;
		best.height = height;
		bestLog2.height = shift;
	}

	struct ipu3_uapi_grid_config &bdsGrid = context_.configuration.grid.bdsGrid;
	bdsGrid.x_start = 0;
	bdsGrid.y_start = 0;
	bdsGrid.width = best.width >> bestLog2.width;
	bdsGrid.block_width_log2 = bestLog2.width;
	bdsGrid.height = best.height >> bestLog2.height;
	bdsGrid.block_height_log2 = bestLog2.height;

	/* The ImgU pads the lines to a multiple of 4 cells. */
	context_.configuration.grid.stride = utils::alignUp(bdsGrid.width, 4);

	LOG(IPAIPU3, Debug) << "Best grid found is: ("
			    << (int)bdsGrid.width << " << " << (int)bdsGrid.block_width_log2 << ") x ("
			    << (int)bdsGrid.height << " << " << (int)bdsGrid.block_height_log2 << ")";
}

/**
 * \brief Configure the IPU3 IPA
 * \param[in] configInfo The IPA configuration data, received from the pipeline
 * handler
 * \param[in] ipaControls The IPA controls to update
 *
 * Calculate the best grid for the statistics based on the pipeline handler BDS
 * output, and parse the minimum and maximum exposure and analogue gain control
 * values.
 *
 * \todo Document what the BDS is, ideally in a block diagram of the ImgU.
 *
 * All algorithm modules are called to allow them to prepare the
 * \a IPASessionConfiguration structure for the \a IPAContext.
 */
int IPAIPU3::configure(const IPAConfigInfo &configInfo,
		       ControlInfoMap *ipaControls)
{
	if (configInfo.sensorControls.empty()) {
		LOG(IPAIPU3, Error) << "No sensor controls provided";
		return -ENODATA;
	}

	sensorInfo_ = configInfo.sensorInfo;

	/*
	 * Compute the sensor V4L2 controls to be used by the algorithms and
	 * to be set on the sensor.
	 */
	ctrls_ = configInfo.sensorControls;

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

	minExposure_ = itExp->second.min().get<int32_t>();
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = minExposure_;

	minGain_ = itGain->second.min().get<int32_t>();
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = minGain_;

	defVBlank_ = itVBlank->second.def().get<int32_t>();

	/* Clean context at configuration */
	context_ = {};

	calculateBdsGrid(configInfo.bdsOutputSize);

	lineDuration_ = sensorInfo_.lineLength * 1.0s / sensorInfo_.pixelRate;

	/* Update the camera controls using the new sensor settings. */
	updateControls(sensorInfo_, ctrls_, ipaControls);

	/* Update the IPASessionConfiguration using the sensor settings. */
	updateSessionConfiguration(ctrls_);

	for (auto const &algo : algorithms_) {
		int ret = algo->configure(context_, configInfo);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * \brief Map the parameters and stats buffers allocated in the pipeline handler
 * \param[in] buffers The buffers to map
 */
void IPAIPU3::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

/**
 * \brief Unmap the parameters and stats buffers
 * \param[in] ids The IDs of the buffers to unmap
 */
void IPAIPU3::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(it);
	}
}

/**
 * \brief Process an event generated by the pipeline handler
 * \param[in] event The event sent from pipeline handler
 *
 * The expected event handling over the lifetime of a Request has
 * the following sequence:
 *
 *   - EventProcessControls : Handle controls from a new Request
 *   - EventFillParams : Prepare the ISP to process the Request
 *   - EventStatReady : Process statistics after ISP completion
 */
void IPAIPU3::processEvent(const IPU3Event &event)
{
	switch (event.op) {
	case EventProcessControls: {
		processControls(event.frame, event.controls);
		break;
	}
	case EventFillParams: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find param buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.planes()[0];
		ipu3_uapi_params *params =
			reinterpret_cast<ipu3_uapi_params *>(mem.data());

		fillParams(event.frame, params);
		break;
	}
	case EventStatReady: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find stats buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.planes()[0];
		const ipu3_uapi_stats_3a *stats =
			reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

		context_.frameContext.sensor.exposure = event.sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
		context_.frameContext.sensor.gain = camHelper_->gain(event.sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

		parseStatistics(event.frame, event.frameTimestamp, stats);
		break;
	}
	default:
		LOG(IPAIPU3, Error) << "Unknown event " << event.op;
		break;
	}
}

/**
 * \brief Process a control list for a request from the application
 * \param[in] frame The number of the frame which will be processed next
 * \param[in] controls The controls for the \a frame
 *
 * Parse the request to handle any IPA-managed controls that were set from the
 * application such as manual sensor settings.
 */
void IPAIPU3::processControls([[maybe_unused]] unsigned int frame,
			      [[maybe_unused]] const ControlList &controls)
{
	/* \todo Start processing for 'frame' based on 'controls'. */
}

/**
 * \brief Fill the ImgU parameter buffer for the next frame
 * \param[in] frame The number of the latest frame processed
 * \param[out] params The parameter buffer to fill
 *
 * Algorithms are expected to fill the IPU3 parameter buffer for the next
 * frame given their most recent processing of the ImgU statistics.
 */
void IPAIPU3::fillParams(unsigned int frame, ipu3_uapi_params *params)
{
	/*
	 * The incoming params buffer may contain uninitialised data, or the
	 * parameters of previously queued frames. Clearing the entire buffer
	 * may be an expensive operation, and the kernel will only read from
	 * structures which have their associated use-flag set.
	 *
	 * It is the responsibility of the algorithms to set the use flags
	 * accordingly for any data structure they update during prepare().
	 */
	params->use = {};

	for (auto const &algo : algorithms_)
		algo->prepare(context_, params);

	IPU3Action op;
	op.op = ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

/**
 * \brief Process the statistics generated by the ImgU
 * \param[in] frame The number of the latest frame processed
 * \param[in] frameTimestamp The current frame timestamp
 * \param[in] stats The IPU3 statistics and ISP results
 *
 * Parse the most recently processed image statistics from the ImgU. The
 * statistics are passed to each algorithm module to run their calculations and
 * update their state accordingly.
 */
void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] int64_t frameTimestamp,
			      const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	for (auto const &algo : algorithms_)
		algo->process(context_, stats);

	setControls(frame);

	/* \todo Use VBlank value calculated from each frame exposure. */
	int64_t frameDuration = (defVBlank_ + sensorInfo_.outputSize.height) * lineDuration_.get<std::micro>();
	ctrls.set(controls::FrameDuration, frameDuration);

	ctrls.set(controls::AnalogueGain, context_.frameContext.sensor.gain);

	ctrls.set(controls::ColourTemperature, context_.frameContext.awb.temperatureK);

	ctrls.set(controls::ExposureTime, context_.frameContext.sensor.exposure * lineDuration_.get<std::micro>());

	/*
	 * \todo The Metadata provides a path to getting extended data
	 * out to the application. Further data such as a simplifed Histogram
	 * might have value to be exposed, however such data may be
	 * difficult to report in a generically parsable way and we
	 * likely want to avoid putting platform specific metadata in.
	 */

	IPU3Action op;
	op.op = ActionMetadataReady;
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

/**
 * \brief Handle sensor controls for a given \a frame number
 * \param[in] frame The frame on which the sensor controls should be set
 *
 * Send the desired sensor control values to the pipeline handler to request
 * that they are applied on the camera sensor.
 */
void IPAIPU3::setControls(unsigned int frame)
{
	IPU3Action op;
	op.op = ActionSetSensorControls;

	exposure_ = context_.frameContext.agc.exposure;
	gain_ = camHelper_->gainCode(context_.frameContext.agc.gain);

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	op.sensorControls = ctrls;

	queueFrameAction.emit(frame, op);
}

} /* namespace ipa::ipu3 */

/**
 * \brief External IPA module interface
 *
 * The IPAModuleInfo is required to match an IPA module construction against the
 * intented pipeline handler with the module. The API and pipeline handler
 * versions must match the corresponding IPA interface and pipeline handler.
 *
 * \sa struct IPAModuleInfo
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerIPU3",
	"ipu3",
};

/**
 * \brief Create an instance of the IPA interface
 *
 * This function is the entry point of the IPA module. It is called by the IPA
 * manager to create an instance of the IPA interface for each camera. When
 * matched against with a pipeline handler, the IPAManager will construct an IPA
 * instance for each associated Camera.
 */
IPAInterface *ipaCreate()
{
	return new ipa::ipu3::IPAIPU3();
}
}

} /* namespace libcamera */
