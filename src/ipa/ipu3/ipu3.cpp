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

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/request.h>

#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/af.h"
#include "algorithms/agc.h"
#include "algorithms/algorithm.h"
#include "algorithms/awb.h"
#include "algorithms/blc.h"
#include "algorithms/tone_mapping.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

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

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

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
 * We extend the IPAIPU3Interface to implement our algorithms and handle
 * calls from the IPU3 PipelineHandler to satisfy requests from the
 * application.
 *
 * At initialisation time, a CameraSensorHelper is instantiated to support
 * camera-specific calculations, while the default controls are computed, and
 * the algorithms are instantiated from the tuning data file.
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
 * In fillParamsBuffer(), we populate the ImgU parameter buffer with
 * settings to configure the device in preparation for handling the frame
 * queued in the Request.
 *
 * When the frame has completed processing, the ImgU will generate a statistics
 * buffer which is given to the IPA with processStatsBuffer(). In this we run the
 * algorithms to parse the statistics and cache any results for the next
 * fillParamsBuffer() call.
 *
 * The individual algorithms are split into modular components that are called
 * iteratively to allow them to process statistics from the ImgU in the order
 * defined in the tuning data file.
 *
 * The current implementation supports five core algorithms:
 *
 * - Auto focus (AF)
 * - Automatic gain and exposure control (AGC)
 * - Automatic white balance (AWB)
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
class IPAIPU3 : public IPAIPU3Interface, public Module
{
public:
	IPAIPU3();

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

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void fillParamsBuffer(const uint32_t frame, const uint32_t bufferId) override;
	void processStatsBuffer(const uint32_t frame, const int64_t frameTimestamp,
				const uint32_t bufferId,
				const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void updateSessionConfiguration(const ControlInfoMap &sensorControls);

	void setControls(unsigned int frame);
	void calculateBdsGrid(const Size &bdsOutputSize);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap sensorCtrls_;
	ControlInfoMap lensCtrls_;

	IPACameraSensorInfo sensorInfo_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;
};

IPAIPU3::IPAIPU3()
	: context_({ {}, {}, { kMaxFrameContexts } })
{
}

std::string IPAIPU3::logPrefix() const
{
	return "ipu3";
}

/**
 * \brief Compute IPASessionConfiguration using the sensor information and the
 * sensor V4L2 controls
 */
void IPAIPU3::updateSessionConfiguration(const ControlInfoMap &sensorControls)
{
	const ControlInfo vBlank = sensorControls.find(V4L2_CID_VBLANK)->second;
	context_.configuration.sensor.defVBlank = vBlank.def().get<int32_t>();

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
	context_.configuration.agc.minShutterSpeed = minExposure * context_.configuration.sensor.lineDuration;
	context_.configuration.agc.maxShutterSpeed = maxExposure * context_.configuration.sensor.lineDuration;
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
	double lineDuration = context_.configuration.sensor.lineDuration.get<std::micro>();

	/*
	 * Compute exposure time limits by using line length and pixel rate
	 * converted to microseconds. Use the V4L2_CID_EXPOSURE control to get
	 * exposure min, max and default and convert it from lines to
	 * microseconds.
	 */
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
	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (camHelper_ == nullptr) {
		LOG(IPAIPU3, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	/* Clean context */
	context_.configuration = {};
	context_.configuration.sensor.lineDuration = sensorInfo.minLineLength
						   * 1.0s / sensorInfo.pixelRate;

	/* Load the tuning data file. */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPAIPU3, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	unsigned int version = (*data)["version"].get<uint32_t>(0);
	if (version != 1) {
		LOG(IPAIPU3, Error)
			<< "Invalid tuning file version " << version;
		return -EINVAL;
	}

	if (!data->contains("algorithms")) {
		LOG(IPAIPU3, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

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
	context_.frameContexts.clear();
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

	lensCtrls_ = configInfo.lensControls;

	/* Clear the IPA context for the new streaming session. */
	context_.activeState = {};
	context_.configuration = {};
	context_.frameContexts.clear();

	/* Initialise the sensor configuration. */
	context_.configuration.sensor.lineDuration = sensorInfo_.minLineLength
						   * 1.0s / sensorInfo_.pixelRate;
	context_.configuration.sensor.size = sensorInfo_.outputSize;

	/*
	 * Compute the sensor V4L2 controls to be used by the algorithms and
	 * to be set on the sensor.
	 */
	sensorCtrls_ = configInfo.sensorControls;

	calculateBdsGrid(configInfo.bdsOutputSize);

	/* Update the camera controls using the new sensor settings. */
	updateControls(sensorInfo_, sensorCtrls_, ipaControls);

	/* Update the IPASessionConfiguration using the sensor settings. */
	updateSessionConfiguration(sensorCtrls_);

	for (auto const &algo : algorithms()) {
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
 * \brief Fill and return a buffer with ISP processing parameters for a frame
 * \param[in] frame The frame number
 * \param[in] bufferId ID of the parameter buffer to fill
 *
 * Algorithms are expected to fill the IPU3 parameter buffer for the next
 * frame given their most recent processing of the ImgU statistics.
 */
void IPAIPU3::fillParamsBuffer(const uint32_t frame, const uint32_t bufferId)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPAIPU3, Error) << "Could not find param buffer!";
		return;
	}

	Span<uint8_t> mem = it->second.planes()[0];
	ipu3_uapi_params *params =
		reinterpret_cast<ipu3_uapi_params *>(mem.data());

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

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	for (auto const &algo : algorithms())
		algo->prepare(context_, frame, frameContext, params);

	paramsBufferReady.emit(frame);
}

/**
 * \brief Process the statistics generated by the ImgU
 * \param[in] frame The frame number
 * \param[in] frameTimestamp Timestamp of the frame
 * \param[in] bufferId ID of the statistics buffer
 * \param[in] sensorControls Sensor controls
 *
 * Parse the most recently processed image statistics from the ImgU. The
 * statistics are passed to each algorithm module to run their calculations and
 * update their state accordingly.
 */
void IPAIPU3::processStatsBuffer(const uint32_t frame,
				 [[maybe_unused]] const int64_t frameTimestamp,
				 const uint32_t bufferId, const ControlList &sensorControls)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPAIPU3, Error) << "Could not find stats buffer!";
		return;
	}

	Span<uint8_t> mem = it->second.planes()[0];
	const ipu3_uapi_stats_3a *stats =
		reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	frameContext.sensor.exposure = sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	frameContext.sensor.gain = camHelper_->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

	ControlList metadata(controls::controls);

	for (auto const &algo : algorithms())
		algo->process(context_, frame, frameContext, stats, metadata);

	setControls(frame);

	/*
	 * \todo The Metadata provides a path to getting extended data
	 * out to the application. Further data such as a simplifed Histogram
	 * might have value to be exposed, however such data may be
	 * difficult to report in a generically parsable way and we
	 * likely want to avoid putting platform specific metadata in.
	 */

	metadataReady.emit(frame, metadata);
}

/**
 * \brief Queue a request and process the control list from the application
 * \param[in] frame The number of the frame which will be processed next
 * \param[in] controls The controls for the \a frame
 *
 * Parse the request to handle any IPA-managed controls that were set from the
 * application such as manual sensor settings.
 */
void IPAIPU3::queueRequest(const uint32_t frame, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(frame);

	for (auto const &algo : algorithms())
		algo->queueRequest(context_, frame, frameContext, controls);
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
	int32_t exposure = context_.activeState.agc.exposure;
	int32_t gain = camHelper_->gainCode(context_.activeState.agc.gain);

	ControlList ctrls(sensorCtrls_);
	ctrls.set(V4L2_CID_EXPOSURE, exposure);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, gain);

	ControlList lensCtrls(lensCtrls_);
	lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE,
		      static_cast<int32_t>(context_.activeState.af.focus));

	setSensorControls.emit(frame, ctrls, lensCtrls);
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
