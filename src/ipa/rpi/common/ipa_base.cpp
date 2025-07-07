/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2023, Raspberry Pi Ltd
 *
 * Raspberry Pi IPA base class
 */

#include "ipa_base.h"

#include <cmath>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>

#include "controller/af_algorithm.h"
#include "controller/af_status.h"
#include "controller/agc_algorithm.h"
#include "controller/awb_algorithm.h"
#include "controller/awb_status.h"
#include "controller/black_level_status.h"
#include "controller/ccm_algorithm.h"
#include "controller/ccm_status.h"
#include "controller/contrast_algorithm.h"
#include "controller/denoise_algorithm.h"
#include "controller/hdr_algorithm.h"
#include "controller/lux_status.h"
#include "controller/sharpen_algorithm.h"
#include "controller/statistics.h"
#include "controller/sync_algorithm.h"
#include "controller/sync_status.h"

namespace libcamera {

using namespace std::literals::chrono_literals;
using utils::Duration;

namespace {

/* Number of frame length times to hold in the queue. */
constexpr unsigned int FrameLengthsQueueSize = 10;

/* Configure the sensor with these values initially. */
constexpr double defaultAnalogueGain = 1.0;
constexpr Duration defaultExposureTime = 20.0ms;
constexpr Duration defaultMinFrameDuration = 1.0s / 30.0;
constexpr Duration defaultMaxFrameDuration = 250.0s;

/*
 * Determine the minimum allowable inter-frame duration to run the controller
 * algorithms. If the pipeline handler provider frames at a rate higher than this,
 * we rate-limit the controller Prepare() and Process() calls to lower than or
 * equal to this rate.
 */
constexpr Duration controllerMinFrameDuration = 1.0s / 30.0;

/* List of controls handled by the Raspberry Pi IPA */
const ControlInfoMap::Map ipaControls{
	/* \todo Move this to the Camera class */
	{ &controls::AeEnable, ControlInfo(false, true, true) },
	{ &controls::ExposureTimeMode,
	  ControlInfo({ { ControlValue(controls::ExposureTimeModeAuto),
			  ControlValue(controls::ExposureTimeModeManual) } },
		      ControlValue(controls::ExposureTimeModeAuto)) },
	{ &controls::ExposureTime,
	  ControlInfo(1, 66666, static_cast<int32_t>(defaultExposureTime.get<std::micro>())) },
	{ &controls::AnalogueGainMode,
	  ControlInfo({ { ControlValue(controls::AnalogueGainModeAuto),
			  ControlValue(controls::AnalogueGainModeManual) } },
		      ControlValue(controls::AnalogueGainModeAuto)) },
	{ &controls::AnalogueGain, ControlInfo(1.0f, 16.0f, 1.0f) },
	{ &controls::AeMeteringMode, ControlInfo(controls::AeMeteringModeValues) },
	{ &controls::AeConstraintMode, ControlInfo(controls::AeConstraintModeValues) },
	{ &controls::AeExposureMode, ControlInfo(controls::AeExposureModeValues) },
	{ &controls::ExposureValue, ControlInfo(-8.0f, 8.0f, 0.0f) },
	{ &controls::AeFlickerMode,
	  ControlInfo({ { ControlValue(controls::FlickerOff),
			  ControlValue(controls::FlickerManual) } },
		      ControlValue(controls::FlickerOff)) },
	{ &controls::AeFlickerPeriod, ControlInfo(100, 1000000) },
	{ &controls::Brightness, ControlInfo(-1.0f, 1.0f, 0.0f) },
	{ &controls::Contrast, ControlInfo(0.0f, 32.0f, 1.0f) },
	{ &controls::HdrMode, ControlInfo(controls::HdrModeValues) },
	{ &controls::Sharpness, ControlInfo(0.0f, 16.0f, 1.0f) },
	{ &controls::ScalerCrop, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) },
	{ &controls::FrameDurationLimits,
	  ControlInfo(INT64_C(33333), INT64_C(120000),
		      static_cast<int64_t>(defaultMinFrameDuration.get<std::micro>())) },
	{ &controls::rpi::SyncMode, ControlInfo(controls::rpi::SyncModeValues) },
	{ &controls::rpi::SyncFrames, ControlInfo(1, 1000000, 100) },
	{ &controls::draft::NoiseReductionMode, ControlInfo(controls::draft::NoiseReductionModeValues) },
	{ &controls::rpi::StatsOutputEnable, ControlInfo(false, true, false) },
	{ &controls::rpi::CnnEnableInputTensor, ControlInfo(false, true, false) },
};

/* IPA controls handled conditionally, if the sensor is not mono */
const ControlInfoMap::Map ipaColourControls{
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::AwbMode, ControlInfo(controls::AwbModeValues) },
	{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
	{ &controls::ColourTemperature, ControlInfo(100, 100000) },
	{ &controls::Saturation, ControlInfo(0.0f, 32.0f, 1.0f) },
};

/* IPA controls handled conditionally, if the lens has a focus control */
const ControlInfoMap::Map ipaAfControls{
	{ &controls::AfMode, ControlInfo(controls::AfModeValues) },
	{ &controls::AfRange, ControlInfo(controls::AfRangeValues) },
	{ &controls::AfSpeed, ControlInfo(controls::AfSpeedValues) },
	{ &controls::AfMetering, ControlInfo(controls::AfMeteringValues) },
	{ &controls::AfWindows, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) },
	{ &controls::AfTrigger, ControlInfo(controls::AfTriggerValues) },
	{ &controls::AfPause, ControlInfo(controls::AfPauseValues) },
	{ &controls::LensPosition, ControlInfo(0.0f, 32.0f, 1.0f) }
};

/* Platform specific controls */
const std::map<const std::string, ControlInfoMap::Map> platformControls {
	{ "pisp", {
		{ &controls::rpi::ScalerCrops, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) }
	} },
};

} /* namespace */

LOG_DEFINE_CATEGORY(IPARPI)

namespace ipa::RPi {

IpaBase::IpaBase()
	: controller_(), frameLengths_(FrameLengthsQueueSize, 0s), statsMetadataOutput_(false),
	  stitchSwapBuffers_(false), frameCount_(0), mistrustCount_(0), lastRunTimestamp_(0),
	  firstStart_(true), flickerState_({ 0, 0s }), cnnEnableInputTensor_(false)
{
}

IpaBase::~IpaBase()
{
}

int32_t IpaBase::init(const IPASettings &settings, const InitParams &params, InitResult *result)
{
	/*
	 * Load the "helper" for this sensor. This tells us all the device specific stuff
	 * that the kernel driver doesn't. We only do this the first time; we don't need
	 * to re-parse the metadata after a simple mode-switch for no reason.
	 */
	helper_ = std::unique_ptr<RPiController::CamHelper>(RPiController::CamHelper::create(settings.sensorModel));
	if (!helper_) {
		LOG(IPARPI, Error) << "Could not create camera helper for "
				   << settings.sensorModel;
		return -EINVAL;
	}

	/* Pass out the sensor metadata to the pipeline handler */
	int sensorMetadata = helper_->sensorEmbeddedDataPresent();
	result->sensorConfig.sensorMetadata = sensorMetadata;

	/* Load the tuning file for this sensor. */
	int ret = controller_.read(settings.configurationFile.c_str());
	if (ret) {
		LOG(IPARPI, Error)
			<< "Failed to load tuning data file "
			<< settings.configurationFile;
		return ret;
	}

	lensPresent_ = params.lensPresent;

	controller_.initialise();
	helper_->setHwConfig(controller_.getHardwareConfig());

	/* Return the controls handled by the IPA */
	ControlInfoMap::Map ctrlMap = ipaControls;
	if (lensPresent_)
		ctrlMap.merge(ControlInfoMap::Map(ipaAfControls));

	auto platformCtrlsIt = platformControls.find(controller_.getTarget());
	if (platformCtrlsIt != platformControls.end())
		ctrlMap.merge(ControlInfoMap::Map(platformCtrlsIt->second));

	monoSensor_ = params.sensorInfo.cfaPattern == properties::draft::ColorFilterArrangementEnum::MONO;
	if (!monoSensor_)
		ctrlMap.merge(ControlInfoMap::Map(ipaColourControls));

	result->controlInfo = ControlInfoMap(std::move(ctrlMap), controls::controls);

	return platformInit(params, result);
}

int32_t IpaBase::configure(const IPACameraSensorInfo &sensorInfo, const ConfigParams &params,
			   ConfigResult *result)
{
	sensorCtrls_ = params.sensorControls;

	if (!validateSensorControls()) {
		LOG(IPARPI, Error) << "Sensor control validation failed.";
		return -1;
	}

	if (lensPresent_) {
		lensCtrls_ = params.lensControls;
		if (!validateLensControls()) {
			LOG(IPARPI, Warning) << "Lens validation failed, "
					     << "no lens control will be available.";
			lensPresent_ = false;
		}
	}

	/* Setup a metadata ControlList to output metadata. */
	libcameraMetadata_ = ControlList(controls::controls);

	/* Re-assemble camera mode using the sensor info. */
	setMode(sensorInfo);

	mode_.transform = static_cast<libcamera::Transform>(params.transform);

	/* Pass the camera mode to the CamHelper to setup algorithms. */
	helper_->setCameraMode(mode_);

	/*
	 * Initialise this ControlList correctly, even if empty, in case the IPA is
	 * running is isolation mode (passing the ControlList through the IPC layer).
	 */
	ControlList ctrls(sensorCtrls_);

	/* The pipeline handler passes out the mode's sensitivity. */
	result->modeSensitivity = mode_.sensitivity;

	if (firstStart_) {
		/* Supply initial values for frame durations. */
		applyFrameDurations(defaultMinFrameDuration, defaultMaxFrameDuration);

		/* Supply initial values for gain and exposure. */
		AgcStatus agcStatus;
		agcStatus.exposureTime = defaultExposureTime;
		agcStatus.analogueGain = defaultAnalogueGain;
		applyAGC(&agcStatus, ctrls);

	}

	result->sensorControls = std::move(ctrls);

	/*
	 * Apply the correct limits to the exposure, gain and frame duration controls
	 * based on the current sensor mode.
	 */
	ControlInfoMap::Map ctrlMap = ipaControls;
	ctrlMap[&controls::FrameDurationLimits] =
		ControlInfo(static_cast<int64_t>(mode_.minFrameDuration.get<std::micro>()),
			    static_cast<int64_t>(mode_.maxFrameDuration.get<std::micro>()),
			    static_cast<int64_t>(defaultMinFrameDuration.get<std::micro>()));

	ctrlMap[&controls::AnalogueGain] =
		ControlInfo(static_cast<float>(mode_.minAnalogueGain),
			    static_cast<float>(mode_.maxAnalogueGain),
			    static_cast<float>(defaultAnalogueGain));

	ctrlMap[&controls::ExposureTime] =
		ControlInfo(static_cast<int32_t>(mode_.minExposureTime.get<std::micro>()),
			    static_cast<int32_t>(mode_.maxExposureTime.get<std::micro>()),
			    static_cast<int32_t>(defaultExposureTime.get<std::micro>()));

	/* Declare colour processing related controls for non-mono sensors. */
	if (!monoSensor_)
		ctrlMap.merge(ControlInfoMap::Map(ipaColourControls));

	/* Declare Autofocus controls, only if we have a controllable lens */
	if (lensPresent_) {
		ctrlMap.merge(ControlInfoMap::Map(ipaAfControls));
		RPiController::AfAlgorithm *af =
			dynamic_cast<RPiController::AfAlgorithm *>(controller_.getAlgorithm("af"));
		if (af) {
			double min, max, dflt;
			af->getLensLimits(min, max);
			dflt = af->getDefaultLensPosition();
			ctrlMap[&controls::LensPosition] =
				ControlInfo(static_cast<float>(min),
					    static_cast<float>(max),
					    static_cast<float>(dflt));
		}
	}

	result->controlInfo = ControlInfoMap(std::move(ctrlMap), controls::controls);

	return platformConfigure(params, result);
}

void IpaBase::start(const ControlList &controls, StartResult *result)
{
	RPiController::Metadata metadata;

	if (!controls.empty()) {
		/* We have been given some controls to action before start. */
		applyControls(controls);
	}

	controller_.switchMode(mode_, &metadata);

	/* Reset the frame lengths queue state. */
	lastTimeout_ = 0s;
	frameLengths_.clear();
	frameLengths_.resize(FrameLengthsQueueSize, 0s);

	/* SwitchMode may supply updated exposure/gain values to use. */
	AgcStatus agcStatus;
	agcStatus.exposureTime = 0.0s;
	agcStatus.analogueGain = 0.0;

	metadata.get("agc.status", agcStatus);
	if (agcStatus.exposureTime && agcStatus.analogueGain) {
		ControlList ctrls(sensorCtrls_);
		applyAGC(&agcStatus, ctrls);
		result->controls = std::move(ctrls);
		setCameraTimeoutValue();
	}
	/* Make a note of this as it tells us the HDR status of the first few frames. */
	hdrStatus_ = agcStatus.hdr;

	/*
	 * AF: If no lens position was specified, drive lens to a default position.
	 * This had to be deferred (not initialised by a constructor) until here
	 * to ensure that exactly ONE starting position is sent to the lens driver.
	 * It should be the static API default, not dependent on AF range or mode.
	 */
	if (firstStart_ && lensPresent_) {
		RPiController::AfAlgorithm *af = dynamic_cast<RPiController::AfAlgorithm *>(
			controller_.getAlgorithm("af"));
		if (af && !af->getLensPosition()) {
			int32_t hwpos;
			double pos = af->getDefaultLensPosition();
			if (af->setLensPosition(pos, &hwpos, true)) {
				ControlList lensCtrls(lensCtrls_);
				lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE, hwpos);
				setLensControls.emit(lensCtrls);
			}
		}
	}

	/*
	 * Initialise frame counts, and decide how many frames must be hidden or
	 * "mistrusted", which depends on whether this is a startup from cold,
	 * or merely a mode switch in a running system.
	 */
	unsigned int agcConvergenceFrames = 0, awbConvergenceFrames = 0;
	frameCount_ = 0;
	if (firstStart_) {
		invalidCount_ = helper_->hideFramesStartup();
		mistrustCount_ = helper_->mistrustFramesStartup();

		/*
		 * Query the AGC/AWB for how many frames they may take to
		 * converge sufficiently. Where these numbers are non-zero
		 * we must allow for the frames with bad statistics
		 * (mistrustCount_) that they won't see. But if zero (i.e.
		 * no convergence necessary), no frames need to be dropped.
		 */
		RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
			controller_.getAlgorithm("agc"));
		if (agc) {
			agcConvergenceFrames = agc->getConvergenceFrames();
			if (agcConvergenceFrames)
				agcConvergenceFrames += mistrustCount_;
		}

		RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
			controller_.getAlgorithm("awb"));
		if (awb) {
			awbConvergenceFrames = awb->getConvergenceFrames();
			if (awbConvergenceFrames)
				awbConvergenceFrames += mistrustCount_;
		}
	} else {
		invalidCount_ = helper_->hideFramesModeSwitch();
		mistrustCount_ = helper_->mistrustFramesModeSwitch();
	}

	result->startupFrameCount = std::max({ agcConvergenceFrames, awbConvergenceFrames });
	result->invalidFrameCount = invalidCount_;

	invalidCount_ = std::max({ invalidCount_, agcConvergenceFrames, awbConvergenceFrames });

	LOG(IPARPI, Debug) << "Startup frames: " << result->startupFrameCount
			   << " Invalid frames: " << result->invalidFrameCount;

	firstStart_ = false;
	lastRunTimestamp_ = 0;

	platformStart(controls, result);
}

void IpaBase::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

void IpaBase::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(id);
	}
}

void IpaBase::prepareIsp(const PrepareParams &params)
{
	applyControls(params.requestControls);

	/*
	 * At start-up, or after a mode-switch, we may want to
	 * avoid running the control algos for a few frames in case
	 * they are "unreliable".
	 */
	int64_t frameTimestamp = params.sensorControls.get(controls::SensorTimestamp).value_or(0);
	unsigned int ipaContext = params.ipaContext % rpiMetadata_.size();
	RPiController::Metadata &rpiMetadata = rpiMetadata_[ipaContext];
	Span<uint8_t> embeddedBuffer;

	rpiMetadata.clear();
	fillDeviceStatus(params.sensorControls, ipaContext);
	fillSyncParams(params, ipaContext);

	if (params.buffers.embedded) {
		/*
		 * Pipeline handler has supplied us with an embedded data buffer,
		 * we must pass it to the CamHelper for parsing.
		 */
		auto it = buffers_.find(params.buffers.embedded);
		ASSERT(it != buffers_.end());
		embeddedBuffer = it->second.planes()[0];
	}

	/*
	 * AGC wants to know the algorithm status from the time it actioned the
	 * sensor exposure/gain changes. So fetch it from the metadata list
	 * indexed by the IPA cookie returned, and put it in the current frame
	 * metadata.
	 *
	 * Note if the HDR mode has changed, as things like tonemaps may need updating.
	 */
	AgcStatus agcStatus;
	bool hdrChange = false;
	RPiController::Metadata &delayedMetadata = rpiMetadata_[params.delayContext];
	if (!delayedMetadata.get<AgcStatus>("agc.status", agcStatus)) {
		rpiMetadata.set("agc.delayed_status", agcStatus);
		hdrChange = agcStatus.hdr.mode != hdrStatus_.mode;
		hdrStatus_ = agcStatus.hdr;
	}

	/*
	 * This may overwrite the DeviceStatus using values from the sensor
	 * metadata, and may also do additional custom processing.
	 */
	helper_->prepare(embeddedBuffer, rpiMetadata);

	/* Allow a 10% margin on the comparison below. */
	Duration delta = (frameTimestamp - lastRunTimestamp_) * 1.0ns;
	if (lastRunTimestamp_ && frameCount_ > invalidCount_ &&
	    delta < controllerMinFrameDuration * 0.9 && !hdrChange) {
		/*
		 * Ensure we merge the previous frame's metadata with the current
		 * frame. This will not overwrite exposure/gain values for the
		 * current frame, or any other bits of metadata that were added
		 * in helper_->Prepare().
		 */
		RPiController::Metadata &lastMetadata =
			rpiMetadata_[(ipaContext ? ipaContext : rpiMetadata_.size()) - 1];
		rpiMetadata.mergeCopy(lastMetadata);
		processPending_ = false;
	} else {
		processPending_ = true;
		lastRunTimestamp_ = frameTimestamp;
	}

	/*
	 * If the statistics are inline (i.e. already available with the Bayer
	 * frame), call processStats() now before prepare().
	 */
	if (controller_.getHardwareConfig().statsInline)
		processStats({ params.buffers, params.ipaContext });

	/* Do we need/want to call prepare? */
	if (processPending_) {
		controller_.prepare(&rpiMetadata);
		/* Actually prepare the ISP parameters for the frame. */
		platformPrepareIsp(params, rpiMetadata);
	}

	frameCount_++;

	/* If the statistics are inline the metadata can be returned early. */
	if (controller_.getHardwareConfig().statsInline)
		reportMetadata(ipaContext);

	/* Ready to push the input buffer into the ISP. */
	prepareIspComplete.emit(params.buffers, stitchSwapBuffers_);
}

void IpaBase::processStats(const ProcessParams &params)
{
	unsigned int ipaContext = params.ipaContext % rpiMetadata_.size();

	if (processPending_ && frameCount_ >= mistrustCount_) {
		RPiController::Metadata &rpiMetadata = rpiMetadata_[ipaContext];

		auto it = buffers_.find(params.buffers.stats);
		if (it == buffers_.end()) {
			LOG(IPARPI, Error) << "Could not find stats buffer!";
			return;
		}

		RPiController::StatisticsPtr statistics = platformProcessStats(it->second.planes()[0]);

		/* reportMetadata() will pick this up and set the FocusFoM metadata */
		rpiMetadata.set("focus.status", statistics->focusRegions);

		helper_->process(statistics, rpiMetadata);
		controller_.process(statistics, &rpiMetadata);

		/* Send any sync algorithm outputs back to the pipeline handler */
		Duration offset(0s);
		struct SyncStatus syncStatus;
		if (rpiMetadata.get("sync.status", syncStatus) == 0) {
			if (minFrameDuration_ != maxFrameDuration_)
				LOG(IPARPI, Error) << "Sync algorithm enabled with variable framerate. "
						   << minFrameDuration_ << " " << maxFrameDuration_;
			offset = syncStatus.frameDurationOffset;

			libcameraMetadata_.set(controls::rpi::SyncReady, syncStatus.ready);
			if (syncStatus.timerKnown)
				libcameraMetadata_.set(controls::rpi::SyncTimer, syncStatus.timerValue);
		}

		struct AgcStatus agcStatus;
		if (rpiMetadata.get("agc.status", agcStatus) == 0) {
			ControlList ctrls(sensorCtrls_);
			applyAGC(&agcStatus, ctrls, offset);
			setDelayedControls.emit(ctrls, ipaContext);
			setCameraTimeoutValue();
		}
	}

	/*
	 * If the statistics are not inline the metadata must be returned now,
	 * before the processStatsComplete signal.
	 */
	if (!controller_.getHardwareConfig().statsInline)
		reportMetadata(ipaContext);

	processStatsComplete.emit(params.buffers);
}

void IpaBase::setMode(const IPACameraSensorInfo &sensorInfo)
{
	mode_.bitdepth = sensorInfo.bitsPerPixel;
	mode_.width = sensorInfo.outputSize.width;
	mode_.height = sensorInfo.outputSize.height;
	mode_.sensorWidth = sensorInfo.activeAreaSize.width;
	mode_.sensorHeight = sensorInfo.activeAreaSize.height;
	mode_.cropX = sensorInfo.analogCrop.x;
	mode_.cropY = sensorInfo.analogCrop.y;
	mode_.pixelRate = sensorInfo.pixelRate;

	/*
	 * Calculate scaling parameters. The scale_[xy] factors are determined
	 * by the ratio between the crop rectangle size and the output size.
	 */
	mode_.scaleX = sensorInfo.analogCrop.width / sensorInfo.outputSize.width;
	mode_.scaleY = sensorInfo.analogCrop.height / sensorInfo.outputSize.height;

	/*
	 * We're not told by the pipeline handler how scaling is split between
	 * binning and digital scaling. For now, as a heuristic, assume that
	 * downscaling up to 2 is achieved through binning, and that any
	 * additional scaling is achieved through digital scaling.
	 *
	 * \todo Get the pipeline handle to provide the full data
	 */
	mode_.binX = std::min(2, static_cast<int>(mode_.scaleX));
	mode_.binY = std::min(2, static_cast<int>(mode_.scaleY));

	/* The noise factor is the square root of the total binning factor. */
	mode_.noiseFactor = std::sqrt(mode_.binX * mode_.binY);

	/*
	 * Calculate the line length as the ratio between the line length in
	 * pixels and the pixel rate.
	 */
	mode_.minLineLength = sensorInfo.minLineLength * (1.0s / sensorInfo.pixelRate);
	mode_.maxLineLength = sensorInfo.maxLineLength * (1.0s / sensorInfo.pixelRate);

	/*
	 * Ensure that the maximum pixel processing rate does not exceed the ISP
	 * hardware capabilities. If it does, try adjusting the minimum line
	 * length to compensate if possible.
	 */
	Duration minPixelTime = controller_.getHardwareConfig().minPixelProcessingTime;
	Duration pixelTime = mode_.minLineLength / mode_.width;
	if (minPixelTime && pixelTime < minPixelTime) {
		Duration adjustedLineLength = minPixelTime * mode_.width;
		if (adjustedLineLength <= mode_.maxLineLength) {
			LOG(IPARPI, Info)
				<< "Adjusting mode minimum line length from " << mode_.minLineLength
				<< " to " << adjustedLineLength << " because of ISP constraints.";
			mode_.minLineLength = adjustedLineLength;
		} else {
			LOG(IPARPI, Error)
				<< "Sensor minimum line length of " << pixelTime * mode_.width
				<< " (" << 1us / pixelTime << " MPix/s)"
				<< " is below the minimum allowable ISP limit of "
				<< adjustedLineLength
				<< " (" << 1us / minPixelTime << " MPix/s) ";
			LOG(IPARPI, Error)
				<< "THIS WILL CAUSE IMAGE CORRUPTION!!! "
				<< "Please update the camera sensor driver to allow more horizontal blanking control.";
		}
	}

	/*
	 * Set the frame length limits for the mode to ensure exposure and
	 * framerate calculations are clipped appropriately.
	 */
	mode_.minFrameLength = sensorInfo.minFrameLength;
	mode_.maxFrameLength = sensorInfo.maxFrameLength;

	/* Store these for convenience. */
	mode_.minFrameDuration = mode_.minFrameLength * mode_.minLineLength;
	mode_.maxFrameDuration = mode_.maxFrameLength * mode_.maxLineLength;

	/*
	 * Some sensors may have different sensitivities in different modes;
	 * the CamHelper will know the correct value.
	 */
	mode_.sensitivity = helper_->getModeSensitivity(mode_);

	const ControlInfo &gainCtrl = sensorCtrls_.at(V4L2_CID_ANALOGUE_GAIN);
	const ControlInfo &exposureTimeCtrl = sensorCtrls_.at(V4L2_CID_EXPOSURE);

	mode_.minAnalogueGain = helper_->gain(gainCtrl.min().get<int32_t>());
	mode_.maxAnalogueGain = helper_->gain(gainCtrl.max().get<int32_t>());

	/*
	 * We need to give the helper the min/max frame durations so it can calculate
	 * the correct exposure limits below.
	 */
	helper_->setCameraMode(mode_);

	/*
	 * Exposure time is calculated based on the limits of the frame
	 * durations.
	 */
	mode_.minExposureTime = helper_->exposure(exposureTimeCtrl.min().get<int32_t>(),
						  mode_.minLineLength);
	mode_.maxExposureTime = Duration::max();
	helper_->getBlanking(mode_.maxExposureTime, mode_.minFrameDuration,
			     mode_.maxFrameDuration);
}

void IpaBase::setCameraTimeoutValue()
{
	/*
	 * Take the maximum value of the exposure queue as the camera timeout
	 * value to pass back to the pipeline handler. Only signal if it has changed
	 * from the last set value.
	 */
	auto max = std::max_element(frameLengths_.begin(), frameLengths_.end());

	if (*max != lastTimeout_) {
		setCameraTimeout.emit(max->get<std::milli>());
		lastTimeout_ = *max;
	}
}

bool IpaBase::validateSensorControls()
{
	static const uint32_t ctrls[] = {
		V4L2_CID_ANALOGUE_GAIN,
		V4L2_CID_EXPOSURE,
		V4L2_CID_VBLANK,
		V4L2_CID_HBLANK,
	};

	for (auto c : ctrls) {
		if (sensorCtrls_.find(c) == sensorCtrls_.end()) {
			LOG(IPARPI, Error) << "Unable to find sensor control "
					   << utils::hex(c);
			return false;
		}
	}

	return true;
}

bool IpaBase::validateLensControls()
{
	if (lensCtrls_.find(V4L2_CID_FOCUS_ABSOLUTE) == lensCtrls_.end()) {
		LOG(IPARPI, Error) << "Unable to find Lens control V4L2_CID_FOCUS_ABSOLUTE";
		return false;
	}

	return true;
}

/*
 * Converting between enums (used in the libcamera API) and the names that
 * we use to identify different modes. Unfortunately, the conversion tables
 * must be kept up-to-date by hand.
 */
static const std::map<int32_t, std::string> MeteringModeTable = {
	{ controls::MeteringCentreWeighted, "centre-weighted" },
	{ controls::MeteringSpot, "spot" },
	{ controls::MeteringMatrix, "matrix" },
	{ controls::MeteringCustom, "custom" },
};

static const std::map<int32_t, std::string> ConstraintModeTable = {
	{ controls::ConstraintNormal, "normal" },
	{ controls::ConstraintHighlight, "highlight" },
	{ controls::ConstraintShadows, "shadows" },
	{ controls::ConstraintCustom, "custom" },
};

static const std::map<int32_t, std::string> ExposureModeTable = {
	{ controls::ExposureNormal, "normal" },
	{ controls::ExposureShort, "short" },
	{ controls::ExposureLong, "long" },
	{ controls::ExposureCustom, "custom" },
};

static const std::map<int32_t, std::string> AwbModeTable = {
	{ controls::AwbAuto, "auto" },
	{ controls::AwbIncandescent, "incandescent" },
	{ controls::AwbTungsten, "tungsten" },
	{ controls::AwbFluorescent, "fluorescent" },
	{ controls::AwbIndoor, "indoor" },
	{ controls::AwbDaylight, "daylight" },
	{ controls::AwbCloudy, "cloudy" },
	{ controls::AwbCustom, "custom" },
};

static const std::map<int32_t, RPiController::AfAlgorithm::AfMode> AfModeTable = {
	{ controls::AfModeManual, RPiController::AfAlgorithm::AfModeManual },
	{ controls::AfModeAuto, RPiController::AfAlgorithm::AfModeAuto },
	{ controls::AfModeContinuous, RPiController::AfAlgorithm::AfModeContinuous },
};

static const std::map<int32_t, RPiController::AfAlgorithm::AfRange> AfRangeTable = {
	{ controls::AfRangeNormal, RPiController::AfAlgorithm::AfRangeNormal },
	{ controls::AfRangeMacro, RPiController::AfAlgorithm::AfRangeMacro },
	{ controls::AfRangeFull, RPiController::AfAlgorithm::AfRangeFull },
};

static const std::map<int32_t, RPiController::AfAlgorithm::AfPause> AfPauseTable = {
	{ controls::AfPauseImmediate, RPiController::AfAlgorithm::AfPauseImmediate },
	{ controls::AfPauseDeferred, RPiController::AfAlgorithm::AfPauseDeferred },
	{ controls::AfPauseResume, RPiController::AfAlgorithm::AfPauseResume },
};

static const std::map<int32_t, std::string> HdrModeTable = {
	{ controls::HdrModeOff, "Off" },
	{ controls::HdrModeMultiExposureUnmerged, "MultiExposureUnmerged" },
	{ controls::HdrModeMultiExposure, "MultiExposure" },
	{ controls::HdrModeSingleExposure, "SingleExposure" },
	{ controls::HdrModeNight, "Night" },
};

void IpaBase::applyControls(const ControlList &controls)
{
	using RPiController::AgcAlgorithm;
	using RPiController::AfAlgorithm;
	using RPiController::ContrastAlgorithm;
	using RPiController::DenoiseAlgorithm;
	using RPiController::HdrAlgorithm;
	using RPiController::SyncAlgorithm;

	/* Clear the return metadata buffer. */
	libcameraMetadata_.clear();

	/* Because some AF controls are mode-specific, handle AF mode change first. */
	if (controls.contains(controls::AF_MODE)) {
		AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
		if (!af) {
			LOG(IPARPI, Warning)
				<< "Could not set AF_MODE - no AF algorithm";
		}

		int32_t idx = controls.get(controls::AF_MODE).get<int32_t>();
		auto mode = AfModeTable.find(idx);
		if (mode == AfModeTable.end()) {
			LOG(IPARPI, Error) << "AF mode " << idx
					   << " not recognised";
		} else if (af)
			af->setMode(mode->second);
	}

	/*
	 * Because some AE controls are mode-specific, handle the AE-related
	 * mode changes first.
	 */
	const auto analogueGainMode = controls.get(controls::AnalogueGainMode);
	const auto exposureTimeMode = controls.get(controls::ExposureTimeMode);

	if (analogueGainMode || exposureTimeMode) {
		RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
			controller_.getAlgorithm("agc"));
		if (agc) {
			if (analogueGainMode) {
				if (*analogueGainMode == controls::AnalogueGainModeManual)
					agc->disableAutoGain();
				else
					agc->enableAutoGain();

				libcameraMetadata_.set(controls::AnalogueGainMode,
						       *analogueGainMode);
			}

			if (exposureTimeMode) {
				if (*exposureTimeMode == controls::ExposureTimeModeManual)
					agc->disableAutoExposure();
				else
					agc->enableAutoExposure();

				libcameraMetadata_.set(controls::ExposureTimeMode,
						       *exposureTimeMode);
			}
		} else {
			LOG(IPARPI, Warning)
				<< "Could not set AnalogueGainMode or ExposureTimeMode - no AGC algorithm";
		}
	}

	/* Iterate over controls */
	for (auto const &ctrl : controls) {
		LOG(IPARPI, Debug) << "Request ctrl: "
				   << controls::controls.at(ctrl.first)->name()
				   << " = " << ctrl.second.toString();

		switch (ctrl.first) {
		case controls::EXPOSURE_TIME_MODE:
			break; /* We already handled this one above */

		case controls::EXPOSURE_TIME: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set EXPOSURE_TIME - no AGC algorithm";
				break;
			}

			/*
			 * Ignore manual exposure time when the auto exposure
			 * algorithm is running.
			 */
			if (agc->autoExposureEnabled())
				break;

			/* The control provides units of microseconds. */
			agc->setFixedExposureTime(0, ctrl.second.get<int32_t>() * 1.0us);

			libcameraMetadata_.set(controls::ExposureTime, ctrl.second.get<int32_t>());
			break;
		}

		case controls::ANALOGUE_GAIN_MODE:
			break; /* We already handled this one above */

		case controls::ANALOGUE_GAIN: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set ANALOGUE_GAIN - no AGC algorithm";
				break;
			}

			/*
			 * Ignore manual analogue gain value when the auto gain
			 * algorithm is running.
			 */
			if (agc->autoGainEnabled())
				break;

			agc->setFixedAnalogueGain(0, ctrl.second.get<float>());

			libcameraMetadata_.set(controls::AnalogueGain,
					       ctrl.second.get<float>());
			break;
		}

		case controls::AE_METERING_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set AE_METERING_MODE - no AGC algorithm";
				break;
			}

			int32_t idx = ctrl.second.get<int32_t>();
			if (MeteringModeTable.count(idx)) {
				agc->setMeteringMode(MeteringModeTable.at(idx));
				libcameraMetadata_.set(controls::AeMeteringMode, idx);
			} else {
				LOG(IPARPI, Error) << "Metering mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::AE_CONSTRAINT_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set AE_CONSTRAINT_MODE - no AGC algorithm";
				break;
			}

			int32_t idx = ctrl.second.get<int32_t>();
			if (ConstraintModeTable.count(idx)) {
				agc->setConstraintMode(ConstraintModeTable.at(idx));
				libcameraMetadata_.set(controls::AeConstraintMode, idx);
			} else {
				LOG(IPARPI, Error) << "Constraint mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::AE_EXPOSURE_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set AE_EXPOSURE_MODE - no AGC algorithm";
				break;
			}

			/*
			 * Ignore AE_EXPOSURE_MODE if the shutter or the gain
			 * are in auto mode.
			 */
			if (agc->autoExposureEnabled() || agc->autoGainEnabled())
				break;

			int32_t idx = ctrl.second.get<int32_t>();
			if (ExposureModeTable.count(idx)) {
				agc->setExposureMode(ExposureModeTable.at(idx));
				libcameraMetadata_.set(controls::AeExposureMode, idx);
			} else {
				LOG(IPARPI, Error) << "Exposure mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::EXPOSURE_VALUE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set EXPOSURE_VALUE - no AGC algorithm";
				break;
			}

			/*
			 * The SetEv() function takes in a direct exposure multiplier.
			 * So convert to 2^EV
			 */
			double ev = pow(2.0, ctrl.second.get<float>());
			agc->setEv(0, ev);
			libcameraMetadata_.set(controls::ExposureValue,
					       ctrl.second.get<float>());
			break;
		}

		case controls::AE_ENABLE: {
			/*
			 * The AeEnable control is now just a wrapper that will already have been
			 * converted to ExposureTimeMode and AnalogueGainMode equivalents, so there
			 * would be nothing to do here. Nonetheless, "handle" the control so as to
			 * avoid warnings from the "default:" clause of the switch statement.
			 */

			break;
		}

		case controls::AE_FLICKER_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set AeFlickerMode - no AGC algorithm";
				break;
			}

			int32_t mode = ctrl.second.get<int32_t>();
			bool modeValid = true;

			switch (mode) {
			case controls::FlickerOff:
				agc->setFlickerPeriod(0us);

				break;

			case controls::FlickerManual:
				agc->setFlickerPeriod(flickerState_.manualPeriod);

				break;

			default:
				LOG(IPARPI, Error) << "Flicker mode " << mode << " is not supported";
				modeValid = false;

				break;
			}

			if (modeValid)
				flickerState_.mode = mode;

			break;
		}

		case controls::AE_FLICKER_PERIOD: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set AeFlickerPeriod - no AGC algorithm";
				break;
			}

			uint32_t manualPeriod = ctrl.second.get<int32_t>();
			flickerState_.manualPeriod = manualPeriod * 1.0us;

			/*
			 * We note that it makes no difference if the mode gets set to "manual"
			 * first, and the period updated after, or vice versa.
			 */
			if (flickerState_.mode == controls::FlickerManual)
				agc->setFlickerPeriod(flickerState_.manualPeriod);

			break;
		}

		case controls::AWB_ENABLE: {
			/* Silently ignore this control for a mono sensor. */
			if (monoSensor_)
				break;

			RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
				controller_.getAlgorithm("awb"));
			if (!awb) {
				LOG(IPARPI, Warning)
					<< "Could not set AWB_ENABLE - no AWB algorithm";
				break;
			}

			if (ctrl.second.get<bool>() == false)
				awb->disableAuto();
			else
				awb->enableAuto();

			libcameraMetadata_.set(controls::AwbEnable,
					       ctrl.second.get<bool>());
			break;
		}

		case controls::AWB_MODE: {
			/* Silently ignore this control for a mono sensor. */
			if (monoSensor_)
				break;

			RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
				controller_.getAlgorithm("awb"));
			if (!awb) {
				LOG(IPARPI, Warning)
					<< "Could not set AWB_MODE - no AWB algorithm";
				break;
			}

			int32_t idx = ctrl.second.get<int32_t>();
			if (AwbModeTable.count(idx)) {
				awb->setMode(AwbModeTable.at(idx));
				libcameraMetadata_.set(controls::AwbMode, idx);
			} else {
				LOG(IPARPI, Error) << "AWB mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::COLOUR_GAINS: {
			/* Silently ignore this control for a mono sensor. */
			if (monoSensor_)
				break;

			auto gains = ctrl.second.get<Span<const float>>();
			RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
				controller_.getAlgorithm("awb"));
			if (!awb) {
				LOG(IPARPI, Warning)
					<< "Could not set COLOUR_GAINS - no AWB algorithm";
				break;
			}

			awb->setManualGains(gains[0], gains[1]);
			if (gains[0] != 0.0f && gains[1] != 0.0f)
				/* A gain of 0.0f will switch back to auto mode. */
				libcameraMetadata_.set(controls::ColourGains,
						       { gains[0], gains[1] });
			break;
		}

		case controls::COLOUR_TEMPERATURE: {
			/* Silently ignore this control for a mono sensor. */
			if (monoSensor_)
				break;

			auto temperatureK = ctrl.second.get<int32_t>();
			RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
				controller_.getAlgorithm("awb"));
			if (!awb) {
				LOG(IPARPI, Warning)
					<< "Could not set COLOUR_TEMPERATURE - no AWB algorithm";
				break;
			}

			awb->setColourTemperature(temperatureK);
			/* This metadata will get reported back automatically. */
			break;
		}

		case controls::BRIGHTNESS: {
			RPiController::ContrastAlgorithm *contrast = dynamic_cast<RPiController::ContrastAlgorithm *>(
				controller_.getAlgorithm("contrast"));
			if (!contrast) {
				LOG(IPARPI, Warning)
					<< "Could not set BRIGHTNESS - no contrast algorithm";
				break;
			}

			contrast->setBrightness(ctrl.second.get<float>() * 65536);
			libcameraMetadata_.set(controls::Brightness,
					       ctrl.second.get<float>());
			break;
		}

		case controls::CONTRAST: {
			RPiController::ContrastAlgorithm *contrast = dynamic_cast<RPiController::ContrastAlgorithm *>(
				controller_.getAlgorithm("contrast"));
			if (!contrast) {
				LOG(IPARPI, Warning)
					<< "Could not set CONTRAST - no contrast algorithm";
				break;
			}

			contrast->setContrast(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Contrast,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SATURATION: {
			/* Silently ignore this control for a mono sensor. */
			if (monoSensor_)
				break;

			RPiController::CcmAlgorithm *ccm = dynamic_cast<RPiController::CcmAlgorithm *>(
				controller_.getAlgorithm("ccm"));
			if (!ccm) {
				LOG(IPARPI, Warning)
					<< "Could not set SATURATION - no ccm algorithm";
				break;
			}

			ccm->setSaturation(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Saturation,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SHARPNESS: {
			RPiController::SharpenAlgorithm *sharpen = dynamic_cast<RPiController::SharpenAlgorithm *>(
				controller_.getAlgorithm("sharpen"));
			if (!sharpen) {
				LOG(IPARPI, Warning)
					<< "Could not set SHARPNESS - no sharpen algorithm";
				break;
			}

			sharpen->setStrength(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Sharpness,
					       ctrl.second.get<float>());
			break;
		}

		case controls::rpi::SCALER_CROPS:
		case controls::SCALER_CROP: {
			/* We do nothing with this, but should avoid the warning below. */
			break;
		}

		case controls::FRAME_DURATION_LIMITS: {
			auto frameDurations = ctrl.second.get<Span<const int64_t>>();
			applyFrameDurations(frameDurations[0] * 1.0us, frameDurations[1] * 1.0us);
			break;
		}

		case controls::draft::NOISE_REDUCTION_MODE:
			/* Handled below in handleControls() */
			libcameraMetadata_.set(controls::draft::NoiseReductionMode,
					       ctrl.second.get<int32_t>());
			break;

		case controls::AF_MODE:
			break; /* We already handled this one above */

		case controls::AF_RANGE: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (!af) {
				LOG(IPARPI, Warning)
					<< "Could not set AF_RANGE - no focus algorithm";
				break;
			}

			auto range = AfRangeTable.find(ctrl.second.get<int32_t>());
			if (range == AfRangeTable.end()) {
				LOG(IPARPI, Error) << "AF range " << ctrl.second.get<int32_t>()
						   << " not recognised";
				break;
			}
			af->setRange(range->second);
			break;
		}

		case controls::AF_SPEED: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (!af) {
				LOG(IPARPI, Warning)
					<< "Could not set AF_SPEED - no focus algorithm";
				break;
			}

			AfAlgorithm::AfSpeed speed = ctrl.second.get<int32_t>() == controls::AfSpeedFast ?
						      AfAlgorithm::AfSpeedFast : AfAlgorithm::AfSpeedNormal;
			af->setSpeed(speed);
			break;
		}

		case controls::AF_METERING: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (!af) {
				LOG(IPARPI, Warning)
					<< "Could not set AF_METERING - no AF algorithm";
				break;
			}
			af->setMetering(ctrl.second.get<int32_t>() == controls::AfMeteringWindows);
			break;
		}

		case controls::AF_WINDOWS: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (!af) {
				LOG(IPARPI, Warning)
					<< "Could not set AF_WINDOWS - no AF algorithm";
				break;
			}
			af->setWindows(ctrl.second.get<Span<const Rectangle>>());
			break;
		}

		case controls::AF_PAUSE: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (!af || af->getMode() != AfAlgorithm::AfModeContinuous) {
				LOG(IPARPI, Warning)
					<< "Could not set AF_PAUSE - no AF algorithm or not Continuous";
				break;
			}
			auto pause = AfPauseTable.find(ctrl.second.get<int32_t>());
			if (pause == AfPauseTable.end()) {
				LOG(IPARPI, Error) << "AF pause " << ctrl.second.get<int32_t>()
						   << " not recognised";
				break;
			}
			af->pause(pause->second);
			break;
		}

		case controls::AF_TRIGGER: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (!af || af->getMode() != AfAlgorithm::AfModeAuto) {
				LOG(IPARPI, Warning)
					<< "Could not set AF_TRIGGER - no AF algorithm or not Auto";
				break;
			} else {
				if (ctrl.second.get<int32_t>() == controls::AfTriggerStart)
					af->triggerScan();
				else
					af->cancelScan();
			}
			break;
		}

		case controls::LENS_POSITION: {
			AfAlgorithm *af = dynamic_cast<AfAlgorithm *>(controller_.getAlgorithm("af"));
			if (af) {
				int32_t hwpos;
				if (af->setLensPosition(ctrl.second.get<float>(), &hwpos)) {
					ControlList lensCtrls(lensCtrls_);
					lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE, hwpos);
					setLensControls.emit(lensCtrls);
				}
			} else {
				LOG(IPARPI, Warning)
					<< "Could not set LENS_POSITION - no AF algorithm";
			}
			break;
		}

		case controls::HDR_MODE: {
			HdrAlgorithm *hdr = dynamic_cast<HdrAlgorithm *>(controller_.getAlgorithm("hdr"));
			if (!hdr) {
				LOG(IPARPI, Warning) << "No HDR algorithm available";
				break;
			}

			auto mode = HdrModeTable.find(ctrl.second.get<int32_t>());
			if (mode == HdrModeTable.end()) {
				LOG(IPARPI, Warning) << "Unrecognised HDR mode";
				break;
			}

			AgcAlgorithm *agc = dynamic_cast<AgcAlgorithm *>(controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning) << "HDR requires an AGC algorithm";
				break;
			}

			if (hdr->setMode(mode->second) == 0) {
				agc->setActiveChannels(hdr->getChannels());

				/* We also disable adpative contrast enhancement if HDR is running. */
				ContrastAlgorithm *contrast =
					dynamic_cast<ContrastAlgorithm *>(controller_.getAlgorithm("contrast"));
				if (contrast) {
					if (mode->second == "Off")
						contrast->restoreCe();
					else
						contrast->enableCe(false);
				}

				DenoiseAlgorithm *denoise =
					dynamic_cast<DenoiseAlgorithm *>(controller_.getAlgorithm("denoise"));
				if (denoise) {
					/* \todo - make the HDR mode say what denoise it wants? */
					if (mode->second == "Night")
						denoise->setConfig("night");
					else if (mode->second == "SingleExposure")
						denoise->setConfig("hdr");
					/* MultiExposure doesn't need extra extra denoise. */
					else
						denoise->setConfig("normal");
				}
			} else
				LOG(IPARPI, Warning)
					<< "HDR mode " << mode->second << " not supported";

			break;
		}

		case controls::rpi::STATS_OUTPUT_ENABLE:
			statsMetadataOutput_ = ctrl.second.get<bool>();
			break;

		case controls::rpi::CNN_ENABLE_INPUT_TENSOR:
			cnnEnableInputTensor_ = ctrl.second.get<bool>();
			break;

		case controls::rpi::SYNC_MODE: {
			SyncAlgorithm *sync = dynamic_cast<SyncAlgorithm *>(controller_.getAlgorithm("sync"));

			if (sync) {
				int mode = ctrl.second.get<int32_t>();
				SyncAlgorithm::Mode m = SyncAlgorithm::Mode::Off;
				if (mode == controls::rpi::SyncModeServer) {
					m = SyncAlgorithm::Mode::Server;
					LOG(IPARPI, Info) << "Sync mode set to server";
				} else if (mode == controls::rpi::SyncModeClient) {
					m = SyncAlgorithm::Mode::Client;
					LOG(IPARPI, Info) << "Sync mode set to client";
				}
				sync->setMode(m);
			}
			break;
		}

		case controls::rpi::SYNC_FRAMES: {
			SyncAlgorithm *sync = dynamic_cast<SyncAlgorithm *>(controller_.getAlgorithm("sync"));

			if (sync) {
				int frames = ctrl.second.get<int32_t>();
				if (frames > 0)
					sync->setReadyFrame(frames);
			}
			break;
		}

		default:
			LOG(IPARPI, Warning)
				<< "Ctrl " << controls::controls.at(ctrl.first)->name()
				<< " is not handled.";
			break;
		}
	}

	/* Give derived classes a chance to examine the new controls. */
	handleControls(controls);
}

void IpaBase::fillDeviceStatus(const ControlList &sensorControls, unsigned int ipaContext)
{
	DeviceStatus deviceStatus = {};

	int32_t exposureLines = sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	int32_t gainCode = sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();
	int32_t vblank = sensorControls.get(V4L2_CID_VBLANK).get<int32_t>();
	int32_t hblank = sensorControls.get(V4L2_CID_HBLANK).get<int32_t>();

	deviceStatus.lineLength = helper_->hblankToLineLength(hblank);
	deviceStatus.exposureTime = helper_->exposure(exposureLines, deviceStatus.lineLength);
	deviceStatus.analogueGain = helper_->gain(gainCode);
	deviceStatus.frameLength = mode_.height + vblank;

	RPiController::AfAlgorithm *af = dynamic_cast<RPiController::AfAlgorithm *>(
			controller_.getAlgorithm("af"));
	if (af)
		deviceStatus.lensPosition = af->getLensPosition();

	LOG(IPARPI, Debug) << "Metadata - " << deviceStatus;

	rpiMetadata_[ipaContext].set("device.status", deviceStatus);
}

void IpaBase::fillSyncParams(const PrepareParams &params, unsigned int ipaContext)
{
	RPiController::SyncAlgorithm *sync = dynamic_cast<RPiController::SyncAlgorithm *>(
		controller_.getAlgorithm("sync"));
	if (!sync)
		return;

	SyncParams syncParams;
	syncParams.wallClock = *params.sensorControls.get(controls::FrameWallClock);
	syncParams.sensorTimestamp = *params.sensorControls.get(controls::SensorTimestamp);
	rpiMetadata_[ipaContext].set("sync.params", syncParams);
}

void IpaBase::reportMetadata(unsigned int ipaContext)
{
	RPiController::Metadata &rpiMetadata = rpiMetadata_[ipaContext];
	std::unique_lock<RPiController::Metadata> lock(rpiMetadata);

	/*
	 * Certain information about the current frame and how it will be
	 * processed can be extracted and placed into the libcamera metadata
	 * buffer, where an application could query it.
	 */
	DeviceStatus *deviceStatus = rpiMetadata.getLocked<DeviceStatus>("device.status");
	if (deviceStatus) {
		libcameraMetadata_.set(controls::ExposureTime,
				       deviceStatus->exposureTime.get<std::micro>());
		libcameraMetadata_.set(controls::AnalogueGain, deviceStatus->analogueGain);
		libcameraMetadata_.set(controls::FrameDuration,
				       helper_->exposure(deviceStatus->frameLength, deviceStatus->lineLength).get<std::micro>());
		if (deviceStatus->sensorTemperature)
			libcameraMetadata_.set(controls::SensorTemperature, *deviceStatus->sensorTemperature);
		if (deviceStatus->lensPosition)
			libcameraMetadata_.set(controls::LensPosition, *deviceStatus->lensPosition);
	}

	AgcPrepareStatus *agcPrepareStatus = rpiMetadata.getLocked<AgcPrepareStatus>("agc.prepare_status");
	if (agcPrepareStatus)
		libcameraMetadata_.set(controls::DigitalGain, agcPrepareStatus->digitalGain);

	RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
		controller_.getAlgorithm("agc"));
	if (agc) {
		if (!agc->autoExposureEnabled() && !agc->autoGainEnabled())
			libcameraMetadata_.set(controls::AeState, controls::AeStateIdle);
		else if (agcPrepareStatus)
			libcameraMetadata_.set(controls::AeState,
					       agcPrepareStatus->locked
						       ? controls::AeStateConverged
						       : controls::AeStateSearching);
	}

	LuxStatus *luxStatus = rpiMetadata.getLocked<LuxStatus>("lux.status");
	if (luxStatus)
		libcameraMetadata_.set(controls::Lux, luxStatus->lux);

	AwbStatus *awbStatus = rpiMetadata.getLocked<AwbStatus>("awb.status");
	if (awbStatus) {
		libcameraMetadata_.set(controls::ColourGains, { static_cast<float>(awbStatus->gainR),
								static_cast<float>(awbStatus->gainB) });
		libcameraMetadata_.set(controls::ColourTemperature, awbStatus->temperatureK);
	}

	BlackLevelStatus *blackLevelStatus = rpiMetadata.getLocked<BlackLevelStatus>("black_level.status");
	if (blackLevelStatus)
		libcameraMetadata_.set(controls::SensorBlackLevels,
				       { static_cast<int32_t>(blackLevelStatus->blackLevelR),
					 static_cast<int32_t>(blackLevelStatus->blackLevelG),
					 static_cast<int32_t>(blackLevelStatus->blackLevelG),
					 static_cast<int32_t>(blackLevelStatus->blackLevelB) });

	RPiController::FocusRegions *focusStatus =
		rpiMetadata.getLocked<RPiController::FocusRegions>("focus.status");
	if (focusStatus) {
		/*
		 * Calculate the average FoM over the central (symmetric) positions
		 * to give an overall scene FoM. This can change later if it is
		 * not deemed suitable.
		 */
		libcamera::Size size = focusStatus->size();
		unsigned rows = size.height;
		unsigned cols = size.width;

		uint64_t sum = 0;
		unsigned int numRegions = 0;
		for (unsigned r = rows / 3; r < rows - rows / 3; ++r) {
			for (unsigned c = cols / 4; c < cols - cols / 4; ++c) {
				sum += focusStatus->get({ (int)c, (int)r }).val;
				numRegions++;
			}
		}

		uint32_t focusFoM = sum / numRegions;
		libcameraMetadata_.set(controls::FocusFoM, focusFoM);
	}

	CcmStatus *ccmStatus = rpiMetadata.getLocked<CcmStatus>("ccm.status");
	if (ccmStatus) {
		float m[9];
		for (unsigned int i = 0; i < 9; i++)
			m[i] = ccmStatus->matrix[i];
		libcameraMetadata_.set(controls::ColourCorrectionMatrix, m);
	}

	const AfStatus *afStatus = rpiMetadata.getLocked<AfStatus>("af.status");
	if (afStatus) {
		int32_t s, p;
		switch (afStatus->state) {
		case AfState::Scanning:
			s = controls::AfStateScanning;
			break;
		case AfState::Focused:
			s = controls::AfStateFocused;
			break;
		case AfState::Failed:
			s = controls::AfStateFailed;
			break;
		default:
			s = controls::AfStateIdle;
		}
		switch (afStatus->pauseState) {
		case AfPauseState::Pausing:
			p = controls::AfPauseStatePausing;
			break;
		case AfPauseState::Paused:
			p = controls::AfPauseStatePaused;
			break;
		default:
			p = controls::AfPauseStateRunning;
		}
		libcameraMetadata_.set(controls::AfState, s);
		libcameraMetadata_.set(controls::AfPauseState, p);
	}

	/*
	 * THe HDR algorithm sets the HDR channel into the agc.status at the time that those
	 * AGC parameters were calculated several frames ago, so it comes back to us now in
	 * the delayed_status. If this frame is too soon after a mode switch for the
	 * delayed_status to be available, we use the HDR status that came out of the
	 * switchMode call.
	 */
	const AgcStatus *agcStatus = rpiMetadata.getLocked<AgcStatus>("agc.delayed_status");
	const HdrStatus &hdrStatus = agcStatus ? agcStatus->hdr : hdrStatus_;
	if (!hdrStatus.mode.empty() && hdrStatus.mode != "Off") {
		int32_t hdrMode = controls::HdrModeOff;
		for (auto const &[mode, name] : HdrModeTable) {
			if (hdrStatus.mode == name) {
				hdrMode = mode;
				break;
			}
		}
		libcameraMetadata_.set(controls::HdrMode, hdrMode);

		if (hdrStatus.channel == "short")
			libcameraMetadata_.set(controls::HdrChannel, controls::HdrChannelShort);
		else if (hdrStatus.channel == "long")
			libcameraMetadata_.set(controls::HdrChannel, controls::HdrChannelLong);
		else if (hdrStatus.channel == "medium")
			libcameraMetadata_.set(controls::HdrChannel, controls::HdrChannelMedium);
		else
			libcameraMetadata_.set(controls::HdrChannel, controls::HdrChannelNone);
	}

	const std::shared_ptr<uint8_t[]> *inputTensor =
		rpiMetadata.getLocked<std::shared_ptr<uint8_t[]>>("cnn.input_tensor");
	if (cnnEnableInputTensor_ && inputTensor) {
		unsigned int size = *rpiMetadata.getLocked<unsigned int>("cnn.input_tensor_size");
		Span<const uint8_t> tensor{ inputTensor->get(), size };
		libcameraMetadata_.set(controls::rpi::CnnInputTensor, tensor);
		/* No need to keep these big buffers any more. */
		rpiMetadata.eraseLocked("cnn.input_tensor");
	}

	const RPiController::CnnInputTensorInfo *inputTensorInfo =
		rpiMetadata.getLocked<RPiController::CnnInputTensorInfo>("cnn.input_tensor_info");
	if (inputTensorInfo) {
		Span<const uint8_t> tensorInfo{ reinterpret_cast<const uint8_t *>(inputTensorInfo),
						sizeof(*inputTensorInfo) };
		libcameraMetadata_.set(controls::rpi::CnnInputTensorInfo, tensorInfo);
	}

	const std::shared_ptr<float[]> *outputTensor =
		rpiMetadata.getLocked<std::shared_ptr<float[]>>("cnn.output_tensor");
	if (outputTensor) {
		unsigned int size = *rpiMetadata.getLocked<unsigned int>("cnn.output_tensor_size");
		Span<const float> tensor{ reinterpret_cast<const float *>(outputTensor->get()),
					  size };
		libcameraMetadata_.set(controls::rpi::CnnOutputTensor, tensor);
		/* No need to keep these big buffers any more. */
		rpiMetadata.eraseLocked("cnn.output_tensor");
	}

	const RPiController::CnnOutputTensorInfo *outputTensorInfo =
		rpiMetadata.getLocked<RPiController::CnnOutputTensorInfo>("cnn.output_tensor_info");
	if (outputTensorInfo) {
		Span<const uint8_t> tensorInfo{ reinterpret_cast<const uint8_t *>(outputTensorInfo),
						sizeof(*outputTensorInfo) };
		libcameraMetadata_.set(controls::rpi::CnnOutputTensorInfo, tensorInfo);
	}

	const RPiController::CnnKpiInfo *kpiInfo =
		rpiMetadata.getLocked<RPiController::CnnKpiInfo>("cnn.kpi_info");
	if (kpiInfo) {
		libcameraMetadata_.set(controls::rpi::CnnKpiInfo,
				       { static_cast<int32_t>(kpiInfo->dnnRuntime),
					 static_cast<int32_t>(kpiInfo->dspRuntime) });
	}

	metadataReady.emit(libcameraMetadata_);
}

void IpaBase::applyFrameDurations(Duration minFrameDuration, Duration maxFrameDuration)
{
	/*
	 * This will only be applied once AGC recalculations occur.
	 * The values may be clamped based on the sensor mode capabilities as well.
	 */
	minFrameDuration_ = minFrameDuration ? minFrameDuration : defaultMinFrameDuration;
	maxFrameDuration_ = maxFrameDuration ? maxFrameDuration : defaultMaxFrameDuration;
	minFrameDuration_ = std::clamp(minFrameDuration_,
				       mode_.minFrameDuration, mode_.maxFrameDuration);
	maxFrameDuration_ = std::clamp(maxFrameDuration_,
				       mode_.minFrameDuration, mode_.maxFrameDuration);
	maxFrameDuration_ = std::max(maxFrameDuration_, minFrameDuration_);

	/* Return the validated limits via metadata. */
	libcameraMetadata_.set(controls::FrameDurationLimits,
			       { static_cast<int64_t>(minFrameDuration_.get<std::micro>()),
				 static_cast<int64_t>(maxFrameDuration_.get<std::micro>()) });

	/*
	 * Calculate the maximum exposure time possible for the AGC to use.
	 * getBlanking() will update maxExposureTime with the largest exposure
	 * value possible.
	 */
	Duration maxExposureTime = Duration::max();
	auto [vblank, hblank] = helper_->getBlanking(maxExposureTime, minFrameDuration_, maxFrameDuration_);

	RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
		controller_.getAlgorithm("agc"));
	if (agc)
		agc->setMaxExposureTime(maxExposureTime);

	RPiController::SyncAlgorithm *sync = dynamic_cast<RPiController::SyncAlgorithm *>(
		controller_.getAlgorithm("sync"));
	if (sync) {
		Duration duration = (mode_.height + vblank) * ((mode_.width + hblank) * 1.0s / mode_.pixelRate);
		LOG(IPARPI, Debug) << "setting sync frame duration to  " << duration;
		sync->setFrameDuration(duration);
	}
}

void IpaBase::applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls, Duration frameDurationOffset)
{
	const int32_t minGainCode = helper_->gainCode(mode_.minAnalogueGain);
	const int32_t maxGainCode = helper_->gainCode(mode_.maxAnalogueGain);
	int32_t gainCode = helper_->gainCode(agcStatus->analogueGain);

	/*
	 * Ensure anything larger than the max gain code will not be passed to
	 * DelayedControls. The AGC will correctly handle a lower gain returned
	 * by the sensor, provided it knows the actual gain used.
	 */
	gainCode = std::clamp<int32_t>(gainCode, minGainCode, maxGainCode);

	/* getBlanking might clip exposure time to the fps limits. */
	Duration exposure = agcStatus->exposureTime;
	auto [vblank, hblank] = helper_->getBlanking(exposure, minFrameDuration_ - frameDurationOffset,
						     maxFrameDuration_ - frameDurationOffset);
	int32_t exposureLines = helper_->exposureLines(exposure,
						       helper_->hblankToLineLength(hblank));

	LOG(IPARPI, Debug) << "Applying AGC Exposure: " << exposure
			   << " (Exposure lines: " << exposureLines << ", AGC requested "
			   << agcStatus->exposureTime << ") Gain: "
			   << agcStatus->analogueGain << " (Gain Code: "
			   << gainCode << ")";

	ctrls.set(V4L2_CID_VBLANK, static_cast<int32_t>(vblank));
	ctrls.set(V4L2_CID_EXPOSURE, exposureLines);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, gainCode);

	/*
	 * At present, there is no way of knowing if a control is read-only.
	 * As a workaround, assume that if the minimum and maximum values of
	 * the V4L2_CID_HBLANK control are the same, it implies the control
	 * is read-only. This seems to be the case for all the cameras our IPA
	 * works with.
	 *
	 * \todo The control API ought to have a flag to specify if a control
	 * is read-only which could be used below.
	 */
	if (mode_.minLineLength != mode_.maxLineLength)
		ctrls.set(V4L2_CID_HBLANK, static_cast<int32_t>(hblank));

	/*
	 * Store the frame length times in a circular queue, up-to FrameLengthsQueueSize
	 * elements. This will be used to advertise a camera timeout value to the
	 * pipeline handler.
	 */
	frameLengths_.pop_front();
	frameLengths_.push_back(helper_->exposure(vblank + mode_.height,
						  helper_->hblankToLineLength(hblank)));
}

} /* namespace ipa::RPi */

} /* namespace libcamera */
