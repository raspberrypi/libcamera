/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * rpi.cpp - Raspberry Pi Image Processing Algorithms
 */

#include <algorithm>
#include <array>
#include <cstring>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include <linux/bcm2835-isp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>
#include <libcamera/base/span.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/raspberrypi_ipa_interface.h>
#include <libcamera/request.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "agc_algorithm.h"
#include "agc_status.h"
#include "alsc_status.h"
#include "awb_algorithm.h"
#include "awb_status.h"
#include "black_level_status.h"
#include "cam_helper.h"
#include "ccm_algorithm.h"
#include "ccm_status.h"
#include "contrast_algorithm.h"
#include "contrast_status.h"
#include "controller.h"
#include "denoise_algorithm.h"
#include "denoise_status.h"
#include "dpc_status.h"
#include "focus_status.h"
#include "geq_status.h"
#include "lux_status.h"
#include "metadata.h"
#include "sharpen_algorithm.h"
#include "sharpen_status.h"

namespace libcamera {

using namespace std::literals::chrono_literals;
using utils::Duration;

/* Number of metadata objects available in the context list. */
constexpr unsigned int numMetadataContexts = 16;

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
static const ControlInfoMap::Map ipaControls{
	{ &controls::AeEnable, ControlInfo(false, true) },
	{ &controls::ExposureTime, ControlInfo(0, 66666) },
	{ &controls::AnalogueGain, ControlInfo(1.0f, 16.0f) },
	{ &controls::AeMeteringMode, ControlInfo(controls::AeMeteringModeValues) },
	{ &controls::AeConstraintMode, ControlInfo(controls::AeConstraintModeValues) },
	{ &controls::AeExposureMode, ControlInfo(controls::AeExposureModeValues) },
	{ &controls::ExposureValue, ControlInfo(-8.0f, 8.0f, 0.0f) },
	{ &controls::AwbEnable, ControlInfo(false, true) },
	{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
	{ &controls::AwbMode, ControlInfo(controls::AwbModeValues) },
	{ &controls::Brightness, ControlInfo(-1.0f, 1.0f, 0.0f) },
	{ &controls::Contrast, ControlInfo(0.0f, 32.0f, 1.0f) },
	{ &controls::Saturation, ControlInfo(0.0f, 32.0f, 1.0f) },
	{ &controls::Sharpness, ControlInfo(0.0f, 16.0f, 1.0f) },
	{ &controls::ColourCorrectionMatrix, ControlInfo(-16.0f, 16.0f) },
	{ &controls::ScalerCrop, ControlInfo(Rectangle{}, Rectangle(65535, 65535, 65535, 65535), Rectangle{}) },
	{ &controls::FrameDurationLimits, ControlInfo(INT64_C(33333), INT64_C(120000)) },
	{ &controls::draft::NoiseReductionMode, ControlInfo(controls::draft::NoiseReductionModeValues) }
};

LOG_DEFINE_CATEGORY(IPARPI)

namespace ipa::RPi {

class IPARPi : public IPARPiInterface
{
public:
	IPARPi()
		: controller_(), frameCount_(0), checkCount_(0), mistrustCount_(0),
		  lastRunTimestamp_(0), lsTable_(nullptr), firstStart_(true)
	{
	}

	~IPARPi()
	{
		if (lsTable_)
			munmap(lsTable_, MaxLsGridSize);
	}

	int init(const IPASettings &settings, IPAInitResult *result) override;
	void start(const ControlList &controls, StartConfig *startConfig) override;
	void stop() override {}

	int configure(const IPACameraSensorInfo &sensorInfo,
		      const std::map<unsigned int, IPAStream> &streamConfig,
		      const std::map<unsigned int, ControlInfoMap> &entityControls,
		      const IPAConfig &data,
		      ControlList *controls, IPAConfigResult *result) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void signalStatReady(const uint32_t bufferId, uint32_t ipaContext) override;
	void signalQueueRequest(const ControlList &controls) override;
	void signalIspPrepare(const ISPConfig &data) override;

private:
	void setMode(const IPACameraSensorInfo &sensorInfo);
	bool validateSensorControls();
	bool validateIspControls();
	void queueRequest(const ControlList &controls);
	void returnEmbeddedBuffer(unsigned int bufferId);
	void prepareISP(const ISPConfig &data);
	void reportMetadata(unsigned int ipaContext);
	void fillDeviceStatus(const ControlList &sensorControls, unsigned int ipaContext);
	void processStats(unsigned int bufferId, unsigned int ipaContext);
	void applyFrameDurations(Duration minFrameDuration, Duration maxFrameDuration);
	void applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls);
	void applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls);
	void applyDG(const struct AgcStatus *dgStatus, ControlList &ctrls);
	void applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls);
	void applyBlackLevel(const struct BlackLevelStatus *blackLevelStatus, ControlList &ctrls);
	void applyGamma(const struct ContrastStatus *contrastStatus, ControlList &ctrls);
	void applyGEQ(const struct GeqStatus *geqStatus, ControlList &ctrls);
	void applyDenoise(const struct DenoiseStatus *denoiseStatus, ControlList &ctrls);
	void applySharpen(const struct SharpenStatus *sharpenStatus, ControlList &ctrls);
	void applyDPC(const struct DpcStatus *dpcStatus, ControlList &ctrls);
	void applyLS(const struct AlscStatus *lsStatus, ControlList &ctrls);
	void resampleTable(uint16_t dest[], double const src[12][16], int destW, int destH);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap sensorCtrls_;
	ControlInfoMap ispCtrls_;
	ControlList libcameraMetadata_;

	/* Camera sensor params. */
	CameraMode mode_;

	/* Raspberry Pi controller specific defines. */
	std::unique_ptr<RPiController::CamHelper> helper_;
	RPiController::Controller controller_;
	std::array<RPiController::Metadata, numMetadataContexts> rpiMetadata_;

	/*
	 * We count frames to decide if the frame must be hidden (e.g. from
	 * display) or mistrusted (i.e. not given to the control algos).
	 */
	uint64_t frameCount_;

	/* For checking the sequencing of Prepare/Process calls. */
	uint64_t checkCount_;

	/* How many frames we should avoid running control algos on. */
	unsigned int mistrustCount_;

	/* Number of frames that need to be dropped on startup. */
	unsigned int dropFrameCount_;

	/* Frame timestamp for the last run of the controller. */
	uint64_t lastRunTimestamp_;

	/* Do we run a Controller::process() for this frame? */
	bool processPending_;

	/* LS table allocation passed in from the pipeline handler. */
	SharedFD lsTableHandle_;
	void *lsTable_;

	/* Distinguish the first camera start from others. */
	bool firstStart_;

	/* Frame duration (1/fps) limits. */
	Duration minFrameDuration_;
	Duration maxFrameDuration_;

	/* Maximum gain code for the sensor. */
	uint32_t maxSensorGainCode_;
};

int IPARPi::init(const IPASettings &settings, IPAInitResult *result)
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

	/*
	 * Pass out the sensor config to the pipeline handler in order
	 * to setup the staggered writer class.
	 */
	int gainDelay, exposureDelay, vblankDelay, hblankDelay, sensorMetadata;
	helper_->getDelays(exposureDelay, gainDelay, vblankDelay, hblankDelay);
	sensorMetadata = helper_->sensorEmbeddedDataPresent();

	result->sensorConfig.gainDelay = gainDelay;
	result->sensorConfig.exposureDelay = exposureDelay;
	result->sensorConfig.vblankDelay = vblankDelay;
	result->sensorConfig.hblankDelay = hblankDelay;
	result->sensorConfig.sensorMetadata = sensorMetadata;

	/* Load the tuning file for this sensor. */
	int ret = controller_.read(settings.configurationFile.c_str());
	if (ret) {
		LOG(IPARPI, Error)
			<< "Failed to load tuning data file "
			<< settings.configurationFile;
		return ret;
	}

	controller_.initialise();

	/* Return the controls handled by the IPA */
	ControlInfoMap::Map ctrlMap = ipaControls;
	result->controlInfo = ControlInfoMap(std::move(ctrlMap), controls::controls);

	return 0;
}

void IPARPi::start(const ControlList &controls, StartConfig *startConfig)
{
	RPiController::Metadata metadata;

	ASSERT(startConfig);
	if (!controls.empty()) {
		/* We have been given some controls to action before start. */
		queueRequest(controls);
	}

	controller_.switchMode(mode_, &metadata);

	/* SwitchMode may supply updated exposure/gain values to use. */
	AgcStatus agcStatus;
	agcStatus.shutterTime = 0.0s;
	agcStatus.analogueGain = 0.0;

	metadata.get("agc.status", agcStatus);
	if (agcStatus.shutterTime && agcStatus.analogueGain) {
		ControlList ctrls(sensorCtrls_);
		applyAGC(&agcStatus, ctrls);
		startConfig->controls = std::move(ctrls);
	}

	/*
	 * Initialise frame counts, and decide how many frames must be hidden or
	 * "mistrusted", which depends on whether this is a startup from cold,
	 * or merely a mode switch in a running system.
	 */
	frameCount_ = 0;
	checkCount_ = 0;
	if (firstStart_) {
		dropFrameCount_ = helper_->hideFramesStartup();
		mistrustCount_ = helper_->mistrustFramesStartup();

		/*
		 * Query the AGC/AWB for how many frames they may take to
		 * converge sufficiently. Where these numbers are non-zero
		 * we must allow for the frames with bad statistics
		 * (mistrustCount_) that they won't see. But if zero (i.e.
		 * no convergence necessary), no frames need to be dropped.
		 */
		unsigned int agcConvergenceFrames = 0;
		RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
			controller_.getAlgorithm("agc"));
		if (agc) {
			agcConvergenceFrames = agc->getConvergenceFrames();
			if (agcConvergenceFrames)
				agcConvergenceFrames += mistrustCount_;
		}

		unsigned int awbConvergenceFrames = 0;
		RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
			controller_.getAlgorithm("awb"));
		if (awb) {
			awbConvergenceFrames = awb->getConvergenceFrames();
			if (awbConvergenceFrames)
				awbConvergenceFrames += mistrustCount_;
		}

		dropFrameCount_ = std::max({ dropFrameCount_, agcConvergenceFrames, awbConvergenceFrames });
		LOG(IPARPI, Debug) << "Drop " << dropFrameCount_ << " frames on startup";
	} else {
		dropFrameCount_ = helper_->hideFramesModeSwitch();
		mistrustCount_ = helper_->mistrustFramesModeSwitch();
	}

	startConfig->dropFrameCount = dropFrameCount_;
	const Duration maxSensorFrameDuration = mode_.maxFrameLength * mode_.maxLineLength;
	startConfig->maxSensorFrameLengthMs = maxSensorFrameDuration.get<std::milli>();

	firstStart_ = false;
	lastRunTimestamp_ = 0;
}

void IPARPi::setMode(const IPACameraSensorInfo &sensorInfo)
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
	mode_.noiseFactor = sqrt(mode_.binX * mode_.binY);

	/*
	 * Calculate the line length as the ratio between the line length in
	 * pixels and the pixel rate.
	 */
	mode_.minLineLength = sensorInfo.minLineLength * (1.0s / sensorInfo.pixelRate);
	mode_.maxLineLength = sensorInfo.maxLineLength * (1.0s / sensorInfo.pixelRate);

	/*
	 * Set the frame length limits for the mode to ensure exposure and
	 * framerate calculations are clipped appropriately.
	 */
	mode_.minFrameLength = sensorInfo.minFrameLength;
	mode_.maxFrameLength = sensorInfo.maxFrameLength;

	/*
	 * Some sensors may have different sensitivities in different modes;
	 * the CamHelper will know the correct value.
	 */
	mode_.sensitivity = helper_->getModeSensitivity(mode_);
}

int IPARPi::configure(const IPACameraSensorInfo &sensorInfo,
		      [[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
		      const std::map<unsigned int, ControlInfoMap> &entityControls,
		      const IPAConfig &ipaConfig,
		      ControlList *controls, IPAConfigResult *result)
{
	if (entityControls.size() != 2) {
		LOG(IPARPI, Error) << "No ISP or sensor controls found.";
		return -1;
	}

	sensorCtrls_ = entityControls.at(0);
	ispCtrls_ = entityControls.at(1);

	if (!validateSensorControls()) {
		LOG(IPARPI, Error) << "Sensor control validation failed.";
		return -1;
	}

	if (!validateIspControls()) {
		LOG(IPARPI, Error) << "ISP control validation failed.";
		return -1;
	}

	maxSensorGainCode_ = sensorCtrls_.at(V4L2_CID_ANALOGUE_GAIN).max().get<int32_t>();

	/* Setup a metadata ControlList to output metadata. */
	libcameraMetadata_ = ControlList(controls::controls);

	/* Re-assemble camera mode using the sensor info. */
	setMode(sensorInfo);

	mode_.transform = static_cast<libcamera::Transform>(ipaConfig.transform);

	/* Store the lens shading table pointer and handle if available. */
	if (ipaConfig.lsTableHandle.isValid()) {
		/* Remove any previous table, if there was one. */
		if (lsTable_) {
			munmap(lsTable_, MaxLsGridSize);
			lsTable_ = nullptr;
		}

		/* Map the LS table buffer into user space. */
		lsTableHandle_ = std::move(ipaConfig.lsTableHandle);
		if (lsTableHandle_.isValid()) {
			lsTable_ = mmap(nullptr, MaxLsGridSize, PROT_READ | PROT_WRITE,
					MAP_SHARED, lsTableHandle_.get(), 0);

			if (lsTable_ == MAP_FAILED) {
				LOG(IPARPI, Error) << "dmaHeap mmap failure for LS table.";
				lsTable_ = nullptr;
			}
		}
	}

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
		agcStatus.shutterTime = defaultExposureTime;
		agcStatus.analogueGain = defaultAnalogueGain;
		applyAGC(&agcStatus, ctrls);
	}

	ASSERT(controls);
	*controls = std::move(ctrls);

	/*
	 * Apply the correct limits to the exposure, gain and frame duration controls
	 * based on the current sensor mode.
	 */
	ControlInfoMap::Map ctrlMap = ipaControls;
	const Duration minSensorFrameDuration = mode_.minFrameLength * mode_.minLineLength;
	const Duration maxSensorFrameDuration = mode_.maxFrameLength * mode_.maxLineLength;
	ctrlMap[&controls::FrameDurationLimits] =
		ControlInfo(static_cast<int64_t>(minSensorFrameDuration.get<std::micro>()),
			    static_cast<int64_t>(maxSensorFrameDuration.get<std::micro>()));

	ctrlMap[&controls::AnalogueGain] =
		ControlInfo(1.0f, static_cast<float>(helper_->gain(maxSensorGainCode_)));

	/*
	 * Calculate the max exposure limit from the frame duration limit as V4L2
	 * will limit the maximum control value based on the current VBLANK value.
	 */
	Duration maxShutter = Duration::max();
	helper_->getBlanking(maxShutter, minSensorFrameDuration, maxSensorFrameDuration);
	const uint32_t exposureMin = sensorCtrls_.at(V4L2_CID_EXPOSURE).min().get<int32_t>();

	ctrlMap[&controls::ExposureTime] =
		ControlInfo(static_cast<int32_t>(helper_->exposure(exposureMin, mode_.minLineLength).get<std::micro>()),
			    static_cast<int32_t>(maxShutter.get<std::micro>()));

	result->controlInfo = ControlInfoMap(std::move(ctrlMap), controls::controls);
	return 0;
}

void IPARPi::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

void IPARPi::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(id);
	}
}

void IPARPi::signalStatReady(uint32_t bufferId, uint32_t ipaContext)
{
	unsigned int context = ipaContext % rpiMetadata_.size();

	if (++checkCount_ != frameCount_) /* assert here? */
		LOG(IPARPI, Error) << "WARNING: Prepare/Process mismatch!!!";
	if (processPending_ && frameCount_ > mistrustCount_)
		processStats(bufferId, context);

	reportMetadata(context);

	statsMetadataComplete.emit(bufferId, libcameraMetadata_);
}

void IPARPi::signalQueueRequest(const ControlList &controls)
{
	queueRequest(controls);
}

void IPARPi::signalIspPrepare(const ISPConfig &data)
{
	/*
	 * At start-up, or after a mode-switch, we may want to
	 * avoid running the control algos for a few frames in case
	 * they are "unreliable".
	 */
	prepareISP(data);
	frameCount_++;

	/* Ready to push the input buffer into the ISP. */
	runIsp.emit(data.bayerBufferId);
}

void IPARPi::reportMetadata(unsigned int ipaContext)
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
				       deviceStatus->shutterSpeed.get<std::micro>());
		libcameraMetadata_.set(controls::AnalogueGain, deviceStatus->analogueGain);
		libcameraMetadata_.set(controls::FrameDuration,
				       helper_->exposure(deviceStatus->frameLength, deviceStatus->lineLength).get<std::micro>());
		if (deviceStatus->sensorTemperature)
			libcameraMetadata_.set(controls::SensorTemperature, *deviceStatus->sensorTemperature);
	}

	AgcStatus *agcStatus = rpiMetadata.getLocked<AgcStatus>("agc.status");
	if (agcStatus) {
		libcameraMetadata_.set(controls::AeLocked, agcStatus->locked);
		libcameraMetadata_.set(controls::DigitalGain, agcStatus->digitalGain);
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

	FocusStatus *focusStatus = rpiMetadata.getLocked<FocusStatus>("focus.status");
	if (focusStatus && focusStatus->num == 12) {
		/*
		 * We get a 4x3 grid of regions by default. Calculate the average
		 * FoM over the central two positions to give an overall scene FoM.
		 * This can change later if it is not deemed suitable.
		 */
		int32_t focusFoM = (focusStatus->focusMeasures[5] + focusStatus->focusMeasures[6]) / 2;
		libcameraMetadata_.set(controls::FocusFoM, focusFoM);
	}

	CcmStatus *ccmStatus = rpiMetadata.getLocked<CcmStatus>("ccm.status");
	if (ccmStatus) {
		float m[9];
		for (unsigned int i = 0; i < 9; i++)
			m[i] = ccmStatus->matrix[i];
		libcameraMetadata_.set(controls::ColourCorrectionMatrix, m);
	}
}

bool IPARPi::validateSensorControls()
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

bool IPARPi::validateIspControls()
{
	static const uint32_t ctrls[] = {
		V4L2_CID_RED_BALANCE,
		V4L2_CID_BLUE_BALANCE,
		V4L2_CID_DIGITAL_GAIN,
		V4L2_CID_USER_BCM2835_ISP_CC_MATRIX,
		V4L2_CID_USER_BCM2835_ISP_GAMMA,
		V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL,
		V4L2_CID_USER_BCM2835_ISP_GEQ,
		V4L2_CID_USER_BCM2835_ISP_DENOISE,
		V4L2_CID_USER_BCM2835_ISP_SHARPEN,
		V4L2_CID_USER_BCM2835_ISP_DPC,
		V4L2_CID_USER_BCM2835_ISP_LENS_SHADING,
		V4L2_CID_USER_BCM2835_ISP_CDN,
	};

	for (auto c : ctrls) {
		if (ispCtrls_.find(c) == ispCtrls_.end()) {
			LOG(IPARPI, Error) << "Unable to find ISP control "
					   << utils::hex(c);
			return false;
		}
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

static const std::map<int32_t, RPiController::DenoiseMode> DenoiseModeTable = {
	{ controls::draft::NoiseReductionModeOff, RPiController::DenoiseMode::Off },
	{ controls::draft::NoiseReductionModeFast, RPiController::DenoiseMode::ColourFast },
	{ controls::draft::NoiseReductionModeHighQuality, RPiController::DenoiseMode::ColourHighQuality },
	{ controls::draft::NoiseReductionModeMinimal, RPiController::DenoiseMode::ColourOff },
	{ controls::draft::NoiseReductionModeZSL, RPiController::DenoiseMode::ColourHighQuality },
};

void IPARPi::queueRequest(const ControlList &controls)
{
	/* Clear the return metadata buffer. */
	libcameraMetadata_.clear();

	for (auto const &ctrl : controls) {
		LOG(IPARPI, Debug) << "Request ctrl: "
				   << controls::controls.at(ctrl.first)->name()
				   << " = " << ctrl.second.toString();

		switch (ctrl.first) {
		case controls::AE_ENABLE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set AE_ENABLE - no AGC algorithm";
				break;
			}

			if (ctrl.second.get<bool>() == false)
				agc->disableAuto();
			else
				agc->enableAuto();

			libcameraMetadata_.set(controls::AeEnable, ctrl.second.get<bool>());
			break;
		}

		case controls::EXPOSURE_TIME: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set EXPOSURE_TIME - no AGC algorithm";
				break;
			}

			/* The control provides units of microseconds. */
			agc->setFixedShutter(ctrl.second.get<int32_t>() * 1.0us);

			libcameraMetadata_.set(controls::ExposureTime, ctrl.second.get<int32_t>());
			break;
		}

		case controls::ANALOGUE_GAIN: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.getAlgorithm("agc"));
			if (!agc) {
				LOG(IPARPI, Warning)
					<< "Could not set ANALOGUE_GAIN - no AGC algorithm";
				break;
			}

			agc->setFixedAnalogueGain(ctrl.second.get<float>());

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
			agc->setEv(ev);
			libcameraMetadata_.set(controls::ExposureValue,
					       ctrl.second.get<float>());
			break;
		}

		case controls::AWB_ENABLE: {
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

		case controls::SCALER_CROP: {
			/* We do nothing with this, but should avoid the warning below. */
			break;
		}

		case controls::FRAME_DURATION_LIMITS: {
			auto frameDurations = ctrl.second.get<Span<const int64_t>>();
			applyFrameDurations(frameDurations[0] * 1.0us, frameDurations[1] * 1.0us);
			break;
		}

		case controls::NOISE_REDUCTION_MODE: {
			RPiController::DenoiseAlgorithm *sdn = dynamic_cast<RPiController::DenoiseAlgorithm *>(
				controller_.getAlgorithm("SDN"));
			if (!sdn) {
				LOG(IPARPI, Warning)
					<< "Could not set NOISE_REDUCTION_MODE - no SDN algorithm";
				break;
			}

			int32_t idx = ctrl.second.get<int32_t>();
			auto mode = DenoiseModeTable.find(idx);
			if (mode != DenoiseModeTable.end()) {
				sdn->setMode(mode->second);

				/*
				 * \todo If the colour denoise is not going to run due to an
				 * analysis image resolution or format mismatch, we should
				 * report the status correctly in the metadata.
				 */
				libcameraMetadata_.set(controls::draft::NoiseReductionMode, idx);
			} else {
				LOG(IPARPI, Error) << "Noise reduction mode " << idx
						   << " not recognised";
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
}

void IPARPi::returnEmbeddedBuffer(unsigned int bufferId)
{
	embeddedComplete.emit(bufferId);
}

void IPARPi::prepareISP(const ISPConfig &data)
{
	int64_t frameTimestamp = data.controls.get(controls::SensorTimestamp).value_or(0);
	unsigned int ipaContext = data.ipaContext % rpiMetadata_.size();
	RPiController::Metadata &rpiMetadata = rpiMetadata_[ipaContext];
	Span<uint8_t> embeddedBuffer;

	rpiMetadata.clear();
	fillDeviceStatus(data.controls, ipaContext);

	if (data.embeddedBufferPresent) {
		/*
		 * Pipeline handler has supplied us with an embedded data buffer,
		 * we must pass it to the CamHelper for parsing.
		 */
		auto it = buffers_.find(data.embeddedBufferId);
		ASSERT(it != buffers_.end());
		embeddedBuffer = it->second.planes()[0];
	}

	/*
	 * AGC wants to know the algorithm status from the time it actioned the
	 * sensor exposure/gain changes. So fetch it from the metadata list
	 * indexed by the IPA cookie returned, and put it in the current frame
	 * metadata.
	 */
	AgcStatus agcStatus;
	RPiController::Metadata &delayedMetadata = rpiMetadata_[data.delayContext];
	if (!delayedMetadata.get<AgcStatus>("agc.status", agcStatus))
		rpiMetadata.set("agc.delayed_status", agcStatus);

	/*
	 * This may overwrite the DeviceStatus using values from the sensor
	 * metadata, and may also do additional custom processing.
	 */
	helper_->prepare(embeddedBuffer, rpiMetadata);

	/* Done with embedded data now, return to pipeline handler asap. */
	if (data.embeddedBufferPresent)
		returnEmbeddedBuffer(data.embeddedBufferId);

	/* Allow a 10% margin on the comparison below. */
	Duration delta = (frameTimestamp - lastRunTimestamp_) * 1.0ns;
	if (lastRunTimestamp_ && frameCount_ > dropFrameCount_ &&
	    delta < controllerMinFrameDuration * 0.9) {
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
		return;
	}

	lastRunTimestamp_ = frameTimestamp;
	processPending_ = true;

	ControlList ctrls(ispCtrls_);

	controller_.prepare(&rpiMetadata);

	/* Lock the metadata buffer to avoid constant locks/unlocks. */
	std::unique_lock<RPiController::Metadata> lock(rpiMetadata);

	AwbStatus *awbStatus = rpiMetadata.getLocked<AwbStatus>("awb.status");
	if (awbStatus)
		applyAWB(awbStatus, ctrls);

	CcmStatus *ccmStatus = rpiMetadata.getLocked<CcmStatus>("ccm.status");
	if (ccmStatus)
		applyCCM(ccmStatus, ctrls);

	AgcStatus *dgStatus = rpiMetadata.getLocked<AgcStatus>("agc.status");
	if (dgStatus)
		applyDG(dgStatus, ctrls);

	AlscStatus *lsStatus = rpiMetadata.getLocked<AlscStatus>("alsc.status");
	if (lsStatus)
		applyLS(lsStatus, ctrls);

	ContrastStatus *contrastStatus = rpiMetadata.getLocked<ContrastStatus>("contrast.status");
	if (contrastStatus)
		applyGamma(contrastStatus, ctrls);

	BlackLevelStatus *blackLevelStatus = rpiMetadata.getLocked<BlackLevelStatus>("black_level.status");
	if (blackLevelStatus)
		applyBlackLevel(blackLevelStatus, ctrls);

	GeqStatus *geqStatus = rpiMetadata.getLocked<GeqStatus>("geq.status");
	if (geqStatus)
		applyGEQ(geqStatus, ctrls);

	DenoiseStatus *denoiseStatus = rpiMetadata.getLocked<DenoiseStatus>("denoise.status");
	if (denoiseStatus)
		applyDenoise(denoiseStatus, ctrls);

	SharpenStatus *sharpenStatus = rpiMetadata.getLocked<SharpenStatus>("sharpen.status");
	if (sharpenStatus)
		applySharpen(sharpenStatus, ctrls);

	DpcStatus *dpcStatus = rpiMetadata.getLocked<DpcStatus>("dpc.status");
	if (dpcStatus)
		applyDPC(dpcStatus, ctrls);

	if (!ctrls.empty())
		setIspControls.emit(ctrls);
}

void IPARPi::fillDeviceStatus(const ControlList &sensorControls, unsigned int ipaContext)
{
	DeviceStatus deviceStatus = {};

	int32_t exposureLines = sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	int32_t gainCode = sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();
	int32_t vblank = sensorControls.get(V4L2_CID_VBLANK).get<int32_t>();
	int32_t hblank = sensorControls.get(V4L2_CID_HBLANK).get<int32_t>();

	deviceStatus.lineLength = helper_->hblankToLineLength(hblank);
	deviceStatus.shutterSpeed = helper_->exposure(exposureLines, deviceStatus.lineLength);
	deviceStatus.analogueGain = helper_->gain(gainCode);
	deviceStatus.frameLength = mode_.height + vblank;

	LOG(IPARPI, Debug) << "Metadata - " << deviceStatus;

	rpiMetadata_[ipaContext].set("device.status", deviceStatus);
}

void IPARPi::processStats(unsigned int bufferId, unsigned int ipaContext)
{
	RPiController::Metadata &rpiMetadata = rpiMetadata_[ipaContext];

	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPARPI, Error) << "Could not find stats buffer!";
		return;
	}

	Span<uint8_t> mem = it->second.planes()[0];
	bcm2835_isp_stats *stats = reinterpret_cast<bcm2835_isp_stats *>(mem.data());
	RPiController::StatisticsPtr statistics = std::make_shared<bcm2835_isp_stats>(*stats);
	helper_->process(statistics, rpiMetadata);
	controller_.process(statistics, &rpiMetadata);

	struct AgcStatus agcStatus;
	if (rpiMetadata.get("agc.status", agcStatus) == 0) {
		ControlList ctrls(sensorCtrls_);
		applyAGC(&agcStatus, ctrls);

		setDelayedControls.emit(ctrls, ipaContext);
	}
}

void IPARPi::applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls)
{
	LOG(IPARPI, Debug) << "Applying WB R: " << awbStatus->gainR << " B: "
			   << awbStatus->gainB;

	ctrls.set(V4L2_CID_RED_BALANCE,
		  static_cast<int32_t>(awbStatus->gainR * 1000));
	ctrls.set(V4L2_CID_BLUE_BALANCE,
		  static_cast<int32_t>(awbStatus->gainB * 1000));
}

void IPARPi::applyFrameDurations(Duration minFrameDuration, Duration maxFrameDuration)
{
	const Duration minSensorFrameDuration = mode_.minFrameLength * mode_.minLineLength;
	const Duration maxSensorFrameDuration = mode_.maxFrameLength * mode_.maxLineLength;

	/*
	 * This will only be applied once AGC recalculations occur.
	 * The values may be clamped based on the sensor mode capabilities as well.
	 */
	minFrameDuration_ = minFrameDuration ? minFrameDuration : defaultMaxFrameDuration;
	maxFrameDuration_ = maxFrameDuration ? maxFrameDuration : defaultMinFrameDuration;
	minFrameDuration_ = std::clamp(minFrameDuration_,
				       minSensorFrameDuration, maxSensorFrameDuration);
	maxFrameDuration_ = std::clamp(maxFrameDuration_,
				       minSensorFrameDuration, maxSensorFrameDuration);
	maxFrameDuration_ = std::max(maxFrameDuration_, minFrameDuration_);

	/* Return the validated limits via metadata. */
	libcameraMetadata_.set(controls::FrameDurationLimits,
			       { static_cast<int64_t>(minFrameDuration_.get<std::micro>()),
				 static_cast<int64_t>(maxFrameDuration_.get<std::micro>()) });

	/*
	 * Calculate the maximum exposure time possible for the AGC to use.
	 * getBlanking() will update maxShutter with the largest exposure
	 * value possible.
	 */
	Duration maxShutter = Duration::max();
	helper_->getBlanking(maxShutter, minFrameDuration_, maxFrameDuration_);

	RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
		controller_.getAlgorithm("agc"));
	agc->setMaxShutter(maxShutter);
}

void IPARPi::applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls)
{
	int32_t gainCode = helper_->gainCode(agcStatus->analogueGain);

	/*
	 * Ensure anything larger than the max gain code will not be passed to
	 * DelayedControls. The AGC will correctly handle a lower gain returned
	 * by the sensor, provided it knows the actual gain used.
	 */
	gainCode = std::min<int32_t>(gainCode, maxSensorGainCode_);

	/* getBlanking might clip exposure time to the fps limits. */
	Duration exposure = agcStatus->shutterTime;
	auto [vblank, hblank] = helper_->getBlanking(exposure, minFrameDuration_, maxFrameDuration_);
	int32_t exposureLines = helper_->exposureLines(exposure,
						       helper_->hblankToLineLength(hblank));

	LOG(IPARPI, Debug) << "Applying AGC Exposure: " << exposure
			   << " (Shutter lines: " << exposureLines << ", AGC requested "
			   << agcStatus->shutterTime << ") Gain: "
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
}

void IPARPi::applyDG(const struct AgcStatus *dgStatus, ControlList &ctrls)
{
	ctrls.set(V4L2_CID_DIGITAL_GAIN,
		  static_cast<int32_t>(dgStatus->digitalGain * 1000));
}

void IPARPi::applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls)
{
	bcm2835_isp_custom_ccm ccm;

	for (int i = 0; i < 9; i++) {
		ccm.ccm.ccm[i / 3][i % 3].den = 1000;
		ccm.ccm.ccm[i / 3][i % 3].num = 1000 * ccmStatus->matrix[i];
	}

	ccm.enabled = 1;
	ccm.ccm.offsets[0] = ccm.ccm.offsets[1] = ccm.ccm.offsets[2] = 0;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&ccm),
					    sizeof(ccm) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_CC_MATRIX, c);
}

void IPARPi::applyGamma(const struct ContrastStatus *contrastStatus, ControlList &ctrls)
{
	struct bcm2835_isp_gamma gamma;

	gamma.enabled = 1;
	for (unsigned int i = 0; i < ContrastNumPoints; i++) {
		gamma.x[i] = contrastStatus->points[i].x;
		gamma.y[i] = contrastStatus->points[i].y;
	}

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&gamma),
					    sizeof(gamma) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_GAMMA, c);
}

void IPARPi::applyBlackLevel(const struct BlackLevelStatus *blackLevelStatus, ControlList &ctrls)
{
	bcm2835_isp_black_level blackLevel;

	blackLevel.enabled = 1;
	blackLevel.black_level_r = blackLevelStatus->blackLevelR;
	blackLevel.black_level_g = blackLevelStatus->blackLevelG;
	blackLevel.black_level_b = blackLevelStatus->blackLevelB;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&blackLevel),
					    sizeof(blackLevel) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL, c);
}

void IPARPi::applyGEQ(const struct GeqStatus *geqStatus, ControlList &ctrls)
{
	bcm2835_isp_geq geq;

	geq.enabled = 1;
	geq.offset = geqStatus->offset;
	geq.slope.den = 1000;
	geq.slope.num = 1000 * geqStatus->slope;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&geq),
					    sizeof(geq) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_GEQ, c);
}

void IPARPi::applyDenoise(const struct DenoiseStatus *denoiseStatus, ControlList &ctrls)
{
	using RPiController::DenoiseMode;

	bcm2835_isp_denoise denoise;
	DenoiseMode mode = static_cast<DenoiseMode>(denoiseStatus->mode);

	denoise.enabled = mode != DenoiseMode::Off;
	denoise.constant = denoiseStatus->noiseConstant;
	denoise.slope.num = 1000 * denoiseStatus->noiseSlope;
	denoise.slope.den = 1000;
	denoise.strength.num = 1000 * denoiseStatus->strength;
	denoise.strength.den = 1000;

	/* Set the CDN mode to match the SDN operating mode. */
	bcm2835_isp_cdn cdn;
	switch (mode) {
	case DenoiseMode::ColourFast:
		cdn.enabled = 1;
		cdn.mode = CDN_MODE_FAST;
		break;
	case DenoiseMode::ColourHighQuality:
		cdn.enabled = 1;
		cdn.mode = CDN_MODE_HIGH_QUALITY;
		break;
	default:
		cdn.enabled = 0;
	}

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&denoise),
					    sizeof(denoise) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_DENOISE, c);

	c = ControlValue(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&cdn),
					      sizeof(cdn) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_CDN, c);
}

void IPARPi::applySharpen(const struct SharpenStatus *sharpenStatus, ControlList &ctrls)
{
	bcm2835_isp_sharpen sharpen;

	sharpen.enabled = 1;
	sharpen.threshold.num = 1000 * sharpenStatus->threshold;
	sharpen.threshold.den = 1000;
	sharpen.strength.num = 1000 * sharpenStatus->strength;
	sharpen.strength.den = 1000;
	sharpen.limit.num = 1000 * sharpenStatus->limit;
	sharpen.limit.den = 1000;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&sharpen),
					    sizeof(sharpen) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_SHARPEN, c);
}

void IPARPi::applyDPC(const struct DpcStatus *dpcStatus, ControlList &ctrls)
{
	bcm2835_isp_dpc dpc;

	dpc.enabled = 1;
	dpc.strength = dpcStatus->strength;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&dpc),
					    sizeof(dpc) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_DPC, c);
}

void IPARPi::applyLS(const struct AlscStatus *lsStatus, ControlList &ctrls)
{
	/*
	 * Program lens shading tables into pipeline.
	 * Choose smallest cell size that won't exceed 63x48 cells.
	 */
	const int cellSizes[] = { 16, 32, 64, 128, 256 };
	unsigned int numCells = std::size(cellSizes);
	unsigned int i, w, h, cellSize;
	for (i = 0; i < numCells; i++) {
		cellSize = cellSizes[i];
		w = (mode_.width + cellSize - 1) / cellSize;
		h = (mode_.height + cellSize - 1) / cellSize;
		if (w < 64 && h <= 48)
			break;
	}

	if (i == numCells) {
		LOG(IPARPI, Error) << "Cannot find cell size";
		return;
	}

	/* We're going to supply corner sampled tables, 16 bit samples. */
	w++, h++;
	bcm2835_isp_lens_shading ls = {
		.enabled = 1,
		.grid_cell_size = cellSize,
		.grid_width = w,
		.grid_stride = w,
		.grid_height = h,
		/* .dmabuf will be filled in by pipeline handler. */
		.dmabuf = 0,
		.ref_transform = 0,
		.corner_sampled = 1,
		.gain_format = GAIN_FORMAT_U4P10
	};

	if (!lsTable_ || w * h * 4 * sizeof(uint16_t) > MaxLsGridSize) {
		LOG(IPARPI, Error) << "Do not have a correctly allocate lens shading table!";
		return;
	}

	if (lsStatus) {
		/* Format will be u4.10 */
		uint16_t *grid = static_cast<uint16_t *>(lsTable_);

		resampleTable(grid, lsStatus->r, w, h);
		resampleTable(grid + w * h, lsStatus->g, w, h);
		std::memcpy(grid + 2 * w * h, grid + w * h, w * h * sizeof(uint16_t));
		resampleTable(grid + 3 * w * h, lsStatus->b, w, h);
	}

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&ls),
					    sizeof(ls) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_LENS_SHADING, c);
}

/*
 * Resamples a 16x12 table with central sampling to destW x destH with corner
 * sampling.
 */
void IPARPi::resampleTable(uint16_t dest[], double const src[12][16],
			   int destW, int destH)
{
	/*
	 * Precalculate and cache the x sampling locations and phases to
	 * save recomputing them on every row.
	 */
	assert(destW > 1 && destH > 1 && destW <= 64);
	int xLo[64], xHi[64];
	double xf[64];
	double x = -0.5, xInc = 16.0 / (destW - 1);
	for (int i = 0; i < destW; i++, x += xInc) {
		xLo[i] = floor(x);
		xf[i] = x - xLo[i];
		xHi[i] = xLo[i] < 15 ? xLo[i] + 1 : 15;
		xLo[i] = xLo[i] > 0 ? xLo[i] : 0;
	}

	/* Now march over the output table generating the new values. */
	double y = -0.5, yInc = 12.0 / (destH - 1);
	for (int j = 0; j < destH; j++, y += yInc) {
		int yLo = floor(y);
		double yf = y - yLo;
		int yHi = yLo < 11 ? yLo + 1 : 11;
		yLo = yLo > 0 ? yLo : 0;
		double const *rowAbove = src[yLo];
		double const *rowBelow = src[yHi];
		for (int i = 0; i < destW; i++) {
			double above = rowAbove[xLo[i]] * (1 - xf[i]) + rowAbove[xHi[i]] * xf[i];
			double below = rowBelow[xLo[i]] * (1 - xf[i]) + rowBelow[xHi[i]] * xf[i];
			int result = floor(1024 * (above * (1 - yf) + below * yf) + .5);
			*(dest++) = result > 16383 ? 16383 : result; /* want u4.10 */
		}
	}
}

} /* namespace ipa::RPi */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerRPi",
	"raspberrypi",
};

IPAInterface *ipaCreate()
{
	return new ipa::RPi::IPARPi();
}

} /* extern "C" */

} /* namespace libcamera */
