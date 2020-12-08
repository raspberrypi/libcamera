/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2020, Raspberry Pi (Trading) Ltd.
 *
 * rpi.cpp - Raspberry Pi Image Processing Algorithms
 */

#include <algorithm>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/file_descriptor.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/raspberrypi.h>
#include <libcamera/request.h>
#include <libcamera/span.h>

#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/buffer.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/utils.h"

#include <linux/bcm2835-isp.h>

#include "agc_algorithm.hpp"
#include "agc_status.h"
#include "alsc_status.h"
#include "awb_algorithm.hpp"
#include "awb_status.h"
#include "black_level_status.h"
#include "cam_helper.hpp"
#include "ccm_algorithm.hpp"
#include "ccm_status.h"
#include "contrast_algorithm.hpp"
#include "contrast_status.h"
#include "controller.hpp"
#include "dpc_status.h"
#include "focus_status.h"
#include "geq_status.h"
#include "lux_status.h"
#include "metadata.hpp"
#include "noise_status.h"
#include "sdn_status.h"
#include "sharpen_algorithm.hpp"
#include "sharpen_status.h"

namespace libcamera {

/* Configure the sensor with these values initially. */
constexpr double DefaultAnalogueGain = 1.0;
constexpr unsigned int DefaultExposureTime = 20000;

LOG_DEFINE_CATEGORY(IPARPI)

class IPARPi : public IPAInterface
{
public:
	IPARPi()
		: lastMode_({}), controller_(), controllerInit_(false),
		  frameCount_(0), checkCount_(0), mistrustCount_(0),
		  lsTable_(nullptr), firstStart_(true)
	{
	}

	~IPARPi()
	{
		if (lsTable_)
			munmap(lsTable_, RPi::MaxLsGridSize);
	}

	int init(const IPASettings &settings) override;
	int start(const IPAOperationData &data, IPAOperationData *result) override;
	void stop() override {}

	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *response) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPAOperationData &event) override;

private:
	void setMode(const CameraSensorInfo &sensorInfo);
	void queueRequest(const ControlList &controls);
	void returnEmbeddedBuffer(unsigned int bufferId);
	void prepareISP(unsigned int bufferId);
	void reportMetadata();
	bool parseEmbeddedData(unsigned int bufferId, struct DeviceStatus &deviceStatus);
	void processStats(unsigned int bufferId);
	void applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls);
	void applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls);
	void applyDG(const struct AgcStatus *dgStatus, ControlList &ctrls);
	void applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls);
	void applyBlackLevel(const struct BlackLevelStatus *blackLevelStatus, ControlList &ctrls);
	void applyGamma(const struct ContrastStatus *contrastStatus, ControlList &ctrls);
	void applyGEQ(const struct GeqStatus *geqStatus, ControlList &ctrls);
	void applyDenoise(const struct SdnStatus *denoiseStatus, ControlList &ctrls);
	void applySharpen(const struct SharpenStatus *sharpenStatus, ControlList &ctrls);
	void applyDPC(const struct DpcStatus *dpcStatus, ControlList &ctrls);
	void applyLS(const struct AlscStatus *lsStatus, ControlList &ctrls);
	void resampleTable(uint16_t dest[], double const src[12][16], int destW, int destH);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap unicamCtrls_;
	ControlInfoMap ispCtrls_;
	ControlList libcameraMetadata_;

	/* IPA configuration. */
	std::string tuningFile_;

	/* Camera sensor params. */
	CameraMode mode_;
	CameraMode lastMode_;

	/* Raspberry Pi controller specific defines. */
	std::unique_ptr<RPiController::CamHelper> helper_;
	RPiController::Controller controller_;
	bool controllerInit_;
	RPiController::Metadata rpiMetadata_;

	/*
	 * We count frames to decide if the frame must be hidden (e.g. from
	 * display) or mistrusted (i.e. not given to the control algos).
	 */
	uint64_t frameCount_;

	/* For checking the sequencing of Prepare/Process calls. */
	uint64_t checkCount_;

	/* How many frames we should avoid running control algos on. */
	unsigned int mistrustCount_;

	/* LS table allocation passed in from the pipeline handler. */
	FileDescriptor lsTableHandle_;
	void *lsTable_;

	/* Distinguish the first camera start from others. */
	bool firstStart_;
};

int IPARPi::init(const IPASettings &settings)
{
	tuningFile_ = settings.configurationFile;
	return 0;
}

int IPARPi::start(const IPAOperationData &data, IPAOperationData *result)
{
	RPiController::Metadata metadata;

	ASSERT(result);
	result->operation = 0;
	if (data.operation & RPi::IPA_CONFIG_STARTUP) {
		/* We have been given some controls to action before start. */
		queueRequest(data.controls[0]);
	}

	controller_.SwitchMode(mode_, &metadata);

	/* SwitchMode may supply updated exposure/gain values to use. */
	AgcStatus agcStatus;
	agcStatus.shutter_time = 0.0;
	agcStatus.analogue_gain = 0.0;

	/* SwitchMode may supply updated exposure/gain values to use. */
	metadata.Get("agc.status", agcStatus);
	if (agcStatus.shutter_time != 0.0 && agcStatus.analogue_gain != 0.0) {
		ControlList ctrls(unicamCtrls_);
		applyAGC(&agcStatus, ctrls);
		result->controls.emplace_back(ctrls);
		result->operation |= RPi::IPA_CONFIG_SENSOR;
	}

	/*
	 * Initialise frame counts, and decide how many frames must be hidden or
	 * "mistrusted", which depends on whether this is a startup from cold,
	 * or merely a mode switch in a running system.
	 */
	frameCount_ = 0;
	checkCount_ = 0;
	unsigned int dropFrame = 0;
	if (firstStart_) {
		dropFrame = helper_->HideFramesStartup();
		mistrustCount_ = helper_->MistrustFramesStartup();

		/*
		 * Query the AGC/AWB for how many frames they may take to
		 * converge sufficiently. Where these numbers are non-zero
		 * we must allow for the frames with bad statistics
		 * (mistrustCount_) that they won't see. But if zero (i.e.
		 * no convergence necessary), no frames need to be dropped.
		 */
		unsigned int agcConvergenceFrames = 0;
		RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
			controller_.GetAlgorithm("agc"));
		if (agc) {
			agcConvergenceFrames = agc->GetConvergenceFrames();
			if (agcConvergenceFrames)
				agcConvergenceFrames += mistrustCount_;
		}

		unsigned int awbConvergenceFrames = 0;
		RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
			controller_.GetAlgorithm("awb"));
		if (awb) {
			awbConvergenceFrames = awb->GetConvergenceFrames();
			if (awbConvergenceFrames)
				awbConvergenceFrames += mistrustCount_;
		}

		dropFrame = std::max({ dropFrame, agcConvergenceFrames, awbConvergenceFrames });
		LOG(IPARPI, Debug) << "Drop " << dropFrame << " frames on startup";
	} else {
		dropFrame = helper_->HideFramesModeSwitch();
		mistrustCount_ = helper_->MistrustFramesModeSwitch();
	}

	result->data.push_back(dropFrame);
	result->operation |= RPi::IPA_CONFIG_DROP_FRAMES;

	firstStart_ = false;

	return 0;
}

void IPARPi::setMode(const CameraSensorInfo &sensorInfo)
{
	mode_.bitdepth = sensorInfo.bitsPerPixel;
	mode_.width = sensorInfo.outputSize.width;
	mode_.height = sensorInfo.outputSize.height;
	mode_.sensor_width = sensorInfo.activeAreaSize.width;
	mode_.sensor_height = sensorInfo.activeAreaSize.height;
	mode_.crop_x = sensorInfo.analogCrop.x;
	mode_.crop_y = sensorInfo.analogCrop.y;

	/*
	 * Calculate scaling parameters. The scale_[xy] factors are determined
	 * by the ratio between the crop rectangle size and the output size.
	 */
	mode_.scale_x = sensorInfo.analogCrop.width / sensorInfo.outputSize.width;
	mode_.scale_y = sensorInfo.analogCrop.height / sensorInfo.outputSize.height;

	/*
	 * We're not told by the pipeline handler how scaling is split between
	 * binning and digital scaling. For now, as a heuristic, assume that
	 * downscaling up to 2 is achieved through binning, and that any
	 * additional scaling is achieved through digital scaling.
	 *
	 * \todo Get the pipeline handle to provide the full data
	 */
	mode_.bin_x = std::min(2, static_cast<int>(mode_.scale_x));
	mode_.bin_y = std::min(2, static_cast<int>(mode_.scale_y));

	/* The noise factor is the square root of the total binning factor. */
	mode_.noise_factor = sqrt(mode_.bin_x * mode_.bin_y);

	/*
	 * Calculate the line length in nanoseconds as the ratio between
	 * the line length in pixels and the pixel rate.
	 */
	mode_.line_length = 1e9 * sensorInfo.lineLength / sensorInfo.pixelRate;
}

void IPARPi::configure(const CameraSensorInfo &sensorInfo,
		       [[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result)
{
	if (entityControls.size() != 2) {
		LOG(IPARPI, Error) << "No ISP or sensor controls found.";
		result->operation = RPi::IPA_CONFIG_FAILED;
		return;
	}

	result->operation = 0;

	unicamCtrls_ = entityControls.at(0);
	ispCtrls_ = entityControls.at(1);

	/* Setup a metadata ControlList to output metadata. */
	libcameraMetadata_ = ControlList(controls::controls);

	/*
	 * Load the "helper" for this sensor. This tells us all the device specific stuff
	 * that the kernel driver doesn't. We only do this the first time; we don't need
	 * to re-parse the metadata after a simple mode-switch for no reason.
	 */
	std::string cameraName(sensorInfo.model);
	if (!helper_) {
		helper_ = std::unique_ptr<RPiController::CamHelper>(RPiController::CamHelper::Create(cameraName));

		if (!helper_) {
			LOG(IPARPI, Error) << "Could not create camera helper for "
					   << cameraName;
			result->operation = RPi::IPA_CONFIG_FAILED;
			return;
		}

		/*
		 * Pass out the sensor config to the pipeline handler in order
		 * to setup the staggered writer class.
		 */
		int gainDelay, exposureDelay, sensorMetadata;
		helper_->GetDelays(exposureDelay, gainDelay);
		sensorMetadata = helper_->SensorEmbeddedDataPresent();

		result->data.push_back(gainDelay);
		result->data.push_back(exposureDelay);
		result->data.push_back(sensorMetadata);

		result->operation |= RPi::IPA_CONFIG_STAGGERED_WRITE;
	}

	/* Re-assemble camera mode using the sensor info. */
	setMode(sensorInfo);

	/*
	 * The ipaConfig.data always gives us the user transform first. Note that
	 * this will always make the LS table pointer (if present) element 1.
	 */
	mode_.transform = static_cast<libcamera::Transform>(ipaConfig.data[0]);

	/* Store the lens shading table pointer and handle if available. */
	if (ipaConfig.operation & RPi::IPA_CONFIG_LS_TABLE) {
		/* Remove any previous table, if there was one. */
		if (lsTable_) {
			munmap(lsTable_, RPi::MaxLsGridSize);
			lsTable_ = nullptr;
		}

		/* Map the LS table buffer into user space (now element 1). */
		lsTableHandle_ = FileDescriptor(ipaConfig.data[1]);
		if (lsTableHandle_.isValid()) {
			lsTable_ = mmap(nullptr, RPi::MaxLsGridSize, PROT_READ | PROT_WRITE,
					MAP_SHARED, lsTableHandle_.fd(), 0);

			if (lsTable_ == MAP_FAILED) {
				LOG(IPARPI, Error) << "dmaHeap mmap failure for LS table.";
				lsTable_ = nullptr;
			}
		}
	}

	/* Pass the camera mode to the CamHelper to setup algorithms. */
	helper_->SetCameraMode(mode_);

	if (!controllerInit_) {
		/* Load the tuning file for this sensor. */
		controller_.Read(tuningFile_.c_str());
		controller_.Initialise();
		controllerInit_ = true;

		/* Supply initial values for gain and exposure. */
		ControlList ctrls(unicamCtrls_);
		AgcStatus agcStatus;
		agcStatus.shutter_time = DefaultExposureTime;
		agcStatus.analogue_gain = DefaultAnalogueGain;
		applyAGC(&agcStatus, ctrls);

		result->controls.emplace_back(ctrls);
		result->operation |= RPi::IPA_CONFIG_SENSOR;
	}

	lastMode_ = mode_;
}

void IPARPi::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id, MappedFrameBuffer(&fb, PROT_READ | PROT_WRITE));
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

void IPARPi::processEvent(const IPAOperationData &event)
{
	switch (event.operation) {
	case RPi::IPA_EVENT_SIGNAL_STAT_READY: {
		unsigned int bufferId = event.data[0];

		if (++checkCount_ != frameCount_) /* assert here? */
			LOG(IPARPI, Error) << "WARNING: Prepare/Process mismatch!!!";
		if (frameCount_ > mistrustCount_)
			processStats(bufferId);

		reportMetadata();

		IPAOperationData op;
		op.operation = RPi::IPA_ACTION_STATS_METADATA_COMPLETE;
		op.data = { bufferId & RPi::BufferMask::ID };
		op.controls = { libcameraMetadata_ };
		queueFrameAction.emit(0, op);
		break;
	}

	case RPi::IPA_EVENT_SIGNAL_ISP_PREPARE: {
		unsigned int embeddedbufferId = event.data[0];
		unsigned int bayerbufferId = event.data[1];

		/*
		 * At start-up, or after a mode-switch, we may want to
		 * avoid running the control algos for a few frames in case
		 * they are "unreliable".
		 */
		prepareISP(embeddedbufferId);
		frameCount_++;

		/* Ready to push the input buffer into the ISP. */
		IPAOperationData op;
		op.operation = RPi::IPA_ACTION_RUN_ISP;
		op.data = { bayerbufferId & RPi::BufferMask::ID };
		queueFrameAction.emit(0, op);
		break;
	}

	case RPi::IPA_EVENT_QUEUE_REQUEST: {
		queueRequest(event.controls[0]);
		break;
	}

	default:
		LOG(IPARPI, Error) << "Unknown event " << event.operation;
		break;
	}
}

void IPARPi::reportMetadata()
{
	std::unique_lock<RPiController::Metadata> lock(rpiMetadata_);

	/*
	 * Certain information about the current frame and how it will be
	 * processed can be extracted and placed into the libcamera metadata
	 * buffer, where an application could query it.
	 */
	DeviceStatus *deviceStatus = rpiMetadata_.GetLocked<DeviceStatus>("device.status");
	if (deviceStatus) {
		libcameraMetadata_.set(controls::ExposureTime, deviceStatus->shutter_speed);
		libcameraMetadata_.set(controls::AnalogueGain, deviceStatus->analogue_gain);
	}

	AgcStatus *agcStatus = rpiMetadata_.GetLocked<AgcStatus>("agc.status");
	if (agcStatus)
		libcameraMetadata_.set(controls::AeLocked, agcStatus->locked);

	LuxStatus *luxStatus = rpiMetadata_.GetLocked<LuxStatus>("lux.status");
	if (luxStatus)
		libcameraMetadata_.set(controls::Lux, luxStatus->lux);

	AwbStatus *awbStatus = rpiMetadata_.GetLocked<AwbStatus>("awb.status");
	if (awbStatus) {
		libcameraMetadata_.set(controls::ColourGains, { static_cast<float>(awbStatus->gain_r),
								static_cast<float>(awbStatus->gain_b) });
		libcameraMetadata_.set(controls::ColourTemperature, awbStatus->temperature_K);
	}

	BlackLevelStatus *blackLevelStatus = rpiMetadata_.GetLocked<BlackLevelStatus>("black_level.status");
	if (blackLevelStatus)
		libcameraMetadata_.set(controls::SensorBlackLevels,
				       { static_cast<int32_t>(blackLevelStatus->black_level_r),
					 static_cast<int32_t>(blackLevelStatus->black_level_g),
					 static_cast<int32_t>(blackLevelStatus->black_level_g),
					 static_cast<int32_t>(blackLevelStatus->black_level_b) });

	FocusStatus *focusStatus = rpiMetadata_.GetLocked<FocusStatus>("focus.status");
	if (focusStatus && focusStatus->num == 12) {
		/*
		 * We get a 4x3 grid of regions by default. Calculate the average
		 * FoM over the central two positions to give an overall scene FoM.
		 * This can change later if it is not deemed suitable.
		 */
		int32_t focusFoM = (focusStatus->focus_measures[5] + focusStatus->focus_measures[6]) / 2;
		libcameraMetadata_.set(controls::FocusFoM, focusFoM);
	}

	CcmStatus *ccmStatus = rpiMetadata_.GetLocked<CcmStatus>("ccm.status");
	if (ccmStatus) {
		float m[9];
		for (unsigned int i = 0; i < 9; i++)
			m[i] = ccmStatus->matrix[i];
		libcameraMetadata_.set(controls::ColourCorrectionMatrix, m);
	}
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
	{ controls::AwbAuto, "normal" },
	{ controls::AwbIncandescent, "incandescent" },
	{ controls::AwbTungsten, "tungsten" },
	{ controls::AwbFluorescent, "fluorescent" },
	{ controls::AwbIndoor, "indoor" },
	{ controls::AwbDaylight, "daylight" },
	{ controls::AwbCloudy, "cloudy" },
	{ controls::AwbCustom, "custom" },
};

void IPARPi::queueRequest(const ControlList &controls)
{
	/* Clear the return metadata buffer. */
	libcameraMetadata_.clear();

	for (auto const &ctrl : controls) {
		LOG(IPARPI, Info) << "Request ctrl: "
				  << controls::controls.at(ctrl.first)->name()
				  << " = " << ctrl.second.toString();

		switch (ctrl.first) {
		case controls::AE_ENABLE: {
			RPiController::Algorithm *agc = controller_.GetAlgorithm("agc");
			ASSERT(agc);
			if (ctrl.second.get<bool>() == false)
				agc->Pause();
			else
				agc->Resume();

			libcameraMetadata_.set(controls::AeEnable, ctrl.second.get<bool>());
			break;
		}

		case controls::EXPOSURE_TIME: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);

			/* This expects units of micro-seconds. */
			agc->SetFixedShutter(ctrl.second.get<int32_t>());

			libcameraMetadata_.set(controls::ExposureTime, ctrl.second.get<int32_t>());
			break;
		}

		case controls::ANALOGUE_GAIN: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);
			agc->SetFixedAnalogueGain(ctrl.second.get<float>());

			libcameraMetadata_.set(controls::AnalogueGain,
					       ctrl.second.get<float>());
			break;
		}

		case controls::AE_METERING_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);

			int32_t idx = ctrl.second.get<int32_t>();
			if (MeteringModeTable.count(idx)) {
				agc->SetMeteringMode(MeteringModeTable.at(idx));
				libcameraMetadata_.set(controls::AeMeteringMode, idx);
			} else {
				LOG(IPARPI, Error) << "Metering mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::AE_CONSTRAINT_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);

			int32_t idx = ctrl.second.get<int32_t>();
			if (ConstraintModeTable.count(idx)) {
				agc->SetConstraintMode(ConstraintModeTable.at(idx));
				libcameraMetadata_.set(controls::AeConstraintMode, idx);
			} else {
				LOG(IPARPI, Error) << "Constraint mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::AE_EXPOSURE_MODE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);

			int32_t idx = ctrl.second.get<int32_t>();
			if (ExposureModeTable.count(idx)) {
				agc->SetExposureMode(ExposureModeTable.at(idx));
				libcameraMetadata_.set(controls::AeExposureMode, idx);
			} else {
				LOG(IPARPI, Error) << "Exposure mode " << idx
						   << " not recognised";
			}
			break;
		}

		case controls::EXPOSURE_VALUE: {
			RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);

			/*
			 * The SetEv() method takes in a direct exposure multiplier.
			 * So convert to 2^EV
			 */
			double ev = pow(2.0, ctrl.second.get<float>());
			agc->SetEv(ev);
			libcameraMetadata_.set(controls::ExposureValue,
					       ctrl.second.get<float>());
			break;
		}

		case controls::AWB_ENABLE: {
			RPiController::Algorithm *awb = controller_.GetAlgorithm("awb");
			ASSERT(awb);

			if (ctrl.second.get<bool>() == false)
				awb->Pause();
			else
				awb->Resume();

			libcameraMetadata_.set(controls::AwbEnable,
					       ctrl.second.get<bool>());
			break;
		}

		case controls::AWB_MODE: {
			RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
				controller_.GetAlgorithm("awb"));
			ASSERT(awb);

			int32_t idx = ctrl.second.get<int32_t>();
			if (AwbModeTable.count(idx)) {
				awb->SetMode(AwbModeTable.at(idx));
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
				controller_.GetAlgorithm("awb"));
			ASSERT(awb);

			awb->SetManualGains(gains[0], gains[1]);
			if (gains[0] != 0.0f && gains[1] != 0.0f)
				/* A gain of 0.0f will switch back to auto mode. */
				libcameraMetadata_.set(controls::ColourGains,
						       { gains[0], gains[1] });
			break;
		}

		case controls::BRIGHTNESS: {
			RPiController::ContrastAlgorithm *contrast = dynamic_cast<RPiController::ContrastAlgorithm *>(
				controller_.GetAlgorithm("contrast"));
			ASSERT(contrast);

			contrast->SetBrightness(ctrl.second.get<float>() * 65536);
			libcameraMetadata_.set(controls::Brightness,
					       ctrl.second.get<float>());
			break;
		}

		case controls::CONTRAST: {
			RPiController::ContrastAlgorithm *contrast = dynamic_cast<RPiController::ContrastAlgorithm *>(
				controller_.GetAlgorithm("contrast"));
			ASSERT(contrast);

			contrast->SetContrast(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Contrast,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SATURATION: {
			RPiController::CcmAlgorithm *ccm = dynamic_cast<RPiController::CcmAlgorithm *>(
				controller_.GetAlgorithm("ccm"));
			ASSERT(ccm);

			ccm->SetSaturation(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Saturation,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SHARPNESS: {
			RPiController::SharpenAlgorithm *sharpen = dynamic_cast<RPiController::SharpenAlgorithm *>(
				controller_.GetAlgorithm("sharpen"));
			ASSERT(sharpen);

			sharpen->SetStrength(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Sharpness,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SCALER_CROP: {
			/* We do nothing with this, but should avoid the warning below. */
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
	IPAOperationData op;
	op.operation = RPi::IPA_ACTION_EMBEDDED_COMPLETE;
	op.data = { bufferId & RPi::BufferMask::ID };
	queueFrameAction.emit(0, op);
}

void IPARPi::prepareISP(unsigned int bufferId)
{
	struct DeviceStatus deviceStatus = {};
	bool success = parseEmbeddedData(bufferId, deviceStatus);

	/* Done with embedded data now, return to pipeline handler asap. */
	returnEmbeddedBuffer(bufferId);

	if (success) {
		ControlList ctrls(ispCtrls_);

		rpiMetadata_.Clear();
		rpiMetadata_.Set("device.status", deviceStatus);
		controller_.Prepare(&rpiMetadata_);

		/* Lock the metadata buffer to avoid constant locks/unlocks. */
		std::unique_lock<RPiController::Metadata> lock(rpiMetadata_);

		AwbStatus *awbStatus = rpiMetadata_.GetLocked<AwbStatus>("awb.status");
		if (awbStatus)
			applyAWB(awbStatus, ctrls);

		CcmStatus *ccmStatus = rpiMetadata_.GetLocked<CcmStatus>("ccm.status");
		if (ccmStatus)
			applyCCM(ccmStatus, ctrls);

		AgcStatus *dgStatus = rpiMetadata_.GetLocked<AgcStatus>("agc.status");
		if (dgStatus)
			applyDG(dgStatus, ctrls);

		AlscStatus *lsStatus = rpiMetadata_.GetLocked<AlscStatus>("alsc.status");
		if (lsStatus)
			applyLS(lsStatus, ctrls);

		ContrastStatus *contrastStatus = rpiMetadata_.GetLocked<ContrastStatus>("contrast.status");
		if (contrastStatus)
			applyGamma(contrastStatus, ctrls);

		BlackLevelStatus *blackLevelStatus = rpiMetadata_.GetLocked<BlackLevelStatus>("black_level.status");
		if (blackLevelStatus)
			applyBlackLevel(blackLevelStatus, ctrls);

		GeqStatus *geqStatus = rpiMetadata_.GetLocked<GeqStatus>("geq.status");
		if (geqStatus)
			applyGEQ(geqStatus, ctrls);

		SdnStatus *denoiseStatus = rpiMetadata_.GetLocked<SdnStatus>("sdn.status");
		if (denoiseStatus)
			applyDenoise(denoiseStatus, ctrls);

		SharpenStatus *sharpenStatus = rpiMetadata_.GetLocked<SharpenStatus>("sharpen.status");
		if (sharpenStatus)
			applySharpen(sharpenStatus, ctrls);

		DpcStatus *dpcStatus = rpiMetadata_.GetLocked<DpcStatus>("dpc.status");
		if (dpcStatus)
			applyDPC(dpcStatus, ctrls);

		if (!ctrls.empty()) {
			IPAOperationData op;
			op.operation = RPi::IPA_ACTION_V4L2_SET_ISP;
			op.controls.push_back(ctrls);
			queueFrameAction.emit(0, op);
		}
	}
}

bool IPARPi::parseEmbeddedData(unsigned int bufferId, struct DeviceStatus &deviceStatus)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPARPI, Error) << "Could not find embedded buffer!";
		return false;
	}

	Span<uint8_t> mem = it->second.maps()[0];
	helper_->Parser().SetBufferSize(mem.size());
	RPiController::MdParser::Status status = helper_->Parser().Parse(mem.data());
	if (status != RPiController::MdParser::Status::OK) {
		LOG(IPARPI, Error) << "Embedded Buffer parsing failed, error " << status;
	} else {
		uint32_t exposureLines, gainCode;
		if (helper_->Parser().GetExposureLines(exposureLines) != RPiController::MdParser::Status::OK) {
			LOG(IPARPI, Error) << "Exposure time failed";
			return false;
		}

		deviceStatus.shutter_speed = helper_->Exposure(exposureLines);
		if (helper_->Parser().GetGainCode(gainCode) != RPiController::MdParser::Status::OK) {
			LOG(IPARPI, Error) << "Gain failed";
			return false;
		}

		deviceStatus.analogue_gain = helper_->Gain(gainCode);
		LOG(IPARPI, Debug) << "Metadata - Exposure : "
				   << deviceStatus.shutter_speed << " Gain : "
				   << deviceStatus.analogue_gain;
	}

	return true;
}

void IPARPi::processStats(unsigned int bufferId)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPARPI, Error) << "Could not find stats buffer!";
		return;
	}

	Span<uint8_t> mem = it->second.maps()[0];
	bcm2835_isp_stats *stats = reinterpret_cast<bcm2835_isp_stats *>(mem.data());
	RPiController::StatisticsPtr statistics = std::make_shared<bcm2835_isp_stats>(*stats);
	controller_.Process(statistics, &rpiMetadata_);

	struct AgcStatus agcStatus;
	if (rpiMetadata_.Get("agc.status", agcStatus) == 0) {
		ControlList ctrls(unicamCtrls_);
		applyAGC(&agcStatus, ctrls);

		IPAOperationData op;
		op.operation = RPi::IPA_ACTION_V4L2_SET_STAGGERED;
		op.controls.emplace_back(ctrls);
		queueFrameAction.emit(0, op);
	}
}

void IPARPi::applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls)
{
	const auto gainR = ispCtrls_.find(V4L2_CID_RED_BALANCE);
	if (gainR == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find red gain control";
		return;
	}

	const auto gainB = ispCtrls_.find(V4L2_CID_BLUE_BALANCE);
	if (gainB == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find blue gain control";
		return;
	}

	LOG(IPARPI, Debug) << "Applying WB R: " << awbStatus->gain_r << " B: "
			   << awbStatus->gain_b;

	ctrls.set(V4L2_CID_RED_BALANCE,
		  static_cast<int32_t>(awbStatus->gain_r * 1000));
	ctrls.set(V4L2_CID_BLUE_BALANCE,
		  static_cast<int32_t>(awbStatus->gain_b * 1000));
}

void IPARPi::applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls)
{
	int32_t gainCode = helper_->GainCode(agcStatus->analogue_gain);
	int32_t exposureLines = helper_->ExposureLines(agcStatus->shutter_time);

	if (unicamCtrls_.find(V4L2_CID_ANALOGUE_GAIN) == unicamCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find analogue gain control";
		return;
	}

	if (unicamCtrls_.find(V4L2_CID_EXPOSURE) == unicamCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find exposure control";
		return;
	}

	LOG(IPARPI, Debug) << "Applying AGC Exposure: " << agcStatus->shutter_time
			   << " (Shutter lines: " << exposureLines << ") Gain: "
			   << agcStatus->analogue_gain << " (Gain Code: "
			   << gainCode << ")";

	ctrls.set(V4L2_CID_ANALOGUE_GAIN, gainCode);
	ctrls.set(V4L2_CID_EXPOSURE, exposureLines);
}

void IPARPi::applyDG(const struct AgcStatus *dgStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_DIGITAL_GAIN) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find digital gain control";
		return;
	}

	ctrls.set(V4L2_CID_DIGITAL_GAIN,
		  static_cast<int32_t>(dgStatus->digital_gain * 1000));
}

void IPARPi::applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_CC_MATRIX) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find CCM control";
		return;
	}

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
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_GAMMA) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find Gamma control";
		return;
	}

	struct bcm2835_isp_gamma gamma;
	gamma.enabled = 1;
	for (int i = 0; i < CONTRAST_NUM_POINTS; i++) {
		gamma.x[i] = contrastStatus->points[i].x;
		gamma.y[i] = contrastStatus->points[i].y;
	}

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&gamma),
					    sizeof(gamma) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_GAMMA, c);
}

void IPARPi::applyBlackLevel(const struct BlackLevelStatus *blackLevelStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find black level control";
		return;
	}

	bcm2835_isp_black_level blackLevel;
	blackLevel.enabled = 1;
	blackLevel.black_level_r = blackLevelStatus->black_level_r;
	blackLevel.black_level_g = blackLevelStatus->black_level_g;
	blackLevel.black_level_b = blackLevelStatus->black_level_b;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&blackLevel),
					    sizeof(blackLevel) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL, c);
}

void IPARPi::applyGEQ(const struct GeqStatus *geqStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_GEQ) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find geq control";
		return;
	}

	bcm2835_isp_geq geq;
	geq.enabled = 1;
	geq.offset = geqStatus->offset;
	geq.slope.den = 1000;
	geq.slope.num = 1000 * geqStatus->slope;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&geq),
					    sizeof(geq) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_GEQ, c);
}

void IPARPi::applyDenoise(const struct SdnStatus *denoiseStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_DENOISE) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find denoise control";
		return;
	}

	bcm2835_isp_denoise denoise;
	denoise.enabled = 1;
	denoise.constant = denoiseStatus->noise_constant;
	denoise.slope.num = 1000 * denoiseStatus->noise_slope;
	denoise.slope.den = 1000;
	denoise.strength.num = 1000 * denoiseStatus->strength;
	denoise.strength.den = 1000;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&denoise),
					    sizeof(denoise) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_DENOISE, c);
}

void IPARPi::applySharpen(const struct SharpenStatus *sharpenStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_SHARPEN) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find sharpen control";
		return;
	}

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
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_DPC) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find DPC control";
		return;
	}

	bcm2835_isp_dpc dpc;
	dpc.enabled = 1;
	dpc.strength = dpcStatus->strength;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&dpc),
					    sizeof(dpc) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_DPC, c);
}

void IPARPi::applyLS(const struct AlscStatus *lsStatus, ControlList &ctrls)
{
	if (ispCtrls_.find(V4L2_CID_USER_BCM2835_ISP_LENS_SHADING) == ispCtrls_.end()) {
		LOG(IPARPI, Error) << "Can't find LS control";
		return;
	}

	/*
	 * Program lens shading tables into pipeline.
	 * Choose smallest cell size that won't exceed 63x48 cells.
	 */
	const int cellSizes[] = { 16, 32, 64, 128, 256 };
	unsigned int numCells = ARRAY_SIZE(cellSizes);
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
		.dmabuf = lsTableHandle_.fd(),
		.ref_transform = 0,
		.corner_sampled = 1,
		.gain_format = GAIN_FORMAT_U4P10
	};

	if (!lsTable_ || w * h * 4 * sizeof(uint16_t) > RPi::MaxLsGridSize) {
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

struct ipa_context *ipaCreate()
{
	return new IPAInterfaceWrapper(std::make_unique<IPARPi>());
}

} /* extern "C" */

} /* namespace libcamera */
