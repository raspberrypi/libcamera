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
#define DEFAULT_ANALOGUE_GAIN 1.0
#define DEFAULT_EXPOSURE_TIME 20000

LOG_DEFINE_CATEGORY(IPARPI)

class IPARPi : public IPAInterface
{
public:
	IPARPi()
		: lastMode_({}), controller_(), controllerInit_(false),
		  frame_count_(0), check_count_(0), hide_count_(0),
		  mistrust_count_(0), lsTable_(nullptr)
	{
	}

	~IPARPi()
	{
		if (lsTable_)
			munmap(lsTable_, MAX_LS_GRID_SIZE);
	}

	int init(const IPASettings &settings) override;
	int start() override { return 0; }
	void stop() override {}

	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &data,
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
	void resampleTable(uint16_t dest[], double const src[12][16], int dest_w, int dest_h);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, void *> buffersMemory_;

	ControlInfoMap unicam_ctrls_;
	ControlInfoMap isp_ctrls_;
	ControlList libcameraMetadata_;

	/* IPA configuration. */
	std::string tuningFile_;

	/* Camera sensor params. */
	CameraMode mode_;
	CameraMode lastMode_;

	/* Raspberry Pi controller specific defines. */
	std::unique_ptr<RPi::CamHelper> helper_;
	RPi::Controller controller_;
	bool controllerInit_;
	RPi::Metadata rpiMetadata_;

	/*
	 * We count frames to decide if the frame must be hidden (e.g. from
	 * display) or mistrusted (i.e. not given to the control algos).
	 */
	uint64_t frame_count_;
	/* For checking the sequencing of Prepare/Process calls. */
	uint64_t check_count_;
	/* How many frames the pipeline handler should hide, or "drop". */
	unsigned int hide_count_;
	/* How many frames we should avoid running control algos on. */
	unsigned int mistrust_count_;
	/* LS table allocation passed in from the pipeline handler. */
	FileDescriptor lsTableHandle_;
	void *lsTable_;
};

int IPARPi::init(const IPASettings &settings)
{
	tuningFile_ = settings.configurationFile;
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
	mode_.bin_y = std::min(2, static_cast<int>(mode_.scale_x));
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
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result)
{
	if (entityControls.empty())
		return;

	result->operation = 0;

	unicam_ctrls_ = entityControls.at(0);
	isp_ctrls_ = entityControls.at(1);
	/* Setup a metadata ControlList to output metadata. */
	libcameraMetadata_ = ControlList(controls::controls);

	/*
	 * Load the "helper" for this sensor. This tells us all the device specific stuff
	 * that the kernel driver doesn't. We only do this the first time; we don't need
	 * to re-parse the metadata after a simple mode-switch for no reason.
	 */
	std::string cameraName(sensorInfo.model);
	if (!helper_) {
		helper_ = std::unique_ptr<RPi::CamHelper>(RPi::CamHelper::Create(cameraName));
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

		result->operation |= RPI_IPA_CONFIG_STAGGERED_WRITE;
	}

	/* Re-assemble camera mode using the sensor info. */
	setMode(sensorInfo);

	/* Pass the camera mode to the CamHelper to setup algorithms. */
	helper_->SetCameraMode(mode_);

	/*
	 * Initialise frame counts, and decide how many frames must be hidden or
	 *"mistrusted", which depends on whether this is a startup from cold,
	 * or merely a mode switch in a running system.
	 */
	frame_count_ = 0;
	check_count_ = 0;
	if (controllerInit_) {
		hide_count_ = helper_->HideFramesModeSwitch();
		mistrust_count_ = helper_->MistrustFramesModeSwitch();
	} else {
		hide_count_ = helper_->HideFramesStartup();
		mistrust_count_ = helper_->MistrustFramesStartup();
	}

	struct AgcStatus agcStatus;
	/* These zero values mean not program anything (unless overwritten). */
	agcStatus.shutter_time = 0.0;
	agcStatus.analogue_gain = 0.0;

	if (!controllerInit_) {
		/* Load the tuning file for this sensor. */
		controller_.Read(tuningFile_.c_str());
		controller_.Initialise();
		controllerInit_ = true;

		/* Supply initial values for gain and exposure. */
		agcStatus.shutter_time = DEFAULT_EXPOSURE_TIME;
		agcStatus.analogue_gain = DEFAULT_ANALOGUE_GAIN;
	}

	RPi::Metadata metadata;
	controller_.SwitchMode(mode_, &metadata);

	/* SwitchMode may supply updated exposure/gain values to use. */
	metadata.Get("agc.status", agcStatus);
	if (agcStatus.shutter_time != 0.0 && agcStatus.analogue_gain != 0.0) {
		ControlList ctrls(unicam_ctrls_);
		applyAGC(&agcStatus, ctrls);
		result->controls.push_back(ctrls);

		result->operation |= RPI_IPA_CONFIG_SENSOR;
	}

	lastMode_ = mode_;

	/* Store the lens shading table pointer and handle if available. */
	if (ipaConfig.operation & RPI_IPA_CONFIG_LS_TABLE) {
		/* Remove any previous table, if there was one. */
		if (lsTable_) {
			munmap(lsTable_, MAX_LS_GRID_SIZE);
			lsTable_ = nullptr;
		}

		/* Map the LS table buffer into user space. */
		lsTableHandle_ = FileDescriptor(ipaConfig.data[0]);
		if (lsTableHandle_.isValid()) {
			lsTable_ = mmap(nullptr, MAX_LS_GRID_SIZE, PROT_READ | PROT_WRITE,
					MAP_SHARED, lsTableHandle_.fd(), 0);

			if (lsTable_ == MAP_FAILED) {
				LOG(IPARPI, Error) << "dmaHeap mmap failure for LS table.";
				lsTable_ = nullptr;
			}
		}
	}
}

void IPARPi::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		buffersMemory_[buffer.id] = mmap(nullptr, fb.planes()[0].length,
						 PROT_READ | PROT_WRITE, MAP_SHARED,
						 fb.planes()[0].fd.fd(), 0);

		if (buffersMemory_[buffer.id] == MAP_FAILED) {
			int ret = -errno;
			LOG(IPARPI, Fatal) << "Failed to mmap buffer: " << strerror(-ret);
		}
	}
}

void IPARPi::unmapBuffers(const std::vector<unsigned int> &ids)
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

void IPARPi::processEvent(const IPAOperationData &event)
{
	switch (event.operation) {
	case RPI_IPA_EVENT_SIGNAL_STAT_READY: {
		unsigned int bufferId = event.data[0];

		if (++check_count_ != frame_count_) /* assert here? */
			LOG(IPARPI, Error) << "WARNING: Prepare/Process mismatch!!!";
		if (frame_count_ > mistrust_count_)
			processStats(bufferId);

		reportMetadata();

		IPAOperationData op;
		op.operation = RPI_IPA_ACTION_STATS_METADATA_COMPLETE;
		op.data = { bufferId & RPiIpaMask::ID };
		op.controls = { libcameraMetadata_ };
		queueFrameAction.emit(0, op);
		break;
	}

	case RPI_IPA_EVENT_SIGNAL_ISP_PREPARE: {
		unsigned int embeddedbufferId = event.data[0];
		unsigned int bayerbufferId = event.data[1];

		/*
		 * At start-up, or after a mode-switch, we may want to
		 * avoid running the control algos for a few frames in case
		 * they are "unreliable".
		 */
		prepareISP(embeddedbufferId);

		/* Ready to push the input buffer into the ISP. */
		IPAOperationData op;
		if (++frame_count_ > hide_count_)
			op.operation = RPI_IPA_ACTION_RUN_ISP;
		else
			op.operation = RPI_IPA_ACTION_RUN_ISP_AND_DROP_FRAME;
		op.data = { bayerbufferId & RPiIpaMask::ID };
		queueFrameAction.emit(0, op);
		break;
	}

	case RPI_IPA_EVENT_QUEUE_REQUEST: {
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
	std::unique_lock<RPi::Metadata> lock(rpiMetadata_);

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
			RPi::Algorithm *agc = controller_.GetAlgorithm("agc");
			ASSERT(agc);
			if (ctrl.second.get<bool>() == false)
				agc->Pause();
			else
				agc->Resume();

			libcameraMetadata_.set(controls::AeEnable, ctrl.second.get<bool>());
			break;
		}

		case controls::EXPOSURE_TIME: {
			RPi::AgcAlgorithm *agc = dynamic_cast<RPi::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);
			/* This expects units of micro-seconds. */
			agc->SetFixedShutter(ctrl.second.get<int32_t>());
			/* For the manual values to take effect, AGC must be unpaused. */
			if (agc->IsPaused())
				agc->Resume();

			libcameraMetadata_.set(controls::ExposureTime, ctrl.second.get<int32_t>());
			break;
		}

		case controls::ANALOGUE_GAIN: {
			RPi::AgcAlgorithm *agc = dynamic_cast<RPi::AgcAlgorithm *>(
				controller_.GetAlgorithm("agc"));
			ASSERT(agc);
			agc->SetFixedAnalogueGain(ctrl.second.get<float>());
			/* For the manual values to take effect, AGC must be unpaused. */
			if (agc->IsPaused())
				agc->Resume();

			libcameraMetadata_.set(controls::AnalogueGain,
					       ctrl.second.get<float>());
			break;
		}

		case controls::AE_METERING_MODE: {
			RPi::AgcAlgorithm *agc = dynamic_cast<RPi::AgcAlgorithm *>(
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
			RPi::AgcAlgorithm *agc = dynamic_cast<RPi::AgcAlgorithm *>(
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
			RPi::AgcAlgorithm *agc = dynamic_cast<RPi::AgcAlgorithm *>(
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
			RPi::AgcAlgorithm *agc = dynamic_cast<RPi::AgcAlgorithm *>(
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
			RPi::Algorithm *awb = controller_.GetAlgorithm("awb");
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
			RPi::AwbAlgorithm *awb = dynamic_cast<RPi::AwbAlgorithm *>(
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
			RPi::AwbAlgorithm *awb = dynamic_cast<RPi::AwbAlgorithm *>(
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
			RPi::ContrastAlgorithm *contrast = dynamic_cast<RPi::ContrastAlgorithm *>(
				controller_.GetAlgorithm("contrast"));
			ASSERT(contrast);

			contrast->SetBrightness(ctrl.second.get<float>() * 65536);
			libcameraMetadata_.set(controls::Brightness,
					       ctrl.second.get<float>());
			break;
		}

		case controls::CONTRAST: {
			RPi::ContrastAlgorithm *contrast = dynamic_cast<RPi::ContrastAlgorithm *>(
				controller_.GetAlgorithm("contrast"));
			ASSERT(contrast);

			contrast->SetContrast(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Contrast,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SATURATION: {
			RPi::CcmAlgorithm *ccm = dynamic_cast<RPi::CcmAlgorithm *>(
				controller_.GetAlgorithm("ccm"));
			ASSERT(ccm);

			ccm->SetSaturation(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Saturation,
					       ctrl.second.get<float>());
			break;
		}

		case controls::SHARPNESS: {
			RPi::SharpenAlgorithm *sharpen = dynamic_cast<RPi::SharpenAlgorithm *>(
				controller_.GetAlgorithm("sharpen"));
			ASSERT(sharpen);

			sharpen->SetStrength(ctrl.second.get<float>());
			libcameraMetadata_.set(controls::Sharpness,
					       ctrl.second.get<float>());
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
	op.operation = RPI_IPA_ACTION_EMBEDDED_COMPLETE;
	op.data = { bufferId & RPiIpaMask::ID };
	queueFrameAction.emit(0, op);
}

void IPARPi::prepareISP(unsigned int bufferId)
{
	struct DeviceStatus deviceStatus = {};
	bool success = parseEmbeddedData(bufferId, deviceStatus);

	/* Done with embedded data now, return to pipeline handler asap. */
	returnEmbeddedBuffer(bufferId);

	if (success) {
		ControlList ctrls(isp_ctrls_);

		rpiMetadata_.Clear();
		rpiMetadata_.Set("device.status", deviceStatus);
		controller_.Prepare(&rpiMetadata_);

		/* Lock the metadata buffer to avoid constant locks/unlocks. */
		std::unique_lock<RPi::Metadata> lock(rpiMetadata_);

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
			op.operation = RPI_IPA_ACTION_V4L2_SET_ISP;
			op.controls.push_back(ctrls);
			queueFrameAction.emit(0, op);
		}
	}
}

bool IPARPi::parseEmbeddedData(unsigned int bufferId, struct DeviceStatus &deviceStatus)
{
	auto it = buffersMemory_.find(bufferId);
	if (it == buffersMemory_.end()) {
		LOG(IPARPI, Error) << "Could not find embedded buffer!";
		return false;
	}

	int size = buffers_.find(bufferId)->second.planes()[0].length;
	helper_->Parser().SetBufferSize(size);
	RPi::MdParser::Status status = helper_->Parser().Parse(it->second);
	if (status != RPi::MdParser::Status::OK) {
		LOG(IPARPI, Error) << "Embedded Buffer parsing failed, error " << status;
	} else {
		uint32_t exposure_lines, gain_code;
		if (helper_->Parser().GetExposureLines(exposure_lines) != RPi::MdParser::Status::OK) {
			LOG(IPARPI, Error) << "Exposure time failed";
			return false;
		}

		deviceStatus.shutter_speed = helper_->Exposure(exposure_lines);
		if (helper_->Parser().GetGainCode(gain_code) != RPi::MdParser::Status::OK) {
			LOG(IPARPI, Error) << "Gain failed";
			return false;
		}

		deviceStatus.analogue_gain = helper_->Gain(gain_code);
		LOG(IPARPI, Debug) << "Metadata - Exposure : "
				   << deviceStatus.shutter_speed << " Gain : "
				   << deviceStatus.analogue_gain;
	}

	return true;
}

void IPARPi::processStats(unsigned int bufferId)
{
	auto it = buffersMemory_.find(bufferId);
	if (it == buffersMemory_.end()) {
		LOG(IPARPI, Error) << "Could not find stats buffer!";
		return;
	}

	bcm2835_isp_stats *stats = static_cast<bcm2835_isp_stats *>(it->second);
	RPi::StatisticsPtr statistics = std::make_shared<bcm2835_isp_stats>(*stats);
	controller_.Process(statistics, &rpiMetadata_);

	struct AgcStatus agcStatus;
	if (rpiMetadata_.Get("agc.status", agcStatus) == 0) {
		ControlList ctrls(unicam_ctrls_);
		applyAGC(&agcStatus, ctrls);

		IPAOperationData op;
		op.operation = RPI_IPA_ACTION_V4L2_SET_STAGGERED;
		op.controls.push_back(ctrls);
		queueFrameAction.emit(0, op);
	}
}

void IPARPi::applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls)
{
	const auto gainR = isp_ctrls_.find(V4L2_CID_RED_BALANCE);
	if (gainR == isp_ctrls_.end()) {
		LOG(IPARPI, Error) << "Can't find red gain control";
		return;
	}

	const auto gainB = isp_ctrls_.find(V4L2_CID_BLUE_BALANCE);
	if (gainB == isp_ctrls_.end()) {
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
	int32_t gain_code = helper_->GainCode(agcStatus->analogue_gain);
	int32_t exposure_lines = helper_->ExposureLines(agcStatus->shutter_time);

	if (unicam_ctrls_.find(V4L2_CID_ANALOGUE_GAIN) == unicam_ctrls_.end()) {
		LOG(IPARPI, Error) << "Can't find analogue gain control";
		return;
	}

	if (unicam_ctrls_.find(V4L2_CID_EXPOSURE) == unicam_ctrls_.end()) {
		LOG(IPARPI, Error) << "Can't find exposure control";
		return;
	}

	LOG(IPARPI, Debug) << "Applying AGC Exposure: " << agcStatus->shutter_time
			   << " (Shutter lines: " << exposure_lines << ") Gain: "
			   << agcStatus->analogue_gain << " (Gain Code: "
			   << gain_code << ")";

	ctrls.set(V4L2_CID_ANALOGUE_GAIN, gain_code);
	ctrls.set(V4L2_CID_EXPOSURE, exposure_lines);
}

void IPARPi::applyDG(const struct AgcStatus *dgStatus, ControlList &ctrls)
{
	if (isp_ctrls_.find(V4L2_CID_DIGITAL_GAIN) == isp_ctrls_.end()) {
		LOG(IPARPI, Error) << "Can't find digital gain control";
		return;
	}

	ctrls.set(V4L2_CID_DIGITAL_GAIN,
		  static_cast<int32_t>(dgStatus->digital_gain * 1000));
}

void IPARPi::applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls)
{
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_CC_MATRIX) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_GAMMA) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_GEQ) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_DENOISE) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_SHARPEN) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_DPC) == isp_ctrls_.end()) {
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
	if (isp_ctrls_.find(V4L2_CID_USER_BCM2835_ISP_LENS_SHADING) == isp_ctrls_.end()) {
		LOG(IPARPI, Error) << "Can't find LS control";
		return;
	}

	/*
	 * Program lens shading tables into pipeline.
	 * Choose smallest cell size that won't exceed 63x48 cells.
	 */
	const int cell_sizes[] = { 16, 32, 64, 128, 256 };
	unsigned int num_cells = ARRAY_SIZE(cell_sizes);
	unsigned int i, w, h, cell_size;
	for (i = 0; i < num_cells; i++) {
		cell_size = cell_sizes[i];
		w = (mode_.width + cell_size - 1) / cell_size;
		h = (mode_.height + cell_size - 1) / cell_size;
		if (w < 64 && h <= 48)
			break;
	}

	if (i == num_cells) {
		LOG(IPARPI, Error) << "Cannot find cell size";
		return;
	}

	/* We're going to supply corner sampled tables, 16 bit samples. */
	w++, h++;
	bcm2835_isp_lens_shading ls = {
		.enabled = 1,
		.grid_cell_size = cell_size,
		.grid_width = w,
		.grid_stride = w,
		.grid_height = h,
		.dmabuf = lsTableHandle_.fd(),
		.ref_transform = 0,
		.corner_sampled = 1,
		.gain_format = GAIN_FORMAT_U4P10
	};

	if (!lsTable_ || w * h * 4 * sizeof(uint16_t) > MAX_LS_GRID_SIZE) {
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
 * Resamples a 16x12 table with central sampling to dest_w x dest_h with corner
 * sampling.
 */
void IPARPi::resampleTable(uint16_t dest[], double const src[12][16],
			   int dest_w, int dest_h)
{
	/*
	 * Precalculate and cache the x sampling locations and phases to
	 * save recomputing them on every row.
	 */
	assert(dest_w > 1 && dest_h > 1 && dest_w <= 64);
	int x_lo[64], x_hi[64];
	double xf[64];
	double x = -0.5, x_inc = 16.0 / (dest_w - 1);
	for (int i = 0; i < dest_w; i++, x += x_inc) {
		x_lo[i] = floor(x);
		xf[i] = x - x_lo[i];
		x_hi[i] = x_lo[i] < 15 ? x_lo[i] + 1 : 15;
		x_lo[i] = x_lo[i] > 0 ? x_lo[i] : 0;
	}

	/* Now march over the output table generating the new values. */
	double y = -0.5, y_inc = 12.0 / (dest_h - 1);
	for (int j = 0; j < dest_h; j++, y += y_inc) {
		int y_lo = floor(y);
		double yf = y - y_lo;
		int y_hi = y_lo < 11 ? y_lo + 1 : 11;
		y_lo = y_lo > 0 ? y_lo : 0;
		double const *row_above = src[y_lo];
		double const *row_below = src[y_hi];
		for (int i = 0; i < dest_w; i++) {
			double above = row_above[x_lo[i]] * (1 - xf[i])
				     + row_above[x_hi[i]] * xf[i];
			double below = row_below[x_lo[i]] * (1 - xf[i])
				     + row_below[x_hi[i]] * xf[i];
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

}; /* extern "C" */

} /* namespace libcamera */
