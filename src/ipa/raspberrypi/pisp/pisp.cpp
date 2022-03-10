/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Ltd.
 *
 * rpi.cpp - Raspberry Pi Image Processing Algorithms
 */

#include <algorithm>
#include <array>
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
#include <libcamera/ipa/pisp_ipa_interface.h>
#include <libcamera/ipa/raspberrypi.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "backend.h"
#include "frontend.h"

#include "agc_algorithm.hpp"
#include "agc_status.h"
#include "awb_algorithm.hpp"
#include "awb_status.h"
#include "cam_helper.hpp"
#include "controller.hpp"
#include "lux_status.h"

namespace libcamera {

using namespace std::literals::chrono_literals;
using utils::Duration;

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

LOG_DEFINE_CATEGORY(IPAPISP)

using ::PiSP::BackEnd;
using ::PiSP::FrontEnd;

class IPAPiSP : public ipa::PiSP::IPAPiSPInterface
{
public:
	IPAPiSP()
		: controller_(), frameCount_(0), mistrustCount_(0),
		  lastRunTimestamp_(0), fe_(nullptr), be_(nullptr), firstStart_(true)
	{
	}

	~IPAPiSP()
	{
		if (fe_)
			munmap(fe_, sizeof(FrontEnd));
		if (be_)
			munmap(be_, sizeof(BackEnd));
	}

	int init(const IPASettings &settings, const ipa::PiSP::InitConfig &config,
		 ipa::PiSP::SensorConfig *sensorConfig) override;
	void start(const ControlList &controls, ipa::PiSP::StartConfig *startConfig) override;
	void stop() override {}

	int configure(const IPACameraSensorInfo &sensorInfo,
		      const std::map<unsigned int, IPAStream> &streamConfig,
		      const ControlInfoMap &sensorControls,
		      const uint32_t transform,
		      ControlList *controls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void signalIpaPrepare(const ipa::PiSP::PrepareConfig &config) override;

private:
	void setMode(const IPACameraSensorInfo &sensorInfo);
	void reportMetadata();
	bool validateSensorControls();
	void applyControls(const ControlList &controls);
	void prepareController(const ipa::PiSP::PrepareConfig &config);
	void fillDeviceStatus(const ControlList &sensorControls);
	void applyFrameDurations(Duration minFrameDuration, Duration maxFrameDuration);
	void applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls);
	void applyAWB(const struct AwbStatus *awbStatus);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap sensorCtrls_;
	ControlInfoMap ispCtrls_;
	ControlList libcameraMetadata_;

	/* Camera sensor params. */
	CameraMode mode_;

	/* Raspberry Pi controller specific defines. */
	std::unique_ptr<RPiController::CamHelper> helper_;
	RPiController::Controller controller_;
	RPiController::Metadata rpiMetadata_;

	/*
	 * We count frames to decide if the frame must be hidden (e.g. from
	 * display) or mistrusted (i.e. not given to the control algos).
	 */
	uint64_t frameCount_;

	/* How many frames we should avoid running control algos on. */
	unsigned int mistrustCount_;

	/* Number of frames that need to be dropped on startup. */
	unsigned int dropFrameCount_;

	/* Frame timestamp for the last run of the controller. */
	uint64_t lastRunTimestamp_;

	/* Do we run a Controller::process() for this frame? */
	bool preparePending_;

	/* Frontend/Backend objects passed in from the pipeline handler. */
	SharedFD feFD_;
	SharedFD beFD_;
	FrontEnd *fe_;
	BackEnd *be_;

	/* Distinguish the first camera start from others. */
	bool firstStart_;

	/* Frame duration (1/fps) limits. */
	Duration minFrameDuration_;
	Duration maxFrameDuration_;

	/* Maximum gain code for the sensor. */
	uint32_t maxSensorGainCode_;
};

int IPAPiSP::init(const IPASettings &settings, const ipa::PiSP::InitConfig &config,
		  ipa::PiSP::SensorConfig *sensorConfig)
{
	/*
	 * Load the "helper" for this sensor. This tells us all the device specific stuff
	 * that the kernel driver doesn't. We only do this the first time; we don't need
	 * to re-parse the metadata after a simple mode-switch for no reason.
	 */
	helper_ = std::unique_ptr<RPiController::CamHelper>(RPiController::CamHelper::Create(settings.sensorModel));
	if (!helper_) {
		LOG(IPAPISP, Error) << "Could not create camera helper for "
				    << settings.sensorModel;
		return -EINVAL;
	}

	/*
	 * Pass out the sensor config to the pipeline handler in order
	 * to setup the staggered writer class.
	 */
	int gainDelay, exposureDelay, vblankDelay, sensorMetadata;
	helper_->GetDelays(exposureDelay, gainDelay, vblankDelay);
	sensorMetadata = helper_->SensorEmbeddedDataPresent();

	sensorConfig->gainDelay = gainDelay;
	sensorConfig->exposureDelay = exposureDelay;
	sensorConfig->vblankDelay = vblankDelay;
	sensorConfig->sensorMetadata = sensorMetadata;

	/* Load the tuning file for this sensor. */
	controller_.Read(settings.configurationFile.c_str());
	controller_.Initialise();

	/* Acquire the Frontend and Backend objects. */
	feFD_ = std::move(config.fe);
	beFD_ = std::move(config.be);

	if (!feFD_.isValid() || !beFD_.isValid()) {
		LOG(IPAPISP, Error) << "Invalid FE/BE handles!";
		return -ENODEV;
	}
	
	fe_ = static_cast<FrontEnd *>
		(mmap(nullptr, sizeof(FrontEnd), PROT_READ | PROT_WRITE, MAP_SHARED, feFD_.get(), 0));
	be_ = static_cast<BackEnd *>
		(mmap(nullptr, sizeof(BackEnd), PROT_READ | PROT_WRITE, MAP_SHARED, beFD_.get(), 0));
	
	if (!fe_ || !be_) {
		LOG(IPAPISP, Error) << "Unable to map FE/BE handles!";
		return -ENODEV;
	}

	return 0;
}

void IPAPiSP::start(const ControlList &controls, ipa::PiSP::StartConfig *startConfig)
{
	RPiController::Metadata metadata;

	ASSERT(startConfig);
	if (!controls.empty()) {
		/* We have been given some controls to action before start. */
		applyControls(controls);
	}

	controller_.SwitchMode(mode_, &metadata);

	/* SwitchMode may supply updated exposure/gain values to use. */
	AgcStatus agcStatus;
	agcStatus.shutter_time = 0.0s;
	agcStatus.analogue_gain = 0.0;

	metadata.Get("agc.status", agcStatus);
	if (agcStatus.shutter_time && agcStatus.analogue_gain) {
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
	if (firstStart_) {
		dropFrameCount_ = helper_->HideFramesStartup();
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

		dropFrameCount_ = std::max({ dropFrameCount_, agcConvergenceFrames, awbConvergenceFrames });
		LOG(IPAPISP, Debug) << "Drop " << dropFrameCount_ << " frames on startup";
	} else {
		dropFrameCount_ = helper_->HideFramesModeSwitch();
		mistrustCount_ = helper_->MistrustFramesModeSwitch();
	}

	startConfig->dropFrameCount = dropFrameCount_;

	firstStart_ = false;
	lastRunTimestamp_ = 0;
}


void IPAPiSP::setMode(const IPACameraSensorInfo &sensorInfo)
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
	 * Calculate the line length as the ratio between the line length in
	 * pixels and the pixel rate.
	 */
	mode_.line_length = sensorInfo.lineLength * (1.0s / sensorInfo.pixelRate);

	/*
	 * Set the frame length limits for the mode to ensure exposure and
	 * framerate calculations are clipped appropriately.
	 */
	mode_.min_frame_length = sensorInfo.minFrameLength;
	mode_.max_frame_length = sensorInfo.maxFrameLength;

	/*
	 * Some sensors may have different sensitivities in different modes;
	 * the CamHelper will know the correct value.
	 */
	mode_.sensitivity = helper_->GetModeSensitivity(mode_);
}

int IPAPiSP::configure(const IPACameraSensorInfo &sensorInfo,
		       [[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
		       const ControlInfoMap &sensorControls, const uint32_t transform,
		       ControlList *controls)
{
	sensorCtrls_ = std::move(sensorControls);

	if (!validateSensorControls()) {
		LOG(IPAPISP, Error) << "Sensor control validation failed.";
		return -1;
	}

	maxSensorGainCode_ = sensorCtrls_.at(V4L2_CID_ANALOGUE_GAIN).max().get<int32_t>();

	/* Setup a metadata ControlList to output metadata. */
	libcameraMetadata_ = ControlList(controls::controls);

	/* Re-assemble camera mode using the sensor info. */
	setMode(sensorInfo);

	mode_.transform = static_cast<libcamera::Transform>(transform);

	/* Pass the camera mode to the CamHelper to setup algorithms. */
	helper_->SetCameraMode(mode_);

	/*
	 * Initialise this ControlList correctly, even if empty, in case the IPA is
	 * running is isolation mode (passing the ControlList through the IPC layer).
	 */
	ControlList ctrls(sensorCtrls_);

	if (firstStart_) {
		/* Supply initial values for frame durations. */
		applyFrameDurations(defaultMinFrameDuration, defaultMaxFrameDuration);

		/* Supply initial values for gain and exposure. */
		AgcStatus agcStatus;
		agcStatus.shutter_time = defaultExposureTime;
		agcStatus.analogue_gain = defaultAnalogueGain;
		applyAGC(&agcStatus, ctrls);
	}

	ASSERT(controls);
	*controls = std::move(ctrls);

	return 0;
}

void IPAPiSP::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

void IPAPiSP::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(id);
	}
}

void IPAPiSP::signalIpaPrepare(const ipa::PiSP::PrepareConfig &config)
{
	if (!config.requestControls.empty())
		applyControls(config.requestControls);

	/*
	 * At start-up, or after a mode-switch, we may want to
	 * avoid running the control algos for a few frames in case
	 * they are "unreliable".
	 */
	if (frameCount_ > mistrustCount_)
		prepareController(config);

	frameCount_++;

	/* Ready to push the input buffer into the ISP. */
	ipaPrepareComplete.emit(config.buffers, libcameraMetadata_);
}

void IPAPiSP::reportMetadata()
{
	std::unique_lock<RPiController::Metadata> lock(rpiMetadata_);

	/*
	 * Certain information about the current frame and how it will be
	 * processed can be extracted and placed into the libcamera metadata
	 * buffer, where an application could query it.
	 */
	DeviceStatus *deviceStatus = rpiMetadata_.GetLocked<DeviceStatus>("device.status");
	if (deviceStatus) {
		libcameraMetadata_.set(controls::ExposureTime,
				       deviceStatus->shutter_speed.get<std::micro>());
		libcameraMetadata_.set(controls::AnalogueGain, deviceStatus->analogue_gain);
		libcameraMetadata_.set(controls::FrameDuration,
				       helper_->Exposure(deviceStatus->frame_length).get<std::micro>());
	}

	AgcStatus *agcStatus = rpiMetadata_.GetLocked<AgcStatus>("agc.status");
	if (agcStatus) {
		libcameraMetadata_.set(controls::AeLocked, agcStatus->locked);
		libcameraMetadata_.set(controls::DigitalGain, agcStatus->digital_gain);
	}

	LuxStatus *luxStatus = rpiMetadata_.GetLocked<LuxStatus>("lux.status");
	if (luxStatus)
		libcameraMetadata_.set(controls::Lux, luxStatus->lux);

	AwbStatus *awbStatus = rpiMetadata_.GetLocked<AwbStatus>("awb.status");
	if (awbStatus) {
		libcameraMetadata_.set(controls::ColourGains, { static_cast<float>(awbStatus->gain_r),
								static_cast<float>(awbStatus->gain_b) });
		libcameraMetadata_.set(controls::ColourTemperature, awbStatus->temperature_K);
	}
}

bool IPAPiSP::validateSensorControls()
{
	static const uint32_t ctrls[] = {
		V4L2_CID_ANALOGUE_GAIN,
		V4L2_CID_EXPOSURE,
		V4L2_CID_VBLANK,
	};

	for (auto c : ctrls) {
		if (sensorCtrls_.find(c) == sensorCtrls_.end()) {
			LOG(IPAPISP, Error) << "Unable to find sensor control "
					   << utils::hex(c);
			return false;
		}
	}

	return true;
}

void IPAPiSP::applyControls(const ControlList &controls)
{
	/* Clear the return metadata buffer. */
	libcameraMetadata_.clear();

	for (auto const &ctrl : controls) {
		LOG(IPAPISP, Info) << "Request ctrl: "
				  << controls::controls.at(ctrl.first)->name()
				  << " = " << ctrl.second.toString();

		switch (ctrl.first) {
		case controls::SCALER_CROP: {
			/* We do nothing with this, but should avoid the warning below. */
			break;
		}

		case controls::FRAME_DURATION_LIMITS: {
			auto frameDurations = ctrl.second.get<Span<const int64_t>>();
			applyFrameDurations(frameDurations[0] * 1.0us, frameDurations[1] * 1.0us);
			break;
		}

		default:
			LOG(IPAPISP, Warning)
				<< "Ctrl " << controls::controls.at(ctrl.first)->name()
				<< " is not handled.";
			break;
		}
	}
}

void IPAPiSP::prepareController(const ipa::PiSP::PrepareConfig &config)
{
	int64_t frameTimestamp = config.sensorControls.get(controls::SensorTimestamp);
	RPiController::Metadata lastMetadata;
	Span<uint8_t> embeddedBuffer;

	lastMetadata = std::move(rpiMetadata_);
	fillDeviceStatus(config.sensorControls);

	if (config.embeddedBufferPresent) {
		/*
		 * Pipeline handler has supplied us with an embedded data buffer,
		 * we must pass it to the CamHelper for parsing.
		 */
		auto it = buffers_.find(config.buffers.embeddedBufferId);
		ASSERT(it != buffers_.end());
		embeddedBuffer = it->second.planes()[0];
	}

	/*
	 * This may overwrite the DeviceStatus using values from the sensor
	 * metadata, and may also do additional custom processing.
	 */
	helper_->Prepare(embeddedBuffer, rpiMetadata_);

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
		rpiMetadata_.Merge(lastMetadata);
		preparePending_ = false;
		return;
	}

	lastRunTimestamp_ = frameTimestamp;
	preparePending_ = true;

	controller_.Prepare(&rpiMetadata_);

	/* Lock the metadata buffer to avoid constant locks/unlocks. */
	std::unique_lock<RPiController::Metadata> lock(rpiMetadata_);

	AwbStatus *awbStatus = rpiMetadata_.GetLocked<AwbStatus>("awb.status");
	if (awbStatus)
		applyAWB(awbStatus);

	AgcStatus *agcStatus = rpiMetadata_.GetLocked<AgcStatus>("agc.status");
	if (agcStatus) {
		ControlList ctrls(sensorCtrls_);
		applyAGC(agcStatus, ctrls);
		setDelayedControls.emit(ctrls);
	}
}

void IPAPiSP::fillDeviceStatus(const ControlList &sensorControls)
{
	DeviceStatus deviceStatus = {};

	int32_t exposureLines = sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	int32_t gainCode = sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();
	int32_t vblank = sensorControls.get(V4L2_CID_VBLANK).get<int32_t>();

	deviceStatus.shutter_speed = helper_->Exposure(exposureLines);
	deviceStatus.analogue_gain = helper_->Gain(gainCode);
	deviceStatus.frame_length = mode_.height + vblank;

	LOG(IPAPISP, Debug) << "Metadata - " << deviceStatus;

	rpiMetadata_.Set("device.status", deviceStatus);
}

void IPAPiSP::applyAWB(const struct AwbStatus *awbStatus)
{
	LOG(IPAPISP, Debug) << "Applying WB R: " << awbStatus->gain_r << " B: "
			    << awbStatus->gain_b;
}

void IPAPiSP::applyFrameDurations(Duration minFrameDuration, Duration maxFrameDuration)
{
	const Duration minSensorFrameDuration = mode_.min_frame_length * mode_.line_length;
	const Duration maxSensorFrameDuration = mode_.max_frame_length * mode_.line_length;

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
	 * GetVBlanking() will update maxShutter with the largest exposure
	 * value possible.
	 */
	Duration maxShutter = Duration::max();
	helper_->GetVBlanking(maxShutter, minFrameDuration_, maxFrameDuration_);

	RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
		controller_.GetAlgorithm("agc"));
	agc->SetMaxShutter(maxShutter);
}

void IPAPiSP::applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls)
{
	int32_t gainCode = helper_->GainCode(agcStatus->analogue_gain);

	/*
	 * Ensure anything larger than the max gain code will not be passed to
	 * DelayedControls. The AGC will correctly handle a lower gain returned
	 * by the sensor, provided it knows the actual gain used.
	 */
	gainCode = std::min<int32_t>(gainCode, maxSensorGainCode_);

	/* GetVBlanking might clip exposure time to the fps limits. */
	Duration exposure = agcStatus->shutter_time;
	int32_t vblanking = helper_->GetVBlanking(exposure, minFrameDuration_, maxFrameDuration_);
	int32_t exposureLines = helper_->ExposureLines(exposure);

	LOG(IPAPISP, Debug) << "Applying AGC Exposure: " << exposure
			   << " (Shutter lines: " << exposureLines << ", AGC requested "
			   << agcStatus->shutter_time << ") Gain: "
			   << agcStatus->analogue_gain << " (Gain Code: "
			   << gainCode << ")";

	/*
	 * Due to the behavior of V4L2, the current value of VBLANK could clip the
	 * exposure time without us knowing. The next time though this function should
	 * clip exposure correctly.
	 */
	ctrls.set(V4L2_CID_VBLANK, vblanking);
	ctrls.set(V4L2_CID_EXPOSURE, exposureLines);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, gainCode);
}

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerPiSP",
	"PiSP",
};

IPAInterface *ipaCreate()
{
	return new IPAPiSP();
}

} /* extern "C" */

} /* namespace libcamera */
