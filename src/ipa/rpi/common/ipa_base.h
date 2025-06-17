/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * Raspberry Pi IPA base class
 */
#pragma once

#include <array>
#include <deque>
#include <map>
#include <stdint.h>

#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include <libcamera/ipa/raspberrypi_ipa_interface.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "cam_helper/cam_helper.h"
#include "controller/agc_status.h"
#include "controller/camera_mode.h"
#include "controller/controller.h"
#include "controller/hdr_status.h"
#include "controller/metadata.h"

namespace libcamera {

namespace ipa::RPi {

class IpaBase : public IPARPiInterface
{
public:
	IpaBase();
	~IpaBase();

	int32_t init(const IPASettings &settings, const InitParams &params, InitResult *result) override;
	int32_t configure(const IPACameraSensorInfo &sensorInfo, const ConfigParams &params,
			  ConfigResult *result) override;

	void start(const ControlList &controls, StartResult *result) override;
	void stop() override {}

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void prepareIsp(const PrepareParams &params) override;
	void processStats(const ProcessParams &params) override;

protected:
	bool monoSensor() const
	{
		return monoSensor_;
	}

	/* Raspberry Pi controller specific defines. */
	std::unique_ptr<RPiController::CamHelper> helper_;
	RPiController::Controller controller_;

	ControlInfoMap sensorCtrls_;
	ControlInfoMap lensCtrls_;

	/* Camera sensor params. */
	CameraMode mode_;

	/* Track the frame length times over FrameLengthsQueueSize frames. */
	std::deque<utils::Duration> frameLengths_;
	utils::Duration lastTimeout_;
	ControlList libcameraMetadata_;
	bool statsMetadataOutput_;

	/* Remember the HDR status after a mode switch. */
	HdrStatus hdrStatus_;

	/* Whether the stitch block (if available) needs to swap buffers. */
	bool stitchSwapBuffers_;

private:
	/* Number of metadata objects available in the context list. */
	static constexpr unsigned int numMetadataContexts = 16;

	virtual int32_t platformInit(const InitParams &params, InitResult *result) = 0;
	virtual int32_t platformStart(const ControlList &controls, StartResult *result) = 0;
	virtual int32_t platformConfigure(const ConfigParams &params, ConfigResult *result) = 0;

	virtual void platformPrepareIsp(const PrepareParams &params,
					RPiController::Metadata &rpiMetadata) = 0;
	virtual RPiController::StatisticsPtr platformProcessStats(Span<uint8_t> mem) = 0;

	void setMode(const IPACameraSensorInfo &sensorInfo);
	void setCameraTimeoutValue();
	bool validateSensorControls();
	bool validateLensControls();
	void applyControls(const ControlList &controls);
	virtual void handleControls(const ControlList &controls) = 0;
	void fillDeviceStatus(const ControlList &sensorControls, unsigned int ipaContext);
	void fillSyncParams(const PrepareParams &params, unsigned int ipaContext);
	void reportMetadata(unsigned int ipaContext);
	void applyFrameDurations(utils::Duration minFrameDuration, utils::Duration maxFrameDuration);
	void applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls,
		      utils::Duration frameDurationOffset = utils::Duration(0));

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	bool lensPresent_;
	bool monoSensor_;

	std::array<RPiController::Metadata, numMetadataContexts> rpiMetadata_;

	/*
	 * We count frames to decide if the frame must be hidden (e.g. from
	 * display) or mistrusted (i.e. not given to the control algos).
	 */
	uint64_t frameCount_;

	/* How many frames we should avoid running control algos on. */
	unsigned int mistrustCount_;

	/* Number of frames that need to be marked as dropped on startup. */
	unsigned int invalidCount_;

	/* Frame timestamp for the last run of the controller. */
	uint64_t lastRunTimestamp_;

	/* Do we run a Controller::process() for this frame? */
	bool processPending_;

	/* Distinguish the first camera start from others. */
	bool firstStart_;

	/* Frame duration (1/fps) limits. */
	utils::Duration minFrameDuration_;
	utils::Duration maxFrameDuration_;

	/* The current state of flicker avoidance. */
	struct FlickerState {
		int32_t mode;
		utils::Duration manualPeriod;
	} flickerState_;

	bool cnnEnableInputTensor_;
};

} /* namespace ipa::RPi */

} /* namespace libcamera */
