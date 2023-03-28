/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * ipa_base.cpp - Raspberry Pi IPA base class
 */
#pragma once

#include <array>
#include <deque>
#include <vector>

#include <libcamera/base/utils.h>

#include <libcamera/controls.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/raspberrypi_ipa_interface.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "cam_helper.h"
#include "camera_mode.h"
#include "controller.h"
#include "metadata.h"

namespace libcamera {

namespace ipa::RPi {

class IpaBase : public IPARPiInterface
{
public:
	IpaBase();
	virtual ~IpaBase() = 0;

	int init(const IPASettings &settings, const InitParams &params, InitResult *result) override;

	void start(const ControlList &controls, StartResult *result) override;
	void stop() override {}

	virtual int configure(const IPACameraSensorInfo &sensorInfo, const ConfigParams &params,
			      ConfigResult *result) override = 0;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void signalPrepareIsp(const PrepareParams &params) override;
	void signalProcessStats(const ProcessParams &params) override;

	void reportMetadata(unsigned int ipaContext, ControlList *controls) override;

private:
	virtual void prepareIsp(const PrepareParams &params,
				RPiController::Metadata &rpiMetadata) = 0;
	virtual RPiController::StatisticsPtr processStats(Span<uint8_t> mem) = 0;

	void setMode(const IPACameraSensorInfo &sensorInfo);
	void setCameraTimeoutValue();
	bool validateSensorControls();
	bool validateLensControls();
	void applyControls(const ControlList &controls);
	virtual void handleControls(const ControlList &controls) = 0;
	void fillDeviceStatus(const ControlList &sensorControls, unsigned int ipaContext);
	void applyFrameDurations(utils::Duration minFrameDuration, utils::Duration maxFrameDuration);
	void applyAGC(const struct AgcStatus *agcStatus, ControlList &ctrls);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	bool lensPresent_;
	ControlList libcameraMetadata_;

	/* Number of metadata objects available in the context list. */
	static constexpr unsigned int numMetadataContexts = 16;
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

	/* Distinguish the first camera start from others. */
	bool firstStart_;

	/* Frame duration (1/fps) limits. */
	utils::Duration minFrameDuration_;
	utils::Duration maxFrameDuration_;

protected:
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
};

} /* namespace ipa::RPi */

} /* namespace libcamera */
