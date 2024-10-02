/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * sync.h - sync algorithm
 */
#pragma once

#include <netinet/ip.h>

#include "../sync_algorithm.h"

namespace RPiController {

struct SyncPayload {
	/* Frame duration in microseconds. */
	uint32_t frameDuration;
	/* Server system (kernel) frame timestamp. */
	uint64_t systemFrameTimestamp;
	/* Server wall clock version of the frame timestamp. */
	uint64_t wallClockFrameTimestamp;
	/* Server system (kernel) sync time (the time at which frames are marked ready). */
	uint64_t systemReadyTime;
	/* Server wall clock version of the sync time. */
	uint64_t wallClockReadyTime;
};

class Sync : public SyncAlgorithm
{
public:
	Sync(Controller *controller);
	~Sync();
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void setMode(Mode mode) override;
	void initialiseSocket();
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	void setFrameDuration(libcamera::utils::Duration frameDuration) override;
	void setReadyFrame(unsigned int frame) override;

private:
	void reset(); /* reset internal state and start over */

	Mode mode_; /* server or client */
	std::string group_; /* IP group address for sync messages */
	uint16_t port_; /* port number for messages */
	uint32_t syncPeriod_; /* send a sync message every this many frames */
	uint32_t readyFrame_; /* tell the application we're ready after this many frames */
	uint32_t minAdjustment_; /* don't adjust the client frame length by less than this */

	struct sockaddr_in addr_;
	int socket_ = -1;
	libcamera::utils::Duration frameDuration_;
	unsigned int frameCount_;
	bool syncReady_;
	int64_t timerValue_ = 0; /* time until "ready time" */

	double frameDurationEstimated_ = 0; /* estimate the true frame duration of the sensor */
	uint64_t lastWallClockFrameTimestamp_; /* wall clock timestamp of previous frame */

	uint32_t serverFrameCountPeriod_ = 0; /* send the next packet when this reaches syncPeriod_ */

	bool clientSeenPacket_ = false; /* whether the client has received a packet yet */
	uint64_t serverReadyTime_ = 0; /* the client's latest value for when the server will be "ready" */
};

} /* namespace RPiController */
