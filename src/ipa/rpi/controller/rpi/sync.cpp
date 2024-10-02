/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * sync.cpp - sync algorithm
 */
#include "sync.h"

#include <chrono>
#include <ctype.h>
#include <fcntl.h>
#include <strings.h>
#include <unistd.h>

#include <libcamera/base/log.h>

#include <arpa/inet.h>

#include "sync_status.h"

using namespace std;
using namespace std::chrono_literals;
using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSync)

#define NAME "rpi.sync"

const char *kDefaultGroup = "239.255.255.250";
constexpr unsigned int kDefaultPort = 10000;
constexpr unsigned int kDefaultSyncPeriod = 30;
constexpr unsigned int kDefaultReadyFrame = 1000;
constexpr unsigned int kDefaultMinAdjustment = 50;
constexpr unsigned int kDefaultFitNumPts = 100;
constexpr unsigned int kDefaultFitMaxJitter = 100000;
constexpr unsigned int kDefaultFitMinPts = 10;

/* Returns IP address of the device we are on. */
static std::string local_address_IP()
{
	const char *google_dns_server = "8.8.8.8";
	int dns_port = 53;

	struct sockaddr_in serv;
	int sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (sock < 0)
		LOG(RPiSync, Error) << "Socket error";

	memset(&serv, 0, sizeof(serv));
	serv.sin_family = AF_INET;
	serv.sin_addr.s_addr = inet_addr(google_dns_server);
	serv.sin_port = htons(dns_port);

	int err = connect(sock, (const struct sockaddr *)&serv, sizeof(serv));
	if (err < 0)
		LOG(RPiSync, Error) << "Socket connect error";

	struct sockaddr_in name;
	socklen_t namelen = sizeof(name);
	err = getsockname(sock, (struct sockaddr *)&name, &namelen);

	char buffer[80];
	(void)inet_ntop(AF_INET, &name.sin_addr, buffer, 80);
	close(sock);
	return buffer;
}

Sync::Sync(Controller *controller)
	: SyncAlgorithm(controller), mode_(Mode::Off), socket_(-1), frameDuration_(0s), frameCount_(0)
{
}

Sync::~Sync()
{
	if (socket_ >= 0)
		close(socket_);
}

char const *Sync::name() const
{
	return NAME;
}

/* This reads from json file and intitiaises server and client */
int Sync::read(const libcamera::YamlObject &params)
{
	/* Socket on which to communicate. */
	group_ = params["group"].get<std::string>(kDefaultGroup);
	port_ = params["port"].get<uint16_t>(kDefaultPort);
	/* Send a sync message every this many frames. */
	syncPeriod_ = params["sync_period"].get<uint32_t>(kDefaultSyncPeriod);
	/* Application will be told we're ready after this many frames. */
	readyFrame_ = params["ready_frame"].get<uint32_t>(kDefaultReadyFrame);
	/* Don't change client frame length unless the change exceeds this amount (microseconds). */
	minAdjustment_ = params["min_adjustment"].get<uint32_t>(kDefaultMinAdjustment);

	/* Parameters controlling the clock fitting. */
	uint32_t fitNumPts = params["fit_num_pts"].get<uint32_t>(kDefaultFitNumPts);
	uint32_t fitMaxJitter = params["fit_max_jitter"].get<uint32_t>(kDefaultFitMaxJitter);
	uint32_t fitMinPts = params["fit_min_pts"].get<uint32_t>(kDefaultFitMinPts);
	systemToWallClock_.initialise(fitNumPts, fitMaxJitter, fitMinPts);

	return 0;
}

void Sync::initialiseSocket()
{
	socket_ = socket(AF_INET, SOCK_DGRAM, 0);
	if (socket_ < 0) {
		LOG(RPiSync, Error) << "Unable to create socket";
		return;
	}

	memset(&addr_, 0, sizeof(addr_));
	addr_.sin_family = AF_INET;
	addr_.sin_addr.s_addr = mode_ == Mode::Client ? htonl(INADDR_ANY) : inet_addr(group_.c_str());
	addr_.sin_port = htons(port_);

	if (mode_ == Mode::Client) {
		/* Set to non-blocking. */
		int flags = fcntl(socket_, F_GETFL, 0);
		fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

		unsigned int en = 1;
		if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &en, sizeof(en)) < 0) {
			LOG(RPiSync, Error) << "Unable to set socket options";
			goto err;
		}

		struct ip_mreq mreq {
		};
		mreq.imr_multiaddr.s_addr = inet_addr(group_.c_str());
		mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
			LOG(RPiSync, Error) << "Unable to set socket options";
			goto err;
		}

		if (bind(socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
			LOG(RPiSync, Error) << "Unable to bind client socket";
			goto err;
		}
	}

	return;

err:
	close(socket_);
	socket_ = -1;
}

void Sync::switchMode([[maybe_unused]] CameraMode const &cameraMode, [[maybe_unused]] Metadata *metadata)
{
	syncReady_ = false;
	frameCount_ = 0;
	firstFrame_ = true;
	lag_ = 0;
	serverFrameCountPeriod_ = 0;
	clientServerReadyTime_ = 0;
	clientSeenPacket_ = false;
}

/*
 * Camera sync algorithm.
 *     Server - there is a single server that sends framerate timing information over the network to any
 *         clients that are listening. It also signals when it will send a "everything is synchronised, now go"
 *         message back to the algorithm.
 *     Client - there may be many clients, either on the same Pi or different ones. They match their
 *         framerates to the server, and indicate when to "go" at the same instant as the server.
 */
void Sync::process([[maybe_unused]] StatisticsPtr &stats, Metadata *imageMetadata)
{
	SyncPayload payload;
	SyncParams local{};
	SyncStatus status{};
	bool lagKnown = true;

	imageMetadata->get("sync.params", local);

	if (!frameDuration_) {
		LOG(RPiSync, Error) << "Sync frame duration not set!";
		return;
	}

	if (mode_ == Mode::Off)
		return;

	if (socket_ < 0)
		initialiseSocket();

	/* The local wallclock for the very first frame can be a bit off, so ignore it. */
	if (firstFrame_) {
		firstFrame_ = false;

		/*
		 * For the client, flush anything in the socket. It might be stale from a previous sync run,
		 * or we might get another packet in a frame to two before the adjustment caused by this (old)
		 * packet, although correct, had taken effect. So this keeps things simpler.
		 */
		if (mode_ == Mode::Client) {
			socklen_t addrlen = sizeof(addr_);
			int ret = 0;
			while (ret >= 0)
				ret = recvfrom(socket_, &payload, sizeof(payload), 0, (struct sockaddr *)&addr_, &addrlen);
		}

		return;
	}

	/*
	 * It might be possible for a frame not to have a valid wallclock, in which case don't let it
	 * get into the clock recovery as it would totally throw it off.
	 */
	if (local.wallClock == 0) {
		LOG(RPiSync, Debug) << "Zero-valued wallclock - ignoring";
		return;
	}

	/* Derive a de-jittered version of wall clock. sensorTimestamp needs converting from ns to us. */
	uint64_t systemFrameTimestamp = local.sensorTimestamp / 1000;
	systemToWallClock_.addSample(systemFrameTimestamp, local.wallClock);
	uint64_t wallClockFrameTimestamp = systemToWallClock_.getOutput(systemFrameTimestamp);

	/*
	 * This is the headline frame duration in microseconds as programmed into the sensor. Strictly,
	 * the sensor might not quite match the system clock, but this shouldn't matter for the calculations
	 * we'll do with it, unless it's a very very long way out!
	 */
	uint32_t frameDuration = frameDuration_.get<std::micro>();

	/* Timestamps tell us if we've dropped any frames, but we still want to count them. */
	int droppedFrames = 0;
	if (frameCount_) {
		/*
		 * Round down here, because frameCount_ gets incremented at the end of the function. Also
		 * ensure droppedFrames can't go negative. It shouldn't, but things would go badly wrong
		 * if it did.
		 */
		wallClockFrameTimestamp = std::max<uint64_t>(wallClockFrameTimestamp, lastWallClockFrameTimestamp_ + frameDuration / 2);
		droppedFrames = (wallClockFrameTimestamp - lastWallClockFrameTimestamp_ - frameDuration / 2) / frameDuration;
		frameCount_ += droppedFrames;
	}

	if (mode_ == Mode::Server) {
		/*
		 * Server sends a packet every syncPeriod_ frames, or as soon after as possible (if any
		 * frames were dropped).
		 */
		serverFrameCountPeriod_ += droppedFrames;

		/*
		 * The client may want a better idea of the true frame duration. Any error would feed straight
		 * into the correction term because of how it uses it to get the "nearest" frame.
		 */
		if (frameCount_ == 0)
			frameDurationEstimated_ = frameDuration;
		else {
			double diff = (systemFrameTimestamp - lastSystemFrameTimestamp_) / (1 + droppedFrames);
			int N = std::min(frameCount_, 99U);
			frameDurationEstimated_ = frameCount_ == 1 ? diff : (N * frameDurationEstimated_ + diff) / (N + 1);
		}

		/* Calculate frames remaining, and therefore "time left until ready". */
		int framesRemaining = readyFrame_ - frameCount_;
		uint64_t systemReadyTime = systemFrameTimestamp + (int64_t)framesRemaining * frameDurationEstimated_;
		uint64_t wallClockReadyTime = systemToWallClock_.getOutput(systemReadyTime);

		if (serverFrameCountPeriod_ >= syncPeriod_) {
			serverFrameCountPeriod_ = 0;

			payload.frameDuration = frameDurationEstimated_ + .5; /* round to nearest */
			payload.systemFrameTimestamp = systemFrameTimestamp;
			payload.wallClockFrameTimestamp = wallClockFrameTimestamp;
			payload.systemReadyTime = systemReadyTime;
			payload.wallClockReadyTime = wallClockReadyTime;

			LOG(RPiSync, Debug) << "Send packet (frameNumber " << frameCount_ << "):";
			LOG(RPiSync, Debug) << "            frameDuration " << payload.frameDuration;
			LOG(RPiSync, Debug) << "            systemFrameTimestamp " << systemFrameTimestamp
					    << " (" << systemFrameTimestamp - lastSystemFrameTimestamp_ << ")";
			LOG(RPiSync, Debug) << "            wallClockFrameTimestamp " << wallClockFrameTimestamp
					    << " (" << wallClockFrameTimestamp - lastWallClockFrameTimestamp_ << ")";
			LOG(RPiSync, Debug) << "            systemReadyTime " << systemReadyTime;
			LOG(RPiSync, Debug) << "            wallClockReadyTime " << wallClockReadyTime;

			if (sendto(socket_, &payload, sizeof(payload), 0, (const sockaddr *)&addr_, sizeof(addr_)) < 0)
				LOG(RPiSync, Error) << "Send error! " << strerror(errno);
		}

		lag_ = (int64_t)wallClockFrameTimestamp - (int64_t)wallClockReadyTime;
		if (!syncReady_ && wallClockFrameTimestamp + frameDurationEstimated_ / 2 > wallClockReadyTime) {
			syncReady_ = true;
			LOG(RPiSync, Info) << "*** Sync achieved! Lag " << lag_;
		}

		serverFrameCountPeriod_ += 1;

	} else if (mode_ == Mode::Client) {
		uint64_t serverFrameTimestamp = 0;

		bool packetReceived = false;
		while (true) {
			socklen_t addrlen = sizeof(addr_);
			int ret = recvfrom(socket_, &payload, sizeof(payload), 0, (struct sockaddr *)&addr_, &addrlen);

			if (ret < 0)
				break;
			packetReceived = (ret > 0);
			clientSeenPacket_ = true;

			if (!IPCheck_) {
				IPCheck_ = true;
				char srcIP[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, &(addr_.sin_addr), srcIP, INET_ADDRSTRLEN);
				clientSamePi_ = (local_address_IP() == srcIP);
				LOG(RPiSync, Debug) << "Server is " << (clientSamePi_ ? "same" : "different");
			}

			frameDurationEstimated_ = payload.frameDuration;
			if (clientSamePi_) {
				serverFrameTimestamp = payload.systemFrameTimestamp;
				clientServerReadyTime_ = payload.systemReadyTime;
			} else {
				serverFrameTimestamp = payload.wallClockFrameTimestamp;
				clientServerReadyTime_ = payload.wallClockReadyTime;
			}
		}

		if (packetReceived) {
			uint64_t clientFrameTimestamp = clientSamePi_ ? systemFrameTimestamp : wallClockFrameTimestamp;
			int64_t clientServerDelta = clientFrameTimestamp - serverFrameTimestamp;
			/* "A few frames ago" may have better matched the server's frame. Calculate when it was. */
			int framePeriodErrors = (clientServerDelta + frameDurationEstimated_ / 2) / frameDurationEstimated_;
			int64_t clientFrameTimestampNearest = clientFrameTimestamp - framePeriodErrors * frameDurationEstimated_;
			/* We must shorten a single client frame by this amount if it exceeds the minimum: */
			int32_t correction = clientFrameTimestampNearest - serverFrameTimestamp;
			if (std::abs(correction) < minAdjustment_)
				correction = 0;

			LOG(RPiSync, Debug) << "Received packet (frameNumber " << frameCount_ << "):";
			LOG(RPiSync, Debug) << "                serverFrameTimestamp " << serverFrameTimestamp;
			LOG(RPiSync, Debug) << "                serverReadyTime " << clientServerReadyTime_;
			LOG(RPiSync, Debug) << "                clientFrameTimestamp " << clientFrameTimestamp;
			LOG(RPiSync, Debug) << "                clientFrameTimestampNearest " << clientFrameTimestampNearest
					    << " (" << framePeriodErrors << ")";
			LOG(RPiSync, Debug) << "                systemFrameTimestamp " << systemFrameTimestamp
					    << " (" << systemFrameTimestamp - lastSystemFrameTimestamp_ << ")";
			LOG(RPiSync, Debug) << "                correction " << correction;

			status.frameDurationOffset = correction * 1us;
		}

		uint64_t clientFrameTimestamp = clientSamePi_ ? systemFrameTimestamp : wallClockFrameTimestamp;
		lag_ = (int64_t)clientFrameTimestamp - (int64_t)clientServerReadyTime_;
		lagKnown = clientSeenPacket_; /* client must receive a packet before the lag is correct */
		if (clientSeenPacket_ && !syncReady_ && clientFrameTimestamp + frameDurationEstimated_ / 2 > clientServerReadyTime_) {
			syncReady_ = true;
			LOG(RPiSync, Info) << "*** Sync achieved! Lag " << lag_;
		}
	}

	lastSystemFrameTimestamp_ = systemFrameTimestamp;
	lastWallClockFrameTimestamp_ = wallClockFrameTimestamp;

	status.ready = syncReady_;
	status.lag = lag_;
	status.lagKnown = lagKnown;
	imageMetadata->set("sync.status", status);
	frameCount_++;
}

void Sync::setFrameDuration(libcamera::utils::Duration frameDuration)
{
	frameDuration_ = frameDuration;
};

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Sync(controller);
}
static RegisterAlgorithm reg(NAME, &create);
