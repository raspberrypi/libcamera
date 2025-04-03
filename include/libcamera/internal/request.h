/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Request class private data
 */

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <stdint.h>
#include <unordered_set>

#include <libcamera/base/event_notifier.h>
#include <libcamera/base/timer.h>

#include <libcamera/request.h>

using namespace std::chrono_literals;

namespace libcamera {

class Camera;
class FrameBuffer;

class Request::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(Request)

public:
	Private(Camera *camera);
	~Private();

	Camera *camera() const { return camera_; }
	bool hasPendingBuffers() const;

	bool completeBuffer(FrameBuffer *buffer);
	void complete();
	void cancel();
	void reset();

	void prepare(std::chrono::milliseconds timeout = 0ms);
	Signal<> prepared;

private:
	friend class PipelineHandler;
	friend std::ostream &operator<<(std::ostream &out, const Request &r);

	void doCancelRequest();
	void emitPrepareCompleted();
	void notifierActivated(FrameBuffer *buffer);
	void timeout();

	Camera *camera_;
	bool cancelled_;
	uint32_t sequence_ = 0;
	bool prepared_ = false;

	std::unordered_set<FrameBuffer *> pending_;
	std::map<FrameBuffer *, std::unique_ptr<EventNotifier>> notifiers_;
	std::unique_ptr<Timer> timer_;
};

} /* namespace libcamera */
