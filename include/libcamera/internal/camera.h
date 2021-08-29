/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera.h - Camera private data
 */

#pragma once

#include <atomic>
#include <list>
#include <memory>
#include <set>
#include <string>

#include <libcamera/base/class.h>

#include <libcamera/camera.h>

namespace libcamera {

class CameraControlValidator;
class PipelineHandler;
class Stream;

class Camera::Private : public Extensible::Private
{
	LIBCAMERA_DECLARE_PUBLIC(Camera)

public:
	Private(PipelineHandler *pipe);
	~Private();

	PipelineHandler *pipe() { return pipe_.get(); }

	std::list<Request *> queuedRequests_;
	ControlInfoMap controlInfo_;
	ControlList properties_;

	uint32_t requestSequence_;

	const CameraControlValidator *validator() const { return validator_.get(); }

private:
	enum State {
		CameraAvailable,
		CameraAcquired,
		CameraConfigured,
		CameraStopping,
		CameraRunning,
	};

	bool isAcquired() const;
	bool isRunning() const;
	int isAccessAllowed(State state, bool allowDisconnected = false,
			    const char *from = __builtin_FUNCTION()) const;
	int isAccessAllowed(State low, State high,
			    bool allowDisconnected = false,
			    const char *from = __builtin_FUNCTION()) const;

	void disconnect();
	void setState(State state);

	std::shared_ptr<PipelineHandler> pipe_;
	std::string id_;
	std::set<Stream *> streams_;
	std::set<const Stream *> activeStreams_;

	bool disconnected_;
	std::atomic<State> state_;

	std::unique_ptr<CameraControlValidator> validator_;
};

} /* namespace libcamera */
