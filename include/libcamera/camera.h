/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * camera.h - Camera object interface
 */
#ifndef __LIBCAMERA_CAMERA_H__
#define __LIBCAMERA_CAMERA_H__

#include <memory>
#include <string>

#include <libcamera/signal.h>

namespace libcamera {

class PipelineHandler;

class Camera final
{
public:
	static std::shared_ptr<Camera> create(PipelineHandler *pipe,
					      const std::string &name);

	Camera(const Camera &) = delete;
	Camera &operator=(const Camera &) = delete;

	const std::string &name() const;

	Signal<Camera *> disconnected;

private:
	Camera(PipelineHandler *pipe, const std::string &name);
	~Camera();

	friend class PipelineHandler;
	void disconnect();

	std::shared_ptr<PipelineHandler> pipe_;
	std::string name_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CAMERA_H__ */
