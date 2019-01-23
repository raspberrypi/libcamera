/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * pipeline_handler.h - Pipeline handler infrastructure
 */
#ifndef __LIBCAMERA_PIPELINE_HANDLER_H__
#define __LIBCAMERA_PIPELINE_HANDLER_H__

#include <map>
#include <string>
#include <vector>

namespace libcamera {

class CameraManager;
class DeviceEnumerator;

class PipelineHandler
{
public:
	PipelineHandler(CameraManager *manager);
	virtual ~PipelineHandler();

	virtual bool match(DeviceEnumerator *enumerator) = 0;

protected:
	CameraManager *manager_;
};

class PipelineHandlerFactory
{
public:
	PipelineHandlerFactory(const char *name);
	virtual ~PipelineHandlerFactory() { };

	virtual PipelineHandler *create(CameraManager *manager) = 0;

	const std::string &name() const { return name_; }

	static void registerType(PipelineHandlerFactory *factory);
	static std::vector<PipelineHandlerFactory *> &factories();

private:
	std::string name_;
};

#define REGISTER_PIPELINE_HANDLER(handler)				\
class handler##Factory final : public PipelineHandlerFactory		\
{									\
public:									\
	handler##Factory() : PipelineHandlerFactory(#handler) {}	\
	PipelineHandler *create(CameraManager *manager) 		\
	{								\
		return new handler(manager);				\
	}								\
};									\
static handler##Factory global_##handler##Factory;

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_HANDLER_H__ */
