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

#include <libcamera/camera.h>

namespace libcamera {

class DeviceEnumerator;

class PipelineHandler
{
public:
	virtual ~PipelineHandler() { };

	virtual bool match(DeviceEnumerator *enumerator) = 0;

	virtual unsigned int count() = 0;
	virtual Camera *camera(unsigned int id) = 0;
};

class PipelineHandlerFactory
{
public:
	virtual ~PipelineHandlerFactory() { };

	virtual PipelineHandler *create() = 0;

	static void registerType(const std::string &name, PipelineHandlerFactory *factory);
	static PipelineHandler *create(const std::string &name, DeviceEnumerator *enumerator);
	static std::vector<std::string> handlers();

private:
	static std::map<std::string, PipelineHandlerFactory *> &registry();
};

#define REGISTER_PIPELINE_HANDLER(handler) \
class handler##Factory : public PipelineHandlerFactory { \
public: \
	handler##Factory() \
	{ \
		PipelineHandlerFactory::registerType(#handler, this); \
	} \
	virtual PipelineHandler *create() { \
		return new handler(); \
	} \
}; \
static handler##Factory global_##handler##Factory;

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_HANDLER_H__ */
