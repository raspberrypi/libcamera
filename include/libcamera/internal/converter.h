/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright 2022 NXP
 *
 * converter.h - Generic format converter interface
 */

#pragma once

#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class PixelFormat;
struct StreamConfiguration;

class Converter
{
public:
	Converter(MediaDevice *media);
	virtual ~Converter();

	virtual int loadConfiguration(const std::string &filename) = 0;

	virtual bool isValid() const = 0;

	virtual std::vector<PixelFormat> formats(PixelFormat input) = 0;
	virtual SizeRange sizes(const Size &input) = 0;

	virtual std::tuple<unsigned int, unsigned int>
	strideAndFrameSize(const PixelFormat &pixelFormat, const Size &size) = 0;

	virtual int configure(const StreamConfiguration &inputCfg,
			      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs) = 0;
	virtual int exportBuffers(unsigned int output, unsigned int count,
				  std::vector<std::unique_ptr<FrameBuffer>> *buffers) = 0;

	virtual int start() = 0;
	virtual void stop() = 0;

	virtual int queueBuffers(FrameBuffer *input,
				 const std::map<unsigned int, FrameBuffer *> &outputs) = 0;

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

	const std::string &deviceNode() const { return deviceNode_; }

private:
	std::string deviceNode_;
};

class ConverterFactoryBase
{
public:
	ConverterFactoryBase(const std::string name, std::initializer_list<std::string> compatibles);
	virtual ~ConverterFactoryBase() = default;

	const std::vector<std::string> &compatibles() const { return compatibles_; }

	static std::unique_ptr<Converter> create(MediaDevice *media);
	static std::vector<ConverterFactoryBase *> &factories();
	static std::vector<std::string> names();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(ConverterFactoryBase)

	static void registerType(ConverterFactoryBase *factory);

	virtual std::unique_ptr<Converter> createInstance(MediaDevice *media) const = 0;

	std::string name_;
	std::vector<std::string> compatibles_;
};

template<typename _Converter>
class ConverterFactory : public ConverterFactoryBase
{
public:
	ConverterFactory(const char *name, std::initializer_list<std::string> compatibles)
		: ConverterFactoryBase(name, compatibles)
	{
	}

	std::unique_ptr<Converter> createInstance(MediaDevice *media) const override
	{
		return std::make_unique<_Converter>(media);
	}
};

#define REGISTER_CONVERTER(name, converter, compatibles) \
	static ConverterFactory<converter> global_##converter##Factory(name, compatibles);

} /* namespace libcamera */
