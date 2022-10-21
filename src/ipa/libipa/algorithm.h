/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - ISP control algorithm interface
 */
#pragma once

#include <memory>
#include <stdint.h>
#include <string>

#include <libcamera/controls.h>

namespace libcamera {

class YamlObject;

namespace ipa {

template<typename _Module>
class Algorithm
{
public:
	using Module = _Module;

	virtual ~Algorithm() {}

	virtual int init([[maybe_unused]] typename Module::Context &context,
			 [[maybe_unused]] const YamlObject &tuningData)
	{
		return 0;
	}

	virtual int configure([[maybe_unused]] typename Module::Context &context,
			      [[maybe_unused]] const typename Module::Config &configInfo)
	{
		return 0;
	}

	virtual void queueRequest([[maybe_unused]] typename Module::Context &context,
				  [[maybe_unused]] const uint32_t frame,
				  [[maybe_unused]] typename Module::FrameContext &frameContext,
				  [[maybe_unused]] const ControlList &controls)
	{
	}

	virtual void prepare([[maybe_unused]] typename Module::Context &context,
			     [[maybe_unused]] const uint32_t frame,
			     [[maybe_unused]] typename Module::FrameContext &frameContext,
			     [[maybe_unused]] typename Module::Params *params)
	{
	}

	virtual void process([[maybe_unused]] typename Module::Context &context,
			     [[maybe_unused]] const uint32_t frame,
			     [[maybe_unused]] typename Module::FrameContext &frameContext,
			     [[maybe_unused]] const typename Module::Stats *stats,
			     [[maybe_unused]] ControlList &metadata)
	{
	}
};

template<typename _Module>
class AlgorithmFactoryBase
{
public:
	AlgorithmFactoryBase(const char *name)
		: name_(name)
	{
		_Module::registerAlgorithm(this);
	}

	virtual ~AlgorithmFactoryBase() = default;

	const std::string &name() const { return name_; }

	virtual std::unique_ptr<Algorithm<_Module>> create() const = 0;

private:
	std::string name_;
};

template<typename _Algorithm>
class AlgorithmFactory : public AlgorithmFactoryBase<typename _Algorithm::Module>
{
public:
	AlgorithmFactory(const char *name)
		: AlgorithmFactoryBase<typename _Algorithm::Module>(name)
	{
	}

	~AlgorithmFactory() = default;

	std::unique_ptr<Algorithm<typename _Algorithm::Module>> create() const override
	{
		return std::make_unique<_Algorithm>();
	}
};

#define REGISTER_IPA_ALGORITHM(algorithm, name) \
static AlgorithmFactory<algorithm> global_##algorithm##Factory(name);

} /* namespace ipa */

} /* namespace libcamera */
