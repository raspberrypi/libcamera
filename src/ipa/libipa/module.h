/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Ideas On Board
 *
 * module.h - IPA module
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "algorithm.h"

namespace libcamera {

namespace ipa {

template<typename _Context, typename _FrameContext, typename _Config,
	 typename _Params, typename _Stats>
class Module
{
public:
	using Context = _Context;
	using FrameContext = _FrameContext;
	using Config = _Config;
	using Params = _Params;
	using Stats = _Stats;

	virtual ~Module() {}

	static std::unique_ptr<Algorithm<Module>> createAlgorithm(const std::string &name)
	{
		for (const AlgorithmFactoryBase<Module> *factory : factories()) {
			if (factory->name() == name)
				return factory->create();
		}

		return nullptr;
	}

	static void registerAlgorithm(AlgorithmFactoryBase<Module> *factory)
	{
		factories().push_back(factory);
	}

private:
	static std::vector<AlgorithmFactoryBase<Module> *> &factories()
	{
		/*
		 * The static factories map is defined inside the function to ensure
		 * it gets initialized on first use, without any dependency on
		 * link order.
		 */
		static std::vector<AlgorithmFactoryBase<Module> *> factories;
		return factories;
	}
};

} /* namespace ipa */

} /* namespace libcamera */
