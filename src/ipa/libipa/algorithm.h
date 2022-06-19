/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - ISP control algorithm interface
 */
#pragma once

namespace libcamera {

namespace ipa {

template<typename Module>
class Algorithm
{
public:
	virtual ~Algorithm() {}

	virtual int configure([[maybe_unused]] typename Module::Context &context,
			      [[maybe_unused]] const typename Module::Config &configInfo)
	{
		return 0;
	}

	virtual void prepare([[maybe_unused]] typename Module::Context &context,
			     [[maybe_unused]] typename Module::Params *params)
	{
	}

	virtual void process([[maybe_unused]] typename Module::Context &context,
			     [[maybe_unused]] typename Module::FrameContext *frameContext,
			     [[maybe_unused]] const typename Module::Stats *stats)
	{
	}
};

} /* namespace ipa */

} /* namespace libcamera */
