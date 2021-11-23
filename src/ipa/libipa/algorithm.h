/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - ISP control algorithm interface
 */
#pragma once

namespace libcamera {

namespace ipa {

template<typename Context, typename Config, typename Params, typename Stats>
class Algorithm
{
public:
	virtual ~Algorithm() {}

	virtual int configure([[maybe_unused]] Context &context,
			      [[maybe_unused]] const Config &configInfo)
	{
		return 0;
	}

	virtual void prepare([[maybe_unused]] Context &context,
			     [[maybe_unused]] Params *params)
	{
	}

	virtual void process([[maybe_unused]] Context &context,
			     [[maybe_unused]] const Stats *stats)
	{
	}
};

} /* namespace ipa */

} /* namespace libcamera */
