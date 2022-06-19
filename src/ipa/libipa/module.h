/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Ideas On Board
 *
 * module.h - IPA module
 */

#pragma once

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
};

} /* namespace ipa */

} /* namespace libcamera */
