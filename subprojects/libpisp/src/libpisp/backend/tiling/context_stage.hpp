/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * context_stage.hpp - Tiling library component for context stages
 */
#pragma once

#include "stages.hpp"

namespace tiling
{

class ContextStage : public BasicStage
{
public:
	struct Config
	{
		Config(Crop2 const &_context, Length2 const &_alignment)
			: context(_context), alignment(_alignment) {}
		Crop2 context;
		Length2 alignment;
	};
	ContextStage(char const *name, Stage *upstream, Config const &config, int struct_offset);
	virtual void PushStartUp(int output_start, Dir dir);
	virtual int PushEndDown(int input_end, Dir dir);
	virtual void PushEndUp(int output_end, Dir dir);
	virtual void PushCropDown(Interval interval, Dir dir);

private:
	Config config_;
};

} // namespace tiling