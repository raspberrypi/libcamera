/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * crop_stage.hpp - Tiling library component for crop stages
 */
#pragma once

#include "stages.hpp"

namespace tiling
{

class CropStage : public BasicStage
{
public:
	struct Config
	{
		Config(Interval2 const &_crop) : crop(_crop) {}
		Interval2 crop;
	};
	CropStage(char const *name, Stage *upstream, Config const &config, int struct_offset);
	virtual Length2 GetOutputImageSize() const override;
	virtual void PushStartUp(int output_start, Dir dir) override;
	virtual int PushEndDown(int input_end, Dir dir) override;
	virtual void PushEndUp(int output_end, Dir dir) override;
	virtual void PushCropDown(Interval interval, Dir dir) override;
	bool GetBranchInactive() const override;

private:
	Config config_;
};

} // namespace tiling