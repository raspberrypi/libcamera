/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * split_stage.hpp - Tiling library component for splitter stages
 */
#pragma once

#include <vector>

#include "stages.hpp"

namespace tiling
{

class SplitStage : public Stage
{
public:
	SplitStage(char const *name, Stage *upstream);
	virtual Length2 GetInputImageSize() const;
	virtual Length2 GetOutputImageSize() const;
	virtual void SetDownstream(Stage *downstream);
	virtual void Reset();
	virtual void PushStartUp(int output_start, Dir dir);
	virtual int PushEndDown(int input_end, Dir dir);
	virtual void PushEndUp(int output_end, Dir dir);
	virtual void PushCropDown(Interval interval, Dir dir);
	virtual void CopyOut(void *dest, Dir dir);
	virtual bool GetBranchComplete() const;
	virtual bool GetBranchInactive() const;

private:
	Stage *upstream_;
	std::vector<Stage *> downstream_;
	Interval input_interval_;
	unsigned int count_;
};

} // namespace tiling