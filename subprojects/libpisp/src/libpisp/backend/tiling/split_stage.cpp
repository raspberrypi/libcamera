/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * split_stage.cpp - Tiling library component for splitter stages
 */
#include "split_stage.hpp"

#include "common/logging.hpp"

#include "pipeline.hpp"

using namespace tiling;

SplitStage::SplitStage(char const *name, Stage *upstream)
	: Stage(name, upstream->GetPipeline(), -1), upstream_(upstream)
{
	upstream->SetDownstream(this);
}

Length2 SplitStage::GetInputImageSize() const
{
	return upstream_->GetOutputImageSize();
}

Length2 SplitStage::GetOutputImageSize() const
{
	return GetInputImageSize();
}

void SplitStage::SetDownstream(Stage *stage)
{
	downstream_.push_back(stage);
}

void SplitStage::Reset()
{
	input_interval_ = Interval(0, 0);
	count_ = 0;
}

void SplitStage::PushStartUp(int output_start, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_start " << output_start);

	// We must wait till all the downstream branches have given us their number, then we send the leftmost
	// one up the pipeline.
	if (count_++ == 0)
		input_interval_ = Interval(output_start);
	else
		input_interval_ |= output_start;

	unsigned int branch_incomplete_count = 0;
	for (auto const &d : downstream_)
		if (!d->GetBranchComplete())
			branch_incomplete_count++;

	if (count_ == branch_incomplete_count)
	{
		count_ = 0;
		PISP_LOG(debug, "(" << name_ << ") Exit - call PushStartUp with " << input_interval_.offset);
		upstream_->PushStartUp(input_interval_.offset, dir);
	}
}

int SplitStage::PushEndDown(int input_end, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with input_end " << input_end);

	// First tell all the branches what the maximum number of pixels is that they can have so that we find
	// out what they can do with it. Then remember the least far-on end position that they need. This avoid
	// potential over-read if one branch can only accept way fewer pixels than another.
	input_interval_.SetEnd(0);
	for (auto d : downstream_)
	{
		if (d->GetBranchComplete())
			continue;
		int branch_end = d->PushEndDown(input_end, dir);
		// (It is OK for a branch to make no progress at all - so long as another branch does!)
		if (branch_end > input_interval_.End())
			input_interval_.SetEnd(branch_end);
	}

	// Finally tell all the branches now what they will really get, which is that end point.
	PISP_LOG(debug, "(" << name_ << ") Split using input_end " << input_interval_.End());
	if (input_interval_.length == 0)
	{
		PISP_LOG(fatal, "(" << name_ << ") Neither branch can make progress");
		throw TilingException();
	}

	for (auto d : downstream_)
		if (!d->GetBranchComplete())
			d->PushEndDown(input_interval_.End(), dir);

	PushEndUp(input_interval_.End(), dir);
	return input_interval_.End();
}

void SplitStage::PushEndUp([[maybe_unused]] int output_end, [[maybe_unused]] Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_end " << output_end);
	// Genuinely nothing to do here, but we just like to log the usual trace information for consistency.
	PISP_LOG(debug, "(" << name_ << ") Exit with input_end " << output_end);
}

void SplitStage::PushCropDown(Interval interval, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with interval " << interval);

	// Whatever we get goes down all the branches. If there are any that don't like it then they'll have to
	// start by cropping off what they can't handle.
	PISP_ASSERT(interval > input_interval_);
	input_interval_ = interval;
	for (auto d : downstream_)
	{
		if (d->GetBranchComplete())
			continue;
		PISP_LOG(debug, "(" << name_ << ") Exit with interval " << interval);
		d->PushCropDown(interval, dir);
	}
}

void SplitStage::CopyOut([[maybe_unused]] void *dest, [[maybe_unused]] Dir dir)
{
}

bool SplitStage::GetBranchComplete() const
{
	bool done = true;
	for (auto d : downstream_)
		done &= d->GetBranchComplete();
	return done;
}

bool SplitStage::GetBranchInactive() const
{
	if (!upstream_)
		return false;
	return upstream_->GetBranchInactive();
}
