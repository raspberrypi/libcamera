/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * output_stage.cpp - Tiling library component for output stages
 */
#include "output_stage.hpp"

#include "common/logging.hpp"

#include "pipeline.hpp"

using namespace tiling;

// There's a rather crucial convention here that when the output image is flipped, we use
// a coordinate system to describe it starting from the RH edge of the image travelling left.
// This means that tile coordinates don't change, it's the coordinate system that did. The
// upshot is that not very much changes when things are flipped, we just have to be careful
// that alignment applies not to the given tile offsets/lengths (which are now working from
// the right to left), but only when we subtract them from the image width (effectively now
// the offset/length from the LH edge).

OutputStage::OutputStage(char const *name, Stage *upstream, Config const &config, int struct_offset)
	: BasicStage(name, upstream->GetPipeline(), upstream, struct_offset), config_(config), branch_complete_(false)
{
	pipeline_->AddOutputStage(this);
}

Interval OutputStage::GetOutputInterval() const
{
	return output_interval_;
}

void OutputStage::PushStartUp(int output_start, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_start " << output_start);

	output_interval_.offset = input_interval_.offset = output_start;

	PISP_LOG(debug, "(" << name_ << ") Exit with input_start " << input_interval_.offset);
	upstream_->PushStartUp(input_interval_.offset, dir);
}

static int align_end(int input_end, int image_size, int align, bool mirrored)
{
	int output_end = input_end, aligned_output_end = output_end;
	if (mirrored)
	{
		// It's the end in the unflipped coordinate space that must align.
		int unflipped_end = image_size - input_end;
		unflipped_end = ((unflipped_end + align - 1) / align) * align;
		aligned_output_end = image_size - unflipped_end;
	}
	else
	{
		if (input_end < image_size)
			aligned_output_end = output_end - (input_end % align);
	}
	return aligned_output_end;
}

int OutputStage::PushEndDown(int input_end, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with input_end " << input_end);

	int output_end = input_end;
	int image_size = GetInputImageSize()[dir];
	bool mirrored = (dir == Dir::X && config_.x_mirrored);
	int aligned_output_end = align_end(output_end, image_size, config_.max_alignment[dir], mirrored);
	if (aligned_output_end >= output_interval_.offset + config_.max_alignment[dir])
		output_end = aligned_output_end;
	else
	{
		aligned_output_end = align_end(output_end, image_size, config_.min_alignment[dir], mirrored);
		if (aligned_output_end > output_interval_.offset)
		{
			output_end = aligned_output_end;
			PISP_LOG(debug, "(" << name_ << ") Unable to achieve optimal alignment " << config_.max_alignment[dir]);
		}
		else if (input_interval_.offset < image_size)
		{ // test against size in case this branch already finished
			PISP_LOG(warning, "(" << name_ << ") Unable to achieve mandatory alignment " << config_.min_alignment[dir]);
			output_end = aligned_output_end;
			// Just because this output can't make progress, the other branch may - at which point this
			// branch may then succeed again. So this isn't necessarily fatal. Let the split stage decide
			// if there was no progress at all.
		}
	}
	input_interval_.SetEnd(input_end);
	output_interval_.SetEnd(output_end);

	PISP_LOG(debug, "(" << name_ << ") Exit with output_end " << output_end);
	PushEndUp(output_end, dir);
	return input_interval_.End();
}

void OutputStage::PushEndUp(int output_end, [[maybe_unused]] Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_end " << output_end);

	// We should just get given back our own output value.
	PISP_ASSERT(output_end == output_interval_.End());
	input_interval_.SetEnd(output_end);

	PISP_LOG(debug, "(" << name_ << ") Exit with input_end " << output_end);
}

void OutputStage::PushCropDown(Interval interval, [[maybe_unused]] Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with interval " << interval);

	// We can't push crop any further down, it has to be trimmed here.
	input_interval_ = interval;
	crop_ = interval - output_interval_;
	PISP_ASSERT(crop_.start >= 0 && crop_.end >= 0);

	// Note that we don't flip our output interval when horizontally mirrored; we assume our caller expects to do that.
	PISP_LOG(debug, "(" << name_ << ") Exit with interval " << output_interval_);
}

void OutputStage::Reset()
{
	BasicStage::Reset();
	branch_complete_ = false;
}

bool OutputStage::GetBranchComplete() const
{
	return branch_complete_;
}

void OutputStage::SetBranchComplete()
{
	branch_complete_ = true;
	PISP_LOG(debug, "(" << name_ << ") Setting branch complete");
}
