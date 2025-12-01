/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * crop_stage.cpp - Tiling library component for crop stages
 */
#include "crop_stage.hpp"

#include "common/logging.hpp"

#include "pipeline.hpp"

using namespace tiling;

namespace
{

inline bool interval_valid(const Interval &interval, const int min_tile_size)
{
	return interval.End() >= min_tile_size && interval.length >= min_tile_size;
}

} // namespace

CropStage::CropStage(char const *name, Stage *upstream, Config const &config, int struct_offset)
	: BasicStage(name, upstream->GetPipeline(), upstream, struct_offset), config_(config)
{
}

Length2 CropStage::GetOutputImageSize() const
{
	return Length2(config_.crop.x.length, config_.crop.y.length);
}

void CropStage::PushStartUp(int output_start, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_start " << output_start);

	int input_start = output_start + config_.crop[dir].offset;
	// input_start can never be negative here, but it is possible to have output_start
	// negative if, for example, a branch starts producing output on the second
	// tile in a row (or column) and the resampler requires left (or top) context pixels.
	if (input_start < 0)
		throw std::runtime_error("input start is negative: " + std::to_string(input_start));
	output_interval_.offset = output_start;
	input_interval_.offset = input_start;

	PISP_LOG(debug, "(" << name_ << ") Exit with input_start " << input_start);
	upstream_->PushStartUp(input_start, dir);
}

int CropStage::PushEndDown(int input_end, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with input_end " << input_end);

	int output_end = input_end - config_.crop[dir].offset;

	if (output_end > config_.crop[dir].length)
		output_end = config_.crop[dir].length;

	output_interval_.SetEnd(output_end);

	// If this is the first tile to generate output, ensure we can make at least
	// min_tile_size of output pixels. If not, terminate iteration here and don't
	// go futher downstream. Defer the output for the next tile.
	//
	// output_end may also be negative if no output will be generated for this tile.
	if (!interval_valid(output_interval_, GetPipeline()->GetConfig().min_tile_size[dir]))
	{
		PISP_LOG(debug, "(" << name_ << ") Output branch not started or output too small, terminating");
		BasicStage::Reset();
		return 0;
	}

	input_interval_.SetEnd(input_end);

	PISP_LOG(debug, "(" << name_ << ") Exit with output_end " << output_end);
	PushEndUp(downstream_->PushEndDown(output_end, dir), dir);
	return input_interval_.End();
}

void CropStage::PushEndUp(int output_end, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_end " << output_end);

	int input_end = output_end + config_.crop[dir].offset;
	input_interval_.SetEnd(input_end);
	output_interval_.SetEnd(output_end);

	// Same check as we do in PushEndDown().
	if (!interval_valid(output_interval_, GetPipeline()->GetConfig().min_tile_size[dir]))
	{
		PISP_LOG(debug, "(" << name_ << ") Output branch not started or output too small, terminating");
		BasicStage::Reset();
		return;
	}

	PISP_LOG(debug, "(" << name_ << ") Exit with input_end " << input_end);
}

void CropStage::PushCropDown(Interval interval, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with interval " << interval);

	// Branch has not started producing output.  Terminate the iteration here
	// and don't go futher downstream.
	if (!interval_valid(output_interval_, GetPipeline()->GetConfig().min_tile_size[dir]))
	{
		PISP_LOG(debug, "(" << name_ << ") Output branch not started or output too small, terminating");
		BasicStage::Reset();
		return;
	}

	PISP_ASSERT(interval > input_interval_);

	input_interval_ = interval;
	interval.offset -= config_.crop[dir].offset;
	crop_ = interval - output_interval_;

	PISP_LOG(debug, "(" << name_ << ") Exit with interval " << output_interval_);
	downstream_->PushCropDown(output_interval_, dir);
}

bool CropStage::GetBranchInactive() const
{
	return !output_interval_.length;
}
