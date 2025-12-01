/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * stages.cpp - Tiling library Stage class implementation
 */
#include "stages.hpp"

#include "common/logging.hpp"
#include "pipeline.hpp"

#include <cstdint>

using namespace tiling;

Stage::Stage(char const *name, Pipeline *pipeline, int struct_offset)
	: name_(name), pipeline_(pipeline), struct_offset_(struct_offset)
{
	if (pipeline)
		pipeline->AddStage(this);
}

Pipeline *Stage::GetPipeline() const
{
	return pipeline_;
}

void Stage::MergeRegions(void *dest, void *x_src, void *y_src) const
{
	if (struct_offset_ >= 0) // Hacky test to exclude stages that don't store a Region in each Tile
	{
		Region *dest_region = (Region *)((int8_t *)dest + struct_offset_);
		Region *x_src_region = (Region *)((int8_t *)x_src + struct_offset_);
		Region *y_src_region = (Region *)((int8_t *)y_src + struct_offset_);
		dest_region->input[Dir::X] = x_src_region->input[Dir::X];
		dest_region->crop[Dir::X] = x_src_region->crop[Dir::X];
		dest_region->output[Dir::X] = x_src_region->output[Dir::X];
		dest_region->input[Dir::Y] = y_src_region->input[Dir::Y];
		dest_region->crop[Dir::Y] = y_src_region->crop[Dir::Y];
		dest_region->output[Dir::Y] = y_src_region->output[Dir::Y];
	}
}

BasicStage::BasicStage(char const *name, Pipeline *pipeline, Stage *upstream, int struct_offset)
	: Stage(name, pipeline, struct_offset), upstream_(upstream), downstream_(nullptr)
{
	if (upstream)
		upstream->SetDownstream(this);
}

Length2 BasicStage::GetInputImageSize() const
{
	return upstream_->GetOutputImageSize();
}

Length2 BasicStage::GetOutputImageSize() const
{
	return GetInputImageSize();
}

void BasicStage::SetDownstream(Stage *downstream)
{
	downstream_ = downstream;
}

void BasicStage::Reset()
{
	input_interval_ = Interval(0, 0);
	crop_ = Crop(0, 0);
	output_interval_ = Interval(0, 0);
}

void BasicStage::CopyOut(void *dest, Dir dir)
{
	if (struct_offset_ >= 0)
	{
		Region *region = (Region *)((uint8_t *)dest + struct_offset_);

		PISP_LOG(debug, "(" << name_ << ") complete: " << GetBranchComplete() << " inactive: " << GetBranchInactive());
		if (GetBranchComplete() || GetBranchInactive())
			BasicStage::Reset();

		region->input[dir] = input_interval_;
		region->crop[dir] = crop_;
		region->output[dir] = output_interval_;
	}
}

bool BasicStage::GetBranchComplete() const
{
	return downstream_->GetBranchComplete();
}

bool BasicStage::GetBranchInactive() const
{
	if (!upstream_)
		return false;
	return upstream_->GetBranchInactive();
}
