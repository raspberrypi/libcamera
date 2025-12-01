/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * pipeline.cpp - Tiling library pipeline generator
 */
#include "pipeline.hpp"

#include "common/logging.hpp"

#include "input_stage.hpp"
#include "output_stage.hpp"
#include "stages.hpp"

#include <cstdint>

using namespace tiling;

Pipeline::Pipeline(char const *name, Config const &config) : name_(name), config_(config), first_tile_(false)
{
}

Pipeline::Config const &Pipeline::GetConfig() const
{
	return config_;
}

void Pipeline::AddStage(Stage *stage)
{
	stages_.push_back(stage);
}

void Pipeline::AddInputStage(InputStage *input_stage)
{
	inputs_.push_back(input_stage);
}

void Pipeline::AddOutputStage(OutputStage *output_stage)
{
	outputs_.push_back(output_stage);
}

void Pipeline::Tile(void *mem, size_t num_items, size_t item_size, Length2 *grid)
{
	// Tiling produces a rectangular X by Y grid. We create X direction tiles along the first row,
	// then Y direction tiles down the column. Finally we copy the X/Y information to all the other
	// tiles in the grid.
	grid->dx = tileDirection(Dir::X, mem, num_items, item_size);
	grid->dy = tileDirection(Dir::Y, mem, num_items / grid->dx, item_size * grid->dx);
	int i, j;
	for (j = 0; j < grid->dy; j++)
	{
		void *y_src = (uint8_t *)mem + item_size * grid->dx * j;
		for (i = 0; i < grid->dx; i++)
		{
			void *x_src = (uint8_t *)mem + item_size * i;
			void *dest = (uint8_t *)y_src + item_size * i;
			for (auto s : stages_)
				s->MergeRegions(dest, x_src, y_src);
		}
	}
}

void Pipeline::reset()
{
	for (auto s : stages_)
		s->Reset();
}

int Pipeline::tileDirection(Dir dir, void *mem, size_t num_items, size_t item_size)
{
	PISP_LOG(debug, "Tiling direction " << dir);

	reset();
	bool done = false;
	unsigned int num_tiles = 0;
	first_tile_ = true;
	for (; !done; num_tiles++)
	{
		PISP_LOG(debug, "----------------------------------------------------------------");
		if (num_tiles == num_items)
			throw std::runtime_error("Too many tiles!");
		for (auto s : outputs_)
		{
			if (!s->GetBranchComplete())
				s->PushStartUp(s->GetOutputInterval().End(), dir);
		}
		for (auto s : inputs_)
			s->PushEndDown(s->GetInputInterval().offset + config_.max_tile_size[dir], dir);
		for (auto s : inputs_)
			s->PushCropDown(s->GetInputInterval(), dir);
		void *dest = (uint8_t *)mem + num_tiles * item_size;
		for (auto s : stages_)
			s->CopyOut(dest, dir);
		done = true;
		for (auto s : outputs_)
		{
			if (s->GetBranchComplete())
				continue;
			else if (s->GetOutputInterval().End() >= s->GetOutputImageSize()[dir])
				s->SetBranchComplete();
			else
				done = false;
		}
		first_tile_ = false;
	}

	PISP_LOG(debug, "Made " << num_tiles << " tiles in direction " << dir);
	return num_tiles;
}
