/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * pisp_tiling.cpp - Tiling library top level
 */
#include "pisp_tiling.hpp"

#include <memory>

#include "common/logging.hpp"

#include "context_stage.hpp"
#include "crop_stage.hpp"
#include "input_stage.hpp"
#include "output_stage.hpp"
#include "pipeline.hpp"
#include "rescale_stage.hpp"
#include "split_stage.hpp"

using namespace tiling;

namespace
{

constexpr unsigned int PipelineContextX = 16;
constexpr unsigned int PipelineContextY = 16;
constexpr unsigned int PipelineAlignX = 2;
constexpr unsigned int PipelineAlignY = 2;
constexpr unsigned int CompressionAlign = 8;

// Resampling parameters
constexpr unsigned int StartContext = 2;
constexpr unsigned int EndContext = 3;
constexpr unsigned int ScalePrecision = 12;
constexpr unsigned int RoundUp = (1 << ScalePrecision) - 1;

} // namespace

namespace libpisp
{

void tile_pipeline(TilingConfig const &config, Tile *tiles, int num_tiles, Length2 *grid)
{
	PISP_LOG(info, config);

	// First set up the pipeline.
	Pipeline::Config pipeline_config(config.max_tile_size, config.min_tile_size);
	Pipeline pipeline("PiSP", pipeline_config);

	InputStage::Config input_config(config.input_image_size, config.input_alignment,
									config.compressed_input ? CompressionAlign : 0);
	InputStage input_stage("input", &pipeline, input_config, offsetof(Tile, input));

	ContextStage::Config context_config(Crop2(Crop(PipelineContextX), Crop(PipelineContextY)),
										Length2(PipelineAlignX, PipelineAlignY));
	ContextStage context_stage("context", &input_stage, context_config, offsetof(Tile, context));

	SplitStage split_stage("split", &context_stage);

	std::unique_ptr<Stage> crop_stages[NumOutputBranches];
	std::unique_ptr<Stage> downscale_stages[NumOutputBranches];
	std::unique_ptr<Stage> resample_stages[NumOutputBranches];
	std::unique_ptr<Stage> output_stages[NumOutputBranches];

	for (int i = 0; i < NumOutputBranches; i++)
	{
		Length2 const &output_image_size = config.output_image_size[i];
		// A zero-dimension output disables that branch.
		if (output_image_size.dx == 0 || output_image_size.dy == 0)
			continue;
		char name[32];
		Stage *prev_stage = &split_stage;

		sprintf(name, "crop%d", i);
		crop_stages[i] = std::unique_ptr<Stage>(
			new CropStage(name, prev_stage, config.crop[i], offsetof(Tile, crop) + i * sizeof(Region)));
		prev_stage = crop_stages[i].get();

		// There's a little awkwardness if the resize blocks (downscale and resample) are not enabled. Resize *does* change the output tile
		// size, even if it's doing a 1-to-1 scaling (it loses context), so we must leave it out of the tiling
		// calculation.
		if (config.downscale_enables & (1 << i))
		{
			sprintf(name, "downscale%d", i);
			Length2 const &downscale_image_size = config.downscale_image_size[i];
			// There is only right context here - and it is the scale factor rounded up.
			Length2 context_right = Length2(((config.downscale_factor[i][Dir::X] + RoundUp) >> ScalePrecision) - 1,
											((config.downscale_factor[i][Dir::Y] + RoundUp) >> ScalePrecision) - 1);
			RescaleStage::Config downscale_config(downscale_image_size, config.downscale_factor[i], Length2(0, 0),
												  context_right, ScalePrecision,
												  RescaleStage::RescalerType::Downscaler);
			downscale_stages[i] = std::unique_ptr<Stage>(
				new RescaleStage(name, prev_stage, downscale_config, offsetof(Tile, downscale) + i * sizeof(Region)));
			prev_stage = downscale_stages[i].get();
		}
		if (config.resample_enables & (1 << i))
		{
			sprintf(name, "resample%d", i);
			RescaleStage::Config resample_config(output_image_size, config.resample_factor[i],
												 Length2(StartContext, StartContext), Length2(EndContext, EndContext),
												 ScalePrecision, RescaleStage::RescalerType::Resampler);
			resample_stages[i] = std::unique_ptr<Stage>(
				new RescaleStage(name, prev_stage, resample_config, offsetof(Tile, resample) + i * sizeof(Region)));
			prev_stage = resample_stages[i].get();
		}

		sprintf(name, "output%d", i);
		OutputStage::Config output_config(config.output_max_alignment[i], config.output_min_alignment[i],
										  config.output_h_mirror[i]);
		output_stages[i] = std::unique_ptr<Stage>(
			new OutputStage(name, prev_stage, output_config, offsetof(Tile, output) + i * sizeof(Region)));
	}

	// Now tile it.
	pipeline.Tile(tiles, num_tiles, sizeof(Tile), grid);
	PISP_LOG(info, "Made " << grid->dx << "x" << grid->dy << " tiles");
}

} // namespace libpisp
