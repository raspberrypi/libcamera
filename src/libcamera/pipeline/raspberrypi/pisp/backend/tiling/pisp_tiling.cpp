#include "pisp_tiling.hpp"

#include <memory>
#include <stdexcept>

#include "common/pisp_logging.hpp"

#include "tiling.hpp"

using namespace PiSP;
using namespace tiling;

#define INPUT_ALIGN_X 16
#define INPUT_ALIGN_Y 1
#define PIPELINE_CONTEXT_X 16
#define PIPELINE_CONTEXT_Y 16
#define PIPELINE_ALIGN_X 2
#define PIPELINE_ALIGN_Y 2
#define COMPRESSION_ALIGN 8

// Resampling parameters
#define START_CONTEXT 2
#define END_CONTEXT 3
#define SCALE_PRECISION 12
#define ROUND_UP ((1 << SCALE_PRECISION) - 1)

void PiSP::tile_pipeline(TilingConfig const &config, Tile *tiles, int num_tiles, Length2 *grid)
{
	PISP_LOG(info, config);
	// First set up the pipeline.
	Pipeline::Config pipeline_config(config.max_tile_size, config.min_tile_size);
	Pipeline pipeline("PiSP", pipeline_config);

	InputStage::Config input_config(
		config.input_image_size,
		config.input_alignment,
		config.compressed_input ? COMPRESSION_ALIGN : 0);
	InputStage input_stage("input", &pipeline, input_config, offsetof(Tile, input));

	ContextStage::Config context_config(
		Crop2(Crop(PIPELINE_CONTEXT_X), Crop(PIPELINE_CONTEXT_Y)),
		Length2(PIPELINE_ALIGN_X, PIPELINE_ALIGN_Y));
	ContextStage context_stage("context", &input_stage, context_config, offsetof(Tile, context));

	CropStage::Config crop_config(config.crop);
	CropStage crop_stage("crop", &context_stage, crop_config, offsetof(Tile, crop));

	SplitStage split_stage("split", &crop_stage);

	std::unique_ptr<Stage> downscale_stages[NUM_OUTPUT_BRANCHES];
	std::unique_ptr<Stage> resample_stages[NUM_OUTPUT_BRANCHES];
	std::unique_ptr<Stage> output_stages[NUM_OUTPUT_BRANCHES];
	std::unique_ptr<Stage> hog_stage;
	for (int i = 0; i < NUM_OUTPUT_BRANCHES; i++) {
		Length2 const &output_image_size = config.output_image_size[i];
		// A zero-dimension output disables that branch.
		if (output_image_size.dx == 0 || output_image_size.dy == 0)
			continue;
		char name[32];
		Stage *prev_stage = &split_stage;

		// There's a little awkwardness if the resize blocks (downscale and resample) are not enabled. Resize *does* change the output tile
		// size, even if it's doing a 1-to-1 scaling (it loses context), so we must leave it out of the tiling
		// calculation.
		if (config.downscale_enables & (1 << i)) {
			sprintf(name, "downscale%d", i);
			Length2 const &downscale_image_size = config.downscale_image_size[i];
			// There is only right context here - and it is the scale factor rounded up.
			Length2 context_right = Length2(((config.downscale_factor[i][Dir::X] + ROUND_UP) >> SCALE_PRECISION) - 1,
							((config.downscale_factor[i][Dir::Y] + ROUND_UP) >> SCALE_PRECISION) - 1);
			RescaleStage::Config downscale_config(downscale_image_size, config.downscale_factor[i], Length2(0, 0), context_right, SCALE_PRECISION, RescaleStage::RescalerType::Downscaler);
			downscale_stages[i] = std::unique_ptr<Stage>(new RescaleStage(
				name, prev_stage, downscale_config, offsetof(Tile, downscale) + i * sizeof(Region)));
			prev_stage = downscale_stages[i].get();
		}
		if (config.resample_enables & (1 << i)) {
			sprintf(name, "resample%d", i);
			RescaleStage::Config resample_config(output_image_size, config.resample_factor[i], Length2(START_CONTEXT, START_CONTEXT), Length2(END_CONTEXT, END_CONTEXT), SCALE_PRECISION, RescaleStage::RescalerType::Resampler);
			resample_stages[i] = std::unique_ptr<Stage>(new RescaleStage(
				name, prev_stage, resample_config, offsetof(Tile, resample) + i * sizeof(Region)));
			prev_stage = resample_stages[i].get();
		}

		sprintf(name, "output%d", i);
		OutputStage::Config output_config(config.output_max_alignment[i], config.output_min_alignment[i], config.output_h_mirror[i]);
		output_stages[i] = std::unique_ptr<Stage>(new OutputStage(
			name, prev_stage, output_config, offsetof(Tile, output) + i * sizeof(Region)));
	}

	// Now tile it.
	pipeline.Tile(tiles, num_tiles, sizeof(Tile), grid);
	PISP_LOG(info, "Made " << grid->dx << "x" << grid->dy << " tiles");
}
