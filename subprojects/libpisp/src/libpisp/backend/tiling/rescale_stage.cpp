/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * rescale_stage.cpp - Tiling library component for rescaling stages
 */
#include "rescale_stage.hpp"

#include "common/logging.hpp"

#include "pipeline.hpp"

using namespace tiling;

RescaleStage::RescaleStage(char const *name, Stage *upstream, Config const &config, int struct_offset)
	: BasicStage(name, upstream->GetPipeline(), upstream, struct_offset), config_(config)
{
	round_up = (1 << config.precision) - 1;
}

Length2 RescaleStage::GetOutputImageSize() const
{
	return config_.output_image_size;
}

// In what follows, the suffix _P indicates that a variable is a fixed point value,left-shifted by PRECISION.
// We also use _w_context and _no_context to denote input coordinates where, respectively, the context pixels
// required by the resample filter either are or are not included.

void RescaleStage::PushStartUp(int output_start, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_start " << output_start);

	int input_start_P = output_start * config_.scale[dir];
	int input_start = input_start_P >> config_.precision;
	int input_start_w_context = input_start - config_.start_context[dir];
	// input_start_w_context is allowed to go negative here if, for example, the branch starts producing output
	// on the second tile in a row/column and the resampler requires left context pixels. In such cases, the left/top
	// edge flag will not be set on the tile and the hardware cannot remove the left/top context pixels. So allow
	// negative values on all but the left/top edge tile.
	if (GetPipeline()->FirstTile() && input_start_w_context < 0)
		input_start_w_context = 0;
	output_interval_.offset = output_start;
	input_interval_.offset = input_start_w_context;

	PISP_LOG(debug, "(" << name_ << ") Exit with input_start " << input_start_w_context);
	upstream_->PushStartUp(input_start_w_context, dir);
}

// Furthermore, when we're talking about end pixels we need to be careful whether we're talking inclusively
// or exclusively. For example, the End() pixel of an interval is an exclusive pixel number, so End()-1 is
// actually the last pixel still in the interval. We'll denote these with the suffices _exc or _inc.

int RescaleStage::PushEndDown(int input_end, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with input_end " << input_end);

	int input_image_size = GetInputImageSize()[dir];
	input_interval_.SetEnd(input_end);
	int output_end_exc;

	if (config_.rescaler_type == RescalerType::Downscaler)
	{
		// Trapezoidal downscaler has a variable-sized kernel. Round down its end
		// position to get the number of complete output samples that can be generated
		// (provided scale was rounded down, there should be no shortage of input).
		output_end_exc = (input_end << config_.precision) / config_.scale[dir];
	}
	else
	{
		// Resampler: find the last ("inclusive") sample that can be generated.
		// Take off context plus an extra 2 pixels off to allow for an initial phase
		// (except that at the bottom of the image, no more context is available).
		int input_end_inc = input_end - 1;
		int input_end_inc_no_context = input_end_inc;
		if (input_end < input_image_size)
		{
			input_end_inc_no_context = input_end_inc - config_.end_context[dir] - 2;
		}
		int input_end_inc_no_context_P = input_end_inc_no_context << config_.precision;
		int output_end_inc = (input_end_inc_no_context_P + round_up) / config_.scale[dir];
		output_end_exc = output_end_inc + 1;
	}

	if (output_end_exc > config_.output_image_size[dir])
		output_end_exc = config_.output_image_size[dir];

	// Upscaling could generate larger output tiles than we can handle, so avoid doing that!
	if (output_end_exc > output_interval_.offset + GetPipeline()->GetConfig().max_tile_size[dir])
		output_end_exc = GetPipeline()->GetConfig().max_tile_size[dir] + output_interval_.offset;

	output_interval_.SetEnd(output_end_exc);

	PISP_LOG(debug, "(" << name_ << ") Exit with output_end " << output_end_exc);
	PushEndUp(downstream_->PushEndDown(output_end_exc, dir), dir);

	// If we didn't quite finish the output then we can't get too close to the end of the input because another tile will be
	// needed, and we can't let that become infeasibly small. So pull our input_end back and simply try again.
	if (output_interval_.End() < config_.output_image_size[dir] &&
		input_interval_.End() > input_image_size - GetPipeline()->GetConfig().min_tile_size[dir])
	{
		PISP_LOG(debug, "(" << name_ << ") Too close to input image edge - try again");
		PushEndDown(input_image_size - GetPipeline()->GetConfig().min_tile_size[dir], dir);
	}

	return input_interval_.End();
}

void RescaleStage::PushEndUp(int output_end, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with output_end " << output_end);

	int input_end_w_context_exc;
	if (config_.rescaler_type == RescalerType::Downscaler)
	{
		// Trapezoidal downscaler has a variable-sized kernel. Round up its fractional end position.
		int input_end_exc_P = output_end * config_.scale[dir];
		input_end_w_context_exc = (input_end_exc_P + round_up) >> config_.precision;
	}
	else
	{
		// Resampler has a fixed-sized context, so calculations are based on the start position
		// for its final output sample ("inclusive" dimensions). Need 2 more pixels to the end
		// to allow for an initial phase that pushes us to use up to 2 extra samples on the right.
		int output_end_inc = output_end - 1;
		int input_end_P = output_end_inc * config_.scale[dir];
		int input_end = input_end_P >> config_.precision;
		int input_end_w_context = input_end + config_.end_context[dir] + 2;
		input_end_w_context_exc = input_end_w_context + 1;
	}

	Length2 input_image_size(GetInputImageSize());
	if (input_end_w_context_exc > input_image_size[dir])
		input_end_w_context_exc = input_image_size[dir];

	output_interval_.SetEnd(output_end);
	input_interval_.SetEnd(input_end_w_context_exc);

	PISP_LOG(debug, "(" << name_ << ") Exit with input_end " << input_end_w_context_exc);
}

void RescaleStage::PushCropDown(Interval interval, Dir dir)
{
	PISP_LOG(debug, "(" << name_ << ") Enter with interval " << interval);

	PISP_ASSERT(interval > input_interval_);
	crop_ = interval - input_interval_;
	input_interval_ = interval;

	PISP_LOG(debug, "(" << name_ << ") Exit with interval " << output_interval_);
	downstream_->PushCropDown(output_interval_, dir);
}
