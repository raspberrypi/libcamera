#include "crop_stage.hpp"

#include <cassert>

#include "common/pisp_logging.hpp"

#include "pipeline.hpp"

using namespace tiling;

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
	PISP_TILING_LOG(debug, "enter with output_start " << output_start);
	int input_start = output_start + config_.crop[dir].offset;
	output_interval_.offset = output_start;
	input_interval_.offset = input_start;
	PISP_TILING_LOG(debug, "exit with input_start " << input_start);
	upstream_->PushStartUp(input_start, dir);
}

int CropStage::PushEndDown(int input_end, Dir dir)
{
	PISP_TILING_LOG(debug, "enter with input_end " << input_end);
	int output_end = input_end - config_.crop[dir].offset;
	if (output_end > config_.crop[dir].length)
		output_end = config_.crop[dir].length;
	input_interval_.SetEnd(input_end);
	output_interval_.SetEnd(output_end);
	PISP_TILING_LOG(debug, "exit with output_end " << output_end);
	PushEndUp(downstream_->PushEndDown(output_end, dir), dir);
	return input_interval_.End();
}

void CropStage::PushEndUp(int output_end, Dir dir)
{
	PISP_TILING_LOG(debug, "enter with output_end " << output_end);
	int input_end = output_end + config_.crop[dir].offset;
	input_interval_.SetEnd(input_end);
	output_interval_.SetEnd(output_end);
	PISP_TILING_LOG(debug, "exit with input_end " << input_end);
}

void CropStage::PushCropDown(Interval interval, Dir dir)
{
	PISP_TILING_LOG(debug, "enter with interval " << interval);
	assert(interval > input_interval_);
	input_interval_ = interval;
	interval.offset -= config_.crop[dir].offset;
	crop_ = interval - output_interval_;
	PISP_TILING_LOG(debug, "exit with interval " << output_interval_);
	downstream_->PushCropDown(output_interval_, dir);
}
