#include "context_stage.h"

#include <libcamera/base/log.h>

#include "pipeline.h"

using namespace libcamera;
using namespace tiling;

LOG_DECLARE_CATEGORY(PISP_TILING);

ContextStage::ContextStage(char const *name, Stage *upstream, Config const &config, int struct_offset)
	: BasicStage(name, upstream->GetPipeline(), upstream, struct_offset), config_(config)
{
}

void ContextStage::PushStartUp(int output_start, Dir dir)
{
	LOG(PISP_TILING, Debug) << "Enter with output_start " << output_start;

	int input_start = output_start - config_.context[dir].start;
	if (input_start < 0)
		input_start = 0;
	input_start -= input_start % config_.alignment[dir];
	output_interval_.offset = output_start;
	input_interval_.offset = input_start;

	LOG(PISP_TILING, Debug) << "Exit - call PushStartUp with " << input_interval_.offset;

	upstream_->PushStartUp(input_start, dir);
}

int ContextStage::PushEndDown(int input_end, Dir dir)
{
	// If we're given an end point that doesn't align then what are we supposed to do about it? Well,
	// we have to rely on the subsequent PushEndUp to correct the upstream stage, but we must ensure
	// we send a value downstream that, when it comes back (possibly modified) in PushEndUp, won't
	// cause us to demand a input larger than was given to us here. Simple, no?
	LOG(PISP_TILING, Debug) << "Enter with input_end " << input_end;

	int output_end = input_end;
	if (input_end < GetInputImageSize()[dir]) {
		output_end -= output_end % config_.alignment[dir];
		output_end -= config_.context[dir].end;
	}
	input_interval_.SetEnd(input_end);
	output_interval_.SetEnd(output_end);

	LOG(PISP_TILING, Debug) << "Exit with output_end " << output_end;

	PushEndUp(downstream_->PushEndDown(output_end, dir), dir);
	return input_interval_.End();
}

void ContextStage::PushEndUp(int output_end, Dir dir)
{
	LOG(PISP_TILING, Debug) << "Enter with output_end " << output_end;
	ASSERT(output_end <= output_interval_.End());

	int input_end = output_end;
	input_end += config_.context[dir].end;
	int align = config_.alignment[dir];
	input_end = ((input_end + align - 1) / align) * align;
	if (input_end > GetInputImageSize()[dir])
		input_end = GetInputImageSize()[dir];
	input_interval_.SetEnd(input_end);
	output_interval_.SetEnd(output_end);

	LOG(PISP_TILING, Debug) << "Exit with input_end " << input_end;
}

void ContextStage::PushCropDown(Interval interval, Dir dir)
{
	LOG(PISP_TILING, Debug) << "Enter with interval " << interval;
	ASSERT(input_interval_ < interval);

	int align = config_.alignment[dir];
	if (interval.offset % align ||
	    (interval.End() % align && interval.End() != GetInputImageSize()[dir])) {
		// Interval doesn't align properly. Some cropping will be required. Rather than calculating
		// the necessary crop it's safe just to send out former input tile downstream. This could
		// genuinely happen if people put weird alignments throughout their pipeline, but in practice
		// Bayer stages should all be 2-pixel aligned so there's no reason this should pop out.
		LOG(PISP_TILING, Warning) << "Stage receiving misaligned input - cropping will be required";
		output_interval_ = input_interval_;
	} else
		output_interval_ = interval;

	input_interval_ = interval;
	crop_ = input_interval_ - output_interval_;

	LOG(PISP_TILING, Debug) << "Exit with interval " << output_interval_;
	downstream_->PushCropDown(output_interval_, dir);
}
