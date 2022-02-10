#include "split_stage.h"

#include <libcamera/base/log.h>

#include "pipeline.h"

using namespace libcamera;
using namespace tiling;

LOG_DECLARE_CATEGORY(PISP_TILING);

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
	LOG(PISP_TILING, Debug) << "Enter with output_start " << output_start;

	// We must wait till all the downstream branches have given us their number, then we send the leftmost
	// one up the pipeline.
	if (count_++ == 0)
		input_interval_ = Interval(output_start);
	else
		input_interval_ |= output_start;
	if (count_ == downstream_.size()) {
		count_ = 0;
		LOG(PISP_TILING, Debug) << "Exit - call PushStartUp with " << input_interval_.offset;
		upstream_->PushStartUp(input_interval_.offset, dir);
	}
}

int SplitStage::PushEndDown(int input_end, Dir dir)
{
	LOG(PISP_TILING, Debug) << "Enter with input_end " << input_end;

	// First tell all the branches what the maximum number of pixels is that they can have so that we find
	// out what they can do with it. Then remember the least far-on end position that they need. This avoid
	// potential over-read if one branch can only accept way fewer pixels than another.
	input_interval_.SetEnd(input_end);
	for (auto d : downstream_) {
		LOG(PISP_TILING, Debug) << "Exit with output_end " << input_end;
		int branch_end = d->PushEndDown(input_end, dir);
		if (branch_end < input_interval_.End())
			input_interval_.SetEnd(branch_end);
	}

	// Finally tell all the branches now what they will really get, which is that minimum end point.
	for (auto d : downstream_)
		d->PushEndDown(input_interval_.End(), dir);

	PushEndUp(input_interval_.End(), dir);
	return input_interval_.End();
}

void SplitStage::PushEndUp(int output_end, [[maybe_unused]] Dir dir)
{
	LOG(PISP_TILING, Debug) << "Enter with output_end " << output_end;
	// Genuinely nothing to do here, but we just like to log the usual trace information for consistency.
	LOG(PISP_TILING, Debug) << "Exit with input_end " << output_end;
}

void SplitStage::PushCropDown(Interval interval, Dir dir)
{
	LOG(PISP_TILING, Debug) << "Enter with interval " << interval;

	// Whatever we get goes down all the branches. If there are any that don't like it then they'll have to
	// start by cropping off what they can't handle.
	ASSERT(interval > input_interval_);
	input_interval_ = interval;
	for (auto d : downstream_) {
		LOG(PISP_TILING, Debug) << "Exit with interval " << interval;
		d->PushCropDown(interval, dir);
	}
}

void SplitStage::CopyOut([[maybe_unused]] void *dest, [[maybe_unused]] Dir dir)
{
}
