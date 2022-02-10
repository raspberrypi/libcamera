#pragma once

#include "stages.h"

namespace tiling {

class InputStage : public BasicStage
{
public:
	struct Config {
		Config(Length2 const &_input_image_size, Length2 const &_alignment, int _compression_alignment = 0)
			: input_image_size(_input_image_size), alignment(_alignment), compression_alignment(_compression_alignment) {}
		Length2 input_image_size;
		Length2 alignment;
		int compression_alignment;
	};
	InputStage(char const *name, Pipeline *pipeline, Config const &config, int struct_offset);
	virtual Length2 GetInputImageSize() const;
	virtual Interval GetInputInterval() const;
	virtual void PushStartUp(int output_start, Dir dir);
	virtual int PushEndDown(int input_end, Dir dir);
	virtual void PushEndUp(int output_end, Dir dir);
	virtual void PushCropDown(Interval interval, Dir dir);

private:
	Config config_;
};

} // namespace tiling