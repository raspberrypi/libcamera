#pragma once

#include "stages.h"

namespace tiling {

class CropStage : public BasicStage
{
public:
	struct Config {
		Config(Interval2 const &_crop)
			: crop(_crop) {}
		Interval2 crop;
	};
	CropStage(char const *name, Stage *upstream, Config const &config, int struct_offset);
	virtual Length2 GetOutputImageSize() const;
	virtual void PushStartUp(int output_start, Dir dir);
	virtual int PushEndDown(int input_end, Dir dir);
	virtual void PushEndUp(int output_end, Dir dir);
	virtual void PushCropDown(Interval interval, Dir dir);

private:
	Config config_;
};

} // namespace tiling