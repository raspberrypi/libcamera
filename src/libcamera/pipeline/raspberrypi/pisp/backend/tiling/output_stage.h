#pragma once

#include "stages.h"

namespace tiling {

class OutputStage : public BasicStage
{
public:
	struct Config {
		Config(Length2 const &_max_alignment, Length2 const &_min_alignment, bool _x_mirrored)
			: max_alignment(_max_alignment), min_alignment(_min_alignment), x_mirrored(_x_mirrored) {}
		Length2 max_alignment;
		Length2 min_alignment;
		bool x_mirrored;
	};
	OutputStage(char const *name, Stage *upstream, Config const &config, int struct_offset);
	virtual Interval GetOutputInterval() const;
	virtual void PushStartUp(int output_start, Dir dir);
	virtual int PushEndDown(int input_end, Dir dir);
	virtual void PushEndUp(int output_end, Dir dir);
	virtual void PushCropDown(Interval interval, Dir dir);
	bool Done(Dir dir) const;

private:
	Config config_;
};

} // namespace tiling