#pragma once

#include "stages.h"

namespace tiling {

class RescaleStage : public BasicStage
{
public:
	enum class RescalerType { Downscaler, Resampler };
	struct Config {
		Config(Length2 const &_output_image_size, Length2 const &_scale, Length2 const &_start_context, Length2 const &_end_context, uint8_t _precision, RescalerType rescaler_type_)
			: output_image_size(_output_image_size), scale(_scale), start_context(_start_context), end_context(_end_context), precision(_precision),
			  rescaler_type(rescaler_type_)
		{
		}
		Length2 output_image_size;
		Length2 scale;
		Length2 start_context;
		Length2 end_context;
		uint8_t precision;
		RescalerType rescaler_type;
	};

	RescaleStage(char const *name, Stage *upstream, Config const &config, int struct_offset);
	virtual Length2 GetOutputImageSize() const;
	virtual void PushStartUp(int output_start, Dir dir);
	virtual int PushEndDown(int input_end, Dir dir);
	virtual void PushEndUp(int output_end, Dir dir);
	virtual void PushCropDown(Interval interval, Dir dir);

private:
	Config config_;
	uint32_t round_up;
};

} // namespace tiling
