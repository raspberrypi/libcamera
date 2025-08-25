#pragma once

#include "../decompand_status.h"

#include "algorithm.h"

namespace RPiController {

struct DecompandConfig {
	uint16_t decompandLUT_[65];
};

class Decompand : public Algorithm
{
public:
	Decompand(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	DecompandConfig config_;
};

} /* namespace RPiController */
