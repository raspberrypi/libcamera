#pragma once

#include "../decompand_algorithm.h"
#include "../decompand_status.h"

namespace RPiController {

class Decompand : public DecompandAlgorithm
{
public:
	Decompand(Controller *controller);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialValues(uint16_t LUT[], uint16_t &pad) override;
	void prepare(Metadata *imageMetadata) override;

private:
	uint16_t decompandLUT_[65];
	uint16_t decompandpad_;
};

} /* namespace RPiController */
