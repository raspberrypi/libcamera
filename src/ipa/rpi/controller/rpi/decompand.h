#pragma once

#include <libipa/pwl.h>
#include "algorithm.h"

#include "../decompand_status.h"

namespace RPiController {

struct DecompandConfig {
	libcamera::ipa::Pwl decompandCurve;
};

class Decompand : public Algorithm
{
public:
	Decompand(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;

private:
	DecompandConfig config_;
};

} /* namespace RPiController */
