#pragma once

#include <libipa/pwl.h>

#include "../decompand_algorithm.h"
#include "../decompand_status.h"

namespace RPiController {

struct DecompandConfig {
	uint32_t bitdepth;
	libcamera::ipa::Pwl decompandCurve;
};

class Decompand : public DecompandAlgorithm
{
public:
	Decompand(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void initialValues(libcamera::ipa::Pwl &decompandCurve) override;
	void prepare(Metadata *imageMetadata) override;

private:
	CameraMode mode_;
	DecompandConfig config_;
};

} /* namespace RPiController */
