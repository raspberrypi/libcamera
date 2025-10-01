#include "decompand.h"

#include <libcamera/base/log.h>

#include "../decompand_status.h"
#include "../histogram.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiDecompand)

#define NAME "rpi.decompand"

Decompand::Decompand(Controller *controller)
	: Algorithm(controller)
{
}

char const *Decompand::name() const
{
	return NAME;
}

int Decompand::read(const libcamera::YamlObject &params)
{
	config_.bitdepth = params["bitdepth"].get<uint32_t>(0);
	config_.decompandCurve = params["decompand_curve"].get<ipa::Pwl>(ipa::Pwl{});
	return config_.decompandCurve.empty() ? -EINVAL : 0;
}

void Decompand::initialise()
{
}

void Decompand::switchMode([[maybe_unused]] CameraMode const &cameraMode,
			   [[maybe_unused]] Metadata *metadata)
{
	mode_ = cameraMode;
}

void Decompand::prepare(Metadata *imageMetadata)
{
	DecompandStatus decompandStatus;

	if (config_.bitdepth == 0 || mode_.bitdepth == config_.bitdepth) {
		decompandStatus.decompandCurve = config_.decompandCurve;
		imageMetadata->set("decompand.status", decompandStatus);
	}
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Decompand(controller);
}

static RegisterAlgorithm reg(NAME, &create);
