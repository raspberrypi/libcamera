#include <libcamera/base/log.h>

#include "../decompand_status.h"

#include "decompand.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiDecompand)

#define NAME "rpi.decompand"

Decompand::Decompand(Controller *controller)
	: DecompandAlgorithm(controller)
{
}

char const *Decompand::name() const
{
	return NAME;
}

int Decompand::read(const libcamera::YamlObject &params)
{
	if (!params.contains("lut") || !params["lut"].isList() || params["lut"].size() != 65) {
		LOG(RPiDecompand, Error) << "Expected LUT with 65 elements";
		return -EINVAL;
	}

	for (unsigned int i = 0; i < 65; ++i) {
		std::optional<uint16_t> value = params["lut"][i].get<uint16_t>();
		if (!value.has_value()) {
			LOG(RPiDecompand, Error) << "Invalid LUT value at index " << i;
			return -EINVAL;
		}
		decompandLUT_[i] = value.value();
	}

	return 0;
}

void Decompand::initialValues(uint16_t LUT[])
{
  for (size_t i = 0; i < sizeof(decompandLUT_) / sizeof(decompandLUT_[0]); ++i)
	{
    LUT[i] = decompandLUT_[i];
	}
}

void Decompand::prepare(Metadata *imageMetadata)
{
	struct DecompandStatus status;
	for (size_t i = 0; i < sizeof(decompandLUT_) / sizeof(decompandLUT_[0]); ++i)
	{
    status.lut[i] = decompandLUT_[i];
	}

	imageMetadata->set("decompand.status", status);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new Decompand(controller);
}

static RegisterAlgorithm reg(NAME, &create);
