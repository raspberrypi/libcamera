/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * sharpen.cpp - sharpening control algorithm
 */

#include <math.h>

#include <libcamera/base/log.h>

#include "../sharpen_status.h"

#include "sharpen.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSharpen)

#define NAME "rpi.sharpen"

Sharpen::Sharpen(Controller *controller)
	: SharpenAlgorithm(controller), userStrength_(1.0)
{
}

char const *Sharpen::name() const
{
	return NAME;
}

void Sharpen::switchMode(CameraMode const &cameraMode,
			 [[maybe_unused]] Metadata *metadata)
{
	/* can't be less than one, right? */
	modeFactor_ = std::max(1.0, cameraMode.noiseFactor);
}

int Sharpen::read(const libcamera::YamlObject &params)
{
	threshold_ = params["threshold"].get<double>(1.0);
	strength_ = params["strength"].get<double>(1.0);
	limit_ = params["limit"].get<double>(1.0);
	LOG(RPiSharpen, Debug)
		<< "Read threshold " << threshold_
		<< " strength " << strength_
		<< " limit " << limit_;
	return 0;
}

void Sharpen::setStrength(double strength)
{
	/*
	 * Note that this function is how an application sets the overall
	 * sharpening "strength". We call this the "user strength" field
	 * as there already is a strength_ field - being an internal gain
	 * parameter that gets passed to the ISP control code. Negative
	 * values are not allowed - coerce them to zero (no sharpening).
	 */
	userStrength_ = std::max(0.0, strength);
}

void Sharpen::prepare(Metadata *imageMetadata)
{
	/*
	 * The userStrength_ affects the algorithm's internal gain directly, but
	 * we adjust the limit and threshold less aggressively. Using a sqrt
	 * function is an arbitrary but gentle way of accomplishing this.
	 */
	double userStrengthSqrt = sqrt(userStrength_);
	struct SharpenStatus status;
	/*
	 * Binned modes seem to need the sharpening toned down with this
	 * pipeline, thus we use the modeFactor_ here. Also avoid
	 * divide-by-zero with the userStrengthSqrt.
	 */
	status.threshold = threshold_ * modeFactor_ /
			   std::max(0.01, userStrengthSqrt);
	status.strength = strength_ / modeFactor_ * userStrength_;
	status.limit = limit_ / modeFactor_ * userStrengthSqrt;
	/* Finally, report any application-supplied parameters that were used. */
	status.userStrength = userStrength_;
	imageMetadata->set("sharpen.status", status);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new Sharpen(controller);
}
static RegisterAlgorithm reg(NAME, &create);
