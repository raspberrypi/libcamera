/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * sharpen.cpp - sharpening control algorithm
 */

#include <math.h>

#include "../logging.hpp"
#include "../sharpen_status.h"

#include "sharpen.hpp"

using namespace RPi;

#define NAME "rpi.sharpen"

Sharpen::Sharpen(Controller *controller)
	: SharpenAlgorithm(controller), user_strength_(1.0)
{
}

char const *Sharpen::Name() const
{
	return NAME;
}

void Sharpen::SwitchMode(CameraMode const &camera_mode, Metadata *metadata)
{
	(void)metadata;

	// can't be less than one, right?
	mode_factor_ = std::max(1.0, camera_mode.noise_factor);
}

void Sharpen::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG(Name());
	threshold_ = params.get<double>("threshold", 1.0);
	strength_ = params.get<double>("strength", 1.0);
	limit_ = params.get<double>("limit", 1.0);
}

void Sharpen::SetStrength(double strength)
{
	// Note that this method is how an application sets the overall
	// sharpening "strength". We call this the "user strength" field
	// as there already is a strength_ field - being an internal gain
	// parameter that gets passed to the ISP control code. Negative
	// values are not allowed - coerce them to zero (no sharpening).
	user_strength_ = std::max(0.0, strength);
}

void Sharpen::Prepare(Metadata *image_metadata)
{
	// The user_strength_ affects the algorithm's internal gain directly, but
	// we adjust the limit and threshold less aggressively. Using a sqrt
	// function is an arbitrary but gentle way of accomplishing this.
	double user_strength_sqrt = sqrt(user_strength_);
	struct SharpenStatus status;
	// Binned modes seem to need the sharpening toned down with this
	// pipeline, thus we use the mode_factor here. Also avoid
	// divide-by-zero with the user_strength_sqrt.
	status.threshold = threshold_ * mode_factor_ /
			   std::max(0.01, user_strength_sqrt);
	status.strength = strength_ / mode_factor_ * user_strength_;
	status.limit = limit_ / mode_factor_ * user_strength_sqrt;
	// Finally, report any application-supplied parameters that were used.
	status.user_strength = user_strength_;
	image_metadata->Set("sharpen.status", status);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return new Sharpen(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
