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
	: Algorithm(controller)
{
}

char const *Sharpen::Name() const
{
	return NAME;
}

void Sharpen::SwitchMode(CameraMode const &camera_mode)
{
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

void Sharpen::Prepare(Metadata *image_metadata)
{
	double mode_factor = mode_factor_;
	struct SharpenStatus status;
	// Binned modes seem to need the sharpening toned down with this
	// pipeline.
	status.threshold = threshold_ * mode_factor;
	status.strength = strength_ / mode_factor;
	status.limit = limit_ / mode_factor;
	image_metadata->Set("sharpen.status", status);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return new Sharpen(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
