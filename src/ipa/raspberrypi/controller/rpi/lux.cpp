/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * lux.cpp - Lux control algorithm
 */
#include <math.h>

#include "linux/bcm2835-isp.h"

#include "../device_status.h"
#include "../logging.hpp"

#include "lux.hpp"

using namespace RPi;

#define NAME "rpi.lux"

Lux::Lux(Controller *controller)
	: Algorithm(controller)
{
	// Put in some defaults as there will be no meaningful values until
	// Process has run.
	status_.aperture = 1.0;
	status_.lux = 400;
}

char const *Lux::Name() const
{
	return NAME;
}

void Lux::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG(Name());
	reference_shutter_speed_ =
		params.get<double>("reference_shutter_speed");
	reference_gain_ = params.get<double>("reference_gain");
	reference_aperture_ = params.get<double>("reference_aperture", 1.0);
	reference_Y_ = params.get<double>("reference_Y");
	reference_lux_ = params.get<double>("reference_lux");
	current_aperture_ = reference_aperture_;
}

void Lux::Prepare(Metadata *image_metadata)
{
	std::unique_lock<std::mutex> lock(mutex_);
	image_metadata->Set("lux.status", status_);
}

void Lux::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	// set some initial values to shut the compiler up
	DeviceStatus device_status =
		{ .shutter_speed = 1.0,
		  .analogue_gain = 1.0,
		  .lens_position = 0.0,
		  .aperture = 0.0,
		  .flash_intensity = 0.0 };
	if (image_metadata->Get("device.status", device_status) == 0) {
		double current_gain = device_status.analogue_gain;
		double current_shutter_speed = device_status.shutter_speed;
		double current_aperture = device_status.aperture;
		if (current_aperture == 0)
			current_aperture = current_aperture_;
		uint64_t sum = 0;
		uint32_t num = 0;
		uint32_t *bin = stats->hist[0].g_hist;
		const int num_bins = sizeof(stats->hist[0].g_hist) /
				     sizeof(stats->hist[0].g_hist[0]);
		for (int i = 0; i < num_bins; i++)
			sum += bin[i] * (uint64_t)i, num += bin[i];
		// add .5 to reflect the mid-points of bins
		double current_Y = sum / (double)num + .5;
		double gain_ratio = reference_gain_ / current_gain;
		double shutter_speed_ratio =
			reference_shutter_speed_ / current_shutter_speed;
		double aperture_ratio = reference_aperture_ / current_aperture;
		double Y_ratio = current_Y * (65536 / num_bins) / reference_Y_;
		double estimated_lux = shutter_speed_ratio * gain_ratio *
				       aperture_ratio * aperture_ratio *
				       Y_ratio * reference_lux_;
		LuxStatus status;
		status.lux = estimated_lux;
		status.aperture = current_aperture;
		RPI_LOG(Name() << ": estimated lux " << estimated_lux);
		{
			std::unique_lock<std::mutex> lock(mutex_);
			status_ = status;
		}
		// Overwrite the metadata here as well, so that downstream
		// algorithms get the latest value.
		image_metadata->Set("lux.status", status);
	} else
		RPI_WARN(Name() << ": no device metadata");
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Lux(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
