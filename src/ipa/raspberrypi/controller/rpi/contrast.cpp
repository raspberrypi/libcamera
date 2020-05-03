/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * contrast.cpp - contrast (gamma) control algorithm
 */
#include <stdint.h>

#include "../contrast_status.h"
#include "../histogram.hpp"

#include "contrast.hpp"

using namespace RPi;

// This is a very simple control algorithm which simply retrieves the results of
// AGC and AWB via their "status" metadata, and applies digital gain to the
// colour channels in accordance with those instructions. We take care never to
// apply less than unity gains, as that would cause fully saturated pixels to go
// off-white.

#define NAME "rpi.contrast"

Contrast::Contrast(Controller *controller)
	: ContrastAlgorithm(controller), brightness_(0.0), contrast_(1.0)
{
}

char const *Contrast::Name() const
{
	return NAME;
}

void Contrast::Read(boost::property_tree::ptree const &params)
{
	// enable adaptive enhancement by default
	config_.ce_enable = params.get<int>("ce_enable", 1);
	// the point near the bottom of the histogram to move
	config_.lo_histogram = params.get<double>("lo_histogram", 0.01);
	// where in the range to try and move it to
	config_.lo_level = params.get<double>("lo_level", 0.015);
	// but don't move by more than this
	config_.lo_max = params.get<double>("lo_max", 500);
	// equivalent values for the top of the histogram...
	config_.hi_histogram = params.get<double>("hi_histogram", 0.95);
	config_.hi_level = params.get<double>("hi_level", 0.95);
	config_.hi_max = params.get<double>("hi_max", 2000);
	config_.gamma_curve.Read(params.get_child("gamma_curve"));
}

void Contrast::SetBrightness(double brightness)
{
	brightness_ = brightness;
}

void Contrast::SetContrast(double contrast)
{
	contrast_ = contrast;
}

static void fill_in_status(ContrastStatus &status, double brightness,
			   double contrast, Pwl &gamma_curve)
{
	status.brightness = brightness;
	status.contrast = contrast;
	for (int i = 0; i < CONTRAST_NUM_POINTS - 1; i++) {
		int x = i < 16 ? i * 1024
			       : (i < 24 ? (i - 16) * 2048 + 16384
					 : (i - 24) * 4096 + 32768);
		status.points[i].x = x;
		status.points[i].y = std::min(65535.0, gamma_curve.Eval(x));
	}
	status.points[CONTRAST_NUM_POINTS - 1].x = 65535;
	status.points[CONTRAST_NUM_POINTS - 1].y = 65535;
}

void Contrast::Initialise()
{
	// Fill in some default values as Prepare will run before Process gets
	// called.
	fill_in_status(status_, brightness_, contrast_, config_.gamma_curve);
}

void Contrast::Prepare(Metadata *image_metadata)
{
	std::unique_lock<std::mutex> lock(mutex_);
	image_metadata->Set("contrast.status", status_);
}

Pwl compute_stretch_curve(Histogram const &histogram,
			  ContrastConfig const &config)
{
	Pwl enhance;
	enhance.Append(0, 0);
	// If the start of the histogram is rather empty, try to pull it down a
	// bit.
	double hist_lo = histogram.Quantile(config.lo_histogram) *
			 (65536 / NUM_HISTOGRAM_BINS);
	double level_lo = config.lo_level * 65536;
	RPI_LOG("Move histogram point " << hist_lo << " to " << level_lo);
	hist_lo = std::max(
		level_lo,
		std::min(65535.0, std::min(hist_lo, level_lo + config.lo_max)));
	RPI_LOG("Final values " << hist_lo << " -> " << level_lo);
	enhance.Append(hist_lo, level_lo);
	// Keep the mid-point (median) in the same place, though, to limit the
	// apparent amount of global brightness shift.
	double mid = histogram.Quantile(0.5) * (65536 / NUM_HISTOGRAM_BINS);
	enhance.Append(mid, mid);

	// If the top to the histogram is empty, try to pull the pixel values
	// there up.
	double hist_hi = histogram.Quantile(config.hi_histogram) *
			 (65536 / NUM_HISTOGRAM_BINS);
	double level_hi = config.hi_level * 65536;
	RPI_LOG("Move histogram point " << hist_hi << " to " << level_hi);
	hist_hi = std::min(
		level_hi,
		std::max(0.0, std::max(hist_hi, level_hi - config.hi_max)));
	RPI_LOG("Final values " << hist_hi << " -> " << level_hi);
	enhance.Append(hist_hi, level_hi);
	enhance.Append(65535, 65535);
	return enhance;
}

Pwl apply_manual_contrast(Pwl const &gamma_curve, double brightness,
			  double contrast)
{
	Pwl new_gamma_curve;
	RPI_LOG("Manual brightness " << brightness << " contrast " << contrast);
	gamma_curve.Map([&](double x, double y) {
		new_gamma_curve.Append(
			x, std::max(0.0, std::min(65535.0,
						  (y - 32768) * contrast +
							  32768 + brightness)));
	});
	return new_gamma_curve;
}

void Contrast::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	(void)image_metadata;
	double brightness = brightness_, contrast = contrast_;
	Histogram histogram(stats->hist[0].g_hist, NUM_HISTOGRAM_BINS);
	// We look at the histogram and adjust the gamma curve in the following
	// ways: 1. Adjust the gamma curve so as to pull the start of the
	// histogram down, and possibly push the end up.
	Pwl gamma_curve = config_.gamma_curve;
	if (config_.ce_enable) {
		if (config_.lo_max != 0 || config_.hi_max != 0)
			gamma_curve = compute_stretch_curve(histogram, config_)
					      .Compose(gamma_curve);
		// We could apply other adjustments (e.g. partial equalisation)
		// based on the histogram...?
	}
	// 2. Finally apply any manually selected brightness/contrast
	// adjustment.
	if (brightness != 0 || contrast != 1.0)
		gamma_curve = apply_manual_contrast(gamma_curve, brightness,
						    contrast);
	// And fill in the status for output. Use more points towards the bottom
	// of the curve.
	ContrastStatus status;
	fill_in_status(status, brightness, contrast, gamma_curve);
	{
		std::unique_lock<std::mutex> lock(mutex_);
		status_ = status;
	}
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Contrast(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
