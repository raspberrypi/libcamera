/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * contrast.cpp - contrast (gamma) control algorithm
 */
#include <stdint.h>

#include <libcamera/base/log.h>

#include "../contrast_status.h"
#include "../histogram.h"

#include "contrast.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiContrast)

/*
 * This is a very simple control algorithm which simply retrieves the results of
 * AGC and AWB via their "status" metadata, and applies digital gain to the
 * colour channels in accordance with those instructions. We take care never to
 * apply less than unity gains, as that would cause fully saturated pixels to go
 * off-white.
 */

#define NAME "rpi.contrast"

Contrast::Contrast(Controller *controller)
	: ContrastAlgorithm(controller), brightness_(0.0), contrast_(1.0)
{
}

char const *Contrast::name() const
{
	return NAME;
}

int Contrast::read(const libcamera::YamlObject &params)
{
	// enable adaptive enhancement by default
	config_.ceEnable = params["ce_enable"].get<int>(1);
	// the point near the bottom of the histogram to move
	config_.loHistogram = params["lo_histogram"].get<double>(0.01);
	// where in the range to try and move it to
	config_.loLevel = params["lo_level"].get<double>(0.015);
	// but don't move by more than this
	config_.loMax = params["lo_max"].get<double>(500);
	// equivalent values for the top of the histogram...
	config_.hiHistogram = params["hi_histogram"].get<double>(0.95);
	config_.hiLevel = params["hi_level"].get<double>(0.95);
	config_.hiMax = params["hi_max"].get<double>(2000);
	return config_.gammaCurve.read(params["gamma_curve"]);
}

void Contrast::setBrightness(double brightness)
{
	brightness_ = brightness;
}

void Contrast::setContrast(double contrast)
{
	contrast_ = contrast;
}

static void fillInStatus(ContrastStatus &status, double brightness,
			 double contrast, Pwl &gammaCurve)
{
	status.brightness = brightness;
	status.contrast = contrast;
	for (unsigned int i = 0; i < ContrastNumPoints - 1; i++) {
		int x = i < 16 ? i * 1024
			       : (i < 24 ? (i - 16) * 2048 + 16384
					 : (i - 24) * 4096 + 32768);
		status.points[i].x = x;
		status.points[i].y = std::min(65535.0, gammaCurve.eval(x));
	}
	status.points[ContrastNumPoints - 1].x = 65535;
	status.points[ContrastNumPoints - 1].y = 65535;
}

void Contrast::initialise()
{
	/*
	 * Fill in some default values as Prepare will run before Process gets
	 * called.
	 */
	fillInStatus(status_, brightness_, contrast_, config_.gammaCurve);
}

void Contrast::prepare(Metadata *imageMetadata)
{
	std::unique_lock<std::mutex> lock(mutex_);
	imageMetadata->set("contrast.status", status_);
}

Pwl computeStretchCurve(Histogram const &histogram,
			ContrastConfig const &config)
{
	Pwl enhance;
	enhance.append(0, 0);
	/*
	 * If the start of the histogram is rather empty, try to pull it down a
	 * bit.
	 */
	double histLo = histogram.quantile(config.loHistogram) *
			(65536 / NUM_HISTOGRAM_BINS);
	double levelLo = config.loLevel * 65536;
	LOG(RPiContrast, Debug)
		<< "Move histogram point " << histLo << " to " << levelLo;
	histLo = std::max(levelLo,
			  std::min(65535.0, std::min(histLo, levelLo + config.loMax)));
	LOG(RPiContrast, Debug)
		<< "Final values " << histLo << " -> " << levelLo;
	enhance.append(histLo, levelLo);
	/*
	 * Keep the mid-point (median) in the same place, though, to limit the
	 * apparent amount of global brightness shift.
	 */
	double mid = histogram.quantile(0.5) * (65536 / NUM_HISTOGRAM_BINS);
	enhance.append(mid, mid);

	/*
	 * If the top to the histogram is empty, try to pull the pixel values
	 * there up.
	 */
	double histHi = histogram.quantile(config.hiHistogram) *
			(65536 / NUM_HISTOGRAM_BINS);
	double levelHi = config.hiLevel * 65536;
	LOG(RPiContrast, Debug)
		<< "Move histogram point " << histHi << " to " << levelHi;
	histHi = std::min(levelHi,
			  std::max(0.0, std::max(histHi, levelHi - config.hiMax)));
	LOG(RPiContrast, Debug)
		<< "Final values " << histHi << " -> " << levelHi;
	enhance.append(histHi, levelHi);
	enhance.append(65535, 65535);
	return enhance;
}

Pwl applyManualContrast(Pwl const &gammaCurve, double brightness,
			double contrast)
{
	Pwl newGammaCurve;
	LOG(RPiContrast, Debug)
		<< "Manual brightness " << brightness << " contrast " << contrast;
	gammaCurve.map([&](double x, double y) {
		newGammaCurve.append(
			x, std::max(0.0, std::min(65535.0,
						  (y - 32768) * contrast +
							  32768 + brightness)));
	});
	return newGammaCurve;
}

void Contrast::process(StatisticsPtr &stats,
		       [[maybe_unused]] Metadata *imageMetadata)
{
	Histogram histogram(stats->hist[0].g_hist, NUM_HISTOGRAM_BINS);
	/*
	 * We look at the histogram and adjust the gamma curve in the following
	 * ways: 1. Adjust the gamma curve so as to pull the start of the
	 * histogram down, and possibly push the end up.
	 */
	Pwl gammaCurve = config_.gammaCurve;
	if (config_.ceEnable) {
		if (config_.loMax != 0 || config_.hiMax != 0)
			gammaCurve = computeStretchCurve(histogram, config_).compose(gammaCurve);
		/*
		 * We could apply other adjustments (e.g. partial equalisation)
		 * based on the histogram...?
		 */
	}
	/*
	 * 2. Finally apply any manually selected brightness/contrast
	 * adjustment.
	 */
	if (brightness_ != 0 || contrast_ != 1.0)
		gammaCurve = applyManualContrast(gammaCurve, brightness_, contrast_);
	/*
	 * And fill in the status for output. Use more points towards the bottom
	 * of the curve.
	 */
	ContrastStatus status;
	fillInStatus(status, brightness_, contrast_, gammaCurve);
	{
		std::unique_lock<std::mutex> lock(mutex_);
		status_ = status;
	}
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Contrast(controller);
}
static RegisterAlgorithm reg(NAME, &create);
