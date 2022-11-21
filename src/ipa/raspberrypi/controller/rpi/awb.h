/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * awb.h - AWB control algorithm
 */
#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>

#include "../awb_algorithm.h"
#include "../pwl.h"
#include "../awb_status.h"

namespace RPiController {

/* Control algorithm to perform AWB calculations. */

struct AwbMode {
	int read(const libcamera::YamlObject &params);
	double ctLo; /* low CT value for search */
	double ctHi; /* high CT value for search */
};

struct AwbPrior {
	int read(const libcamera::YamlObject &params);
	double lux; /* lux level */
	Pwl prior; /* maps CT to prior log likelihood for this lux level */
};

struct AwbConfig {
	AwbConfig() : defaultMode(nullptr) {}
	int read(const libcamera::YamlObject &params);
	/* Only repeat the AWB calculation every "this many" frames */
	uint16_t framePeriod;
	/* number of initial frames for which speed taken as 1.0 (maximum) */
	uint16_t startupFrames;
	unsigned int convergenceFrames; /* approx number of frames to converge */
	double speed; /* IIR filter speed applied to algorithm results */
	bool fast; /* "fast" mode uses a 16x16 rather than 32x32 grid */
	Pwl ctR; /* function maps CT to r (= R/G) */
	Pwl ctB; /* function maps CT to b (= B/G) */
	Pwl ctRInverse; /* inverse of ctR */
	Pwl ctBInverse; /* inverse of ctB */
	/* table of illuminant priors at different lux levels */
	std::vector<AwbPrior> priors;
	/* AWB "modes" (determines the search range) */
	std::map<std::string, AwbMode> modes;
	AwbMode *defaultMode; /* mode used if no mode selected */
	/*
	 * minimum proportion of pixels counted within AWB region for it to be
	 * "useful"
	 */
	double minPixels;
	/* minimum G value of those pixels, to be regarded a "useful" */
	uint16_t minG;
	/*
	 * number of AWB regions that must be "useful" in order to do the AWB
	 * calculation
	 */
	uint32_t minRegions;
	/* clamp on colour error term (so as not to penalise non-grey excessively) */
	double deltaLimit;
	/* step size control in coarse search */
	double coarseStep;
	/* how far to wander off CT curve towards "more purple" */
	double transversePos;
	/* how far to wander off CT curve towards "more green" */
	double transverseNeg;
	/*
	 * red sensitivity ratio (set to canonical sensor's R/G divided by this
	 * sensor's R/G)
	 */
	double sensitivityR;
	/*
	 * blue sensitivity ratio (set to canonical sensor's B/G divided by this
	 * sensor's B/G)
	 */
	double sensitivityB;
	/* The whitepoint (which we normally "aim" for) can be moved. */
	double whitepointR;
	double whitepointB;
	bool bayes; /* use Bayesian algorithm */
};

class Awb : public AwbAlgorithm
{
public:
	Awb(Controller *controller = NULL);
	~Awb();
	char const *name() const override;
	void initialise() override;
	int read(const libcamera::YamlObject &params) override;
	unsigned int getConvergenceFrames() const override;
	void setMode(std::string const &name) override;
	void setManualGains(double manualR, double manualB) override;
	void enableAuto() override;
	void disableAuto() override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;
	struct RGB {
		RGB(double r = 0, double g = 0, double b = 0)
			: R(r), G(g), B(b)
		{
		}
		double R, G, B;
		RGB &operator+=(RGB const &other)
		{
			R += other.R, G += other.G, B += other.B;
			return *this;
		}
	};

private:
	bool isAutoEnabled() const;
	/* configuration is read-only, and available to both threads */
	AwbConfig config_;
	std::thread asyncThread_;
	void asyncFunc(); /* asynchronous thread function */
	std::mutex mutex_;
	/* condvar for async thread to wait on */
	std::condition_variable asyncSignal_;
	/* condvar for synchronous thread to wait on */
	std::condition_variable syncSignal_;
	/* for sync thread to check  if async thread finished (requires mutex) */
	bool asyncFinished_;
	/* for async thread to check if it's been told to run (requires mutex) */
	bool asyncStart_;
	/* for async thread to check if it's been told to quit (requires mutex) */
	bool asyncAbort_;

	/*
	 * The following are only for the synchronous thread to use:
	 * for sync thread to note its has asked async thread to run
	 */
	bool asyncStarted_;
	/* counts up to framePeriod before restarting the async thread */
	int framePhase_;
	int frameCount_; /* counts up to startup_frames */
	AwbStatus syncResults_;
	AwbStatus prevSyncResults_;
	std::string modeName_;
	/*
	 * The following are for the asynchronous thread to use, though the main
	 * thread can set/reset them if the async thread is known to be idle:
	 */
	void restartAsync(StatisticsPtr &stats, double lux);
	/* copy out the results from the async thread so that it can be restarted */
	void fetchAsyncResults();
	StatisticsPtr statistics_;
	AwbMode *mode_;
	double lux_;
	AwbStatus asyncResults_;
	void doAwb();
	void awbBayes();
	void awbGrey();
	void prepareStats();
	double computeDelta2Sum(double gainR, double gainB);
	Pwl interpolatePrior();
	double coarseSearch(Pwl const &prior);
	void fineSearch(double &t, double &r, double &b, Pwl const &prior);
	std::vector<RGB> zones_;
	std::vector<Pwl::Point> points_;
	/* manual r setting */
	double manualR_;
	/* manual b setting */
	double manualB_;
};

static inline Awb::RGB operator+(Awb::RGB const &a, Awb::RGB const &b)
{
	return Awb::RGB(a.R + b.R, a.G + b.G, a.B + b.B);
}
static inline Awb::RGB operator-(Awb::RGB const &a, Awb::RGB const &b)
{
	return Awb::RGB(a.R - b.R, a.G - b.G, a.B - b.B);
}
static inline Awb::RGB operator*(double d, Awb::RGB const &rgb)
{
	return Awb::RGB(d * rgb.R, d * rgb.G, d * rgb.B);
}
static inline Awb::RGB operator*(Awb::RGB const &rgb, double d)
{
	return d * rgb;
}

} /* namespace RPiController */
