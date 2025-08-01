/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * AWB control algorithm
 */
#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include "../awb_algorithm.h"
#include "../awb_status.h"
#include "libipa/pwl.h"

namespace RPiController {

struct AwbMode {
	int read(const libcamera::YamlObject &params);
	double ctLo; /* low CT value for search */
	double ctHi; /* high CT value for search */
};

struct AwbConfig {
	AwbConfig()
		: defaultMode(nullptr) {}
	int read(const libcamera::YamlObject &params);
	bool hasCtCurve() const;

	/* Only repeat the AWB calculation every "this many" frames */
	uint16_t framePeriod;
	/* number of initial frames for which speed taken as 1.0 (maximum) */
	uint16_t startupFrames;
	unsigned int convergenceFrames; /* approx number of frames to converge */
	double speed; /* IIR filter speed applied to algorithm results */
	libcamera::ipa::Pwl ctR; /* function maps CT to r (= R/G) */
	libcamera::ipa::Pwl ctB; /* function maps CT to b (= B/G) */
	libcamera::ipa::Pwl ctRInverse; /* inverse of ctR */
	libcamera::ipa::Pwl ctBInverse; /* inverse of ctB */

	/* AWB "modes" (determines the search range) */
	std::map<std::string, AwbMode> modes;
	AwbMode *defaultMode; /* mode used if no mode selected */

	/* clamp on colour error term (so as not to penalise non-grey excessively) */
	double deltaLimit;
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

	bool greyWorld; /* don't use the ct curve when in grey world mode */
};

class Awb : public AwbAlgorithm
{
public:
	Awb(Controller *controller = NULL);
	~Awb();
	virtual void initialise() override;
	unsigned int getConvergenceFrames() const override;
	void initialValues(double &gainR, double &gainB) override;
	void setMode(std::string const &name) override;
	void setManualGains(double manualR, double manualB) override;
	void setColourTemperature(double temperatureK) override;
	void enableAuto() override;
	void disableAuto() override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

	static double interpolateQuadatric(libcamera::ipa::Pwl::Point const &a,
					   libcamera::ipa::Pwl::Point const &b,
					   libcamera::ipa::Pwl::Point const &c);

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

protected:
	/* configuration is read-only, and available to both threads */
	AwbConfig config_;
	/*
	 * The following are for the asynchronous thread to use, though the main
	 * thread can set/reset them if the async thread is known to be idle:
	 */
	std::vector<RGB> zones_;
	StatisticsPtr statistics_;
	double lux_;
	AwbMode *mode_;
	AwbStatus asyncResults_;

	virtual void doAwb() = 0;
	virtual void prepareStats() = 0;
	double computeDelta2Sum(double gainR, double gainB, double whitepointR, double whitepointB);
	void awbGrey();
	static void generateStats(std::vector<Awb::RGB> &zones,
				  StatisticsPtr &stats, double minPixels,
				  double minG, Metadata &globalMetadata,
				  double biasProportion, double biasCtR, double biasCtB);

private:
	bool isAutoEnabled() const;
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

} // namespace RPiController
