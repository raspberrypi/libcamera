
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af.cpp - PDAF control algorithm
 */

#include "af.h"

#include <iomanip>
#include <math.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiAf)

/*
 * This algorithm implements a hybrid of CDAF and PDAF, favouring PDAF.
 *
 * Whenever PDAF is available, it is used in a continuous feedback loop.
 * When triggered in auto mode, we simply enable AF for a limited number
 * frames; the use of both proportional and integral control coefficients
 * should allow it to converge in finite time.
 *
 * When PDAF confidence is low (due e.g. to low contrast or extreme defocus)
 * or PDAF data are absent, fall back to CDAF with a programmed scan pattern.
 * A coarse and fine scan are performed, using ISP's CDAF focus FoM to
 * estimate the lens position with peak contrast. This is slower due to
 * extra latency in the ISP, and requires a settling time between steps.
 *
 * Some hysteresis is applied to the switch between PDAF and CDAF, to avoid
 * "nuisance" scans. During each interval where PDAF is not working, only
 * ONE scan will be performed; CAF cannot track objects using CDAF alone.
 *
 * This algorithm is unrelated to "rpi.focus" which merely reports CDAF FoM.
 */

#define NAME "rpi.af"

/*
 * Default values for parameters. All may be overridden in the tuning file.
 * Many of these values are sensor- or module-dependent; the defaults here
 * assume IMX708 in a Raspberry Pi V3 camera with the standard lens.
 *
 * Here all focus values are in dioptres (1/m). They are converted to hardware
 * units when written to status.focusSetting or returned from setLensPosition.
 *
 * Gain and delay values are relative to frame interval, since much (not all)
 * of the delay is in the sensor and (for CDAF) ISP, not the lens mechanism.
 */

#define DEFAULT_FOCUS_MIN        0.0 /* infinity                           */
#define DEFAULT_FOCUS_MAX       12.0 /* closest focus, about 83 mm         */
#define DEFAULT_FOCUS_DEFAULT    1.0 /* about 1 m, guessed "hyperfocal"    */
#define DEFAULT_CONF_EPSILON       8 /* PDAF confidence hysteresis level   */
#define DEFAULT_CONF_THRESH       16 /* PDAF cell confidence threshold     */
#define DEFAULT_CONF_CLIP        512 /* PDAF cell confidence limit         */
#define DEFAULT_GAIN           -0.02 /* loop gain, should be small and -ve */
#define DEFAULT_SQUELCH        0.125 /* suppress small changes of lens pos */
#define DEFAULT_SKIP_FRAMES        5 /* initial frames to skip             */
#define DEFAULT_STEP_FRAMES        4 /* skip frames per step during scan   */
#define DEFAULT_STEP_COARSE      1.0 /* step size for coarse scan          */
#define DEFAULT_STEP_FINE       0.25 /* step size for fine scan            */
#define DEFAULT_MAX_SLEW         2.0 /* largest lens change per frame      */
#define DEFAULT_PDAF_FRAMES       16 /* Number of iterations in auto mode  */
#define DEFAULT_DROPOUT_FRAMES     6 /* Absence of PDAF to switch to CDAF  */
#define DEFAULT_CONTRAST_RATIO  0.75 /* min/max contrast ratio for success */
#define DEFAULT_MAP_X0          -1.0 /* slightly beyond infinity           */
#define DEFAULT_MAP_Y0           420 /* corresponding lens setting         */
#define DEFAULT_MAP_X1          15.0 /* close to physical end stop         */
#define DEFAULT_MAP_Y1           960 /* corresponding lens setting         */

Af::CfgParams::CfgParams()
	: focus_min{ DEFAULT_FOCUS_MIN, DEFAULT_FOCUS_MIN, DEFAULT_FOCUS_MIN },
	  focus_max{ DEFAULT_FOCUS_MAX, DEFAULT_FOCUS_MAX, DEFAULT_FOCUS_MAX },
	  focus_default{ DEFAULT_FOCUS_DEFAULT, DEFAULT_FOCUS_DEFAULT, DEFAULT_FOCUS_DEFAULT },
	  step_coarse{ DEFAULT_STEP_COARSE, DEFAULT_STEP_COARSE },
	  step_fine{ DEFAULT_STEP_FINE, DEFAULT_STEP_FINE },
	  contrast_ratio{ DEFAULT_CONTRAST_RATIO, DEFAULT_CONTRAST_RATIO },
	  pdaf_gain{ DEFAULT_GAIN, DEFAULT_GAIN },
	  pdaf_squelch{ DEFAULT_SQUELCH, DEFAULT_SQUELCH },
	  max_slew{ DEFAULT_MAX_SLEW, DEFAULT_MAX_SLEW },
	  pdaf_frames{ DEFAULT_PDAF_FRAMES, DEFAULT_PDAF_FRAMES },
	  dropout_frames{ DEFAULT_DROPOUT_FRAMES, DEFAULT_DROPOUT_FRAMES },
	  step_frames{ DEFAULT_STEP_FRAMES, DEFAULT_STEP_FRAMES },
	  conf_epsilon(DEFAULT_CONF_EPSILON),
	  conf_thresh(DEFAULT_CONF_THRESH),
	  conf_clip(DEFAULT_CONF_CLIP),
	  skip_frames(DEFAULT_SKIP_FRAMES),
	  map()
{
}

template<typename T>
static void readNumber(T &dest, const libcamera::YamlObject &params, char const *name)
{
	auto value = params[name].get<T>();
	if (!value) {
		LOG(RPiAf, Warning) << "Missing parameter \"" << name << "\"";
	} else {
		dest = *value;
	}
}

bool Af::CfgParams::readRange(int index, const libcamera::YamlObject &ranges, char const *name)
{
	if (ranges.contains(name)) {
		const libcamera::YamlObject &r = ranges[name];

		readNumber<double>(focus_min[index], r, "min");
		readNumber<double>(focus_max[index], r, "max");
		readNumber<double>(focus_default[index], r, "default");
		return true;
	}

	return false;
}

bool Af::CfgParams::readSpeed(int index, const libcamera::YamlObject &speeds, char const *name)
{
	if (speeds.contains(name)) {
		const libcamera::YamlObject &s = speeds[name];

		readNumber<double>(step_coarse[index], s, "step_coarse");
		readNumber<double>(step_fine[index], s, "step_fine");
		readNumber<double>(contrast_ratio[index], s, "contrast_ratio");
		readNumber<double>(pdaf_gain[index], s, "pdaf_gain");
		readNumber<double>(pdaf_squelch[index], s, "pdaf_squelch");
		readNumber<double>(max_slew[index], s, "max_slew");
		readNumber<uint32_t>(pdaf_frames[index], s, "pdaf_frames");
		readNumber<uint32_t>(dropout_frames[index], s, "dropout_frames");
		readNumber<uint32_t>(step_frames[index], s, "step_frames");
		return true;
	}

	return false;
}

int Af::CfgParams::read(const libcamera::YamlObject &params)
{
	if (params.contains("ranges")) {
		auto &ranges = params["ranges"];

		if (!readRange(0, ranges, "normal"))
			LOG(RPiAf, Warning) << "Missing range \"normal\"";
		focus_min[1] = focus_min[0];
		focus_max[1] = focus_max[0];
		focus_default[1] = focus_default[0];
		readRange(1, ranges, "macro");
		focus_min[2] = std::min(focus_min[0], focus_min[1]);
		focus_max[2] = std::max(focus_max[0], focus_max[1]);
		focus_default[2] = focus_default[0];
		readRange(2, ranges, "full");
	} else {
		LOG(RPiAf, Warning) << "No ranges defined";
	}

	if (params.contains("speeds")) {
		auto &speeds = params["speeds"];

		if (!readSpeed(0, speeds, "normal"))
			LOG(RPiAf, Warning) << "Missing speed \"normal\"";
		step_coarse[1]    = step_coarse[0];
		step_fine[1]      = step_fine[0];
		contrast_ratio[1] = contrast_ratio[0];
		pdaf_gain[1]      = pdaf_gain[0];
		pdaf_squelch[1]   = pdaf_squelch[0];
		max_slew[1]       = max_slew[0];
		pdaf_frames[1]    = pdaf_frames[0];
		dropout_frames[1] = dropout_frames[0];
		step_frames[1]    = step_frames[0];
		readSpeed(1, speeds, "fast");
	} else {
		LOG(RPiAf, Warning) << "No speeds defined";
	}

	readNumber<uint32_t>(conf_epsilon, params, "conf_epsilon");
	readNumber<uint32_t>(conf_thresh, params, "conf_thresh");
	readNumber<uint32_t>(conf_clip, params, "conf_clip");
	readNumber<uint32_t>(skip_frames, params, "skip_frames");

	if (params.contains("map"))
		map.read(params["map"]);
	else
		LOG(RPiAf, Warning) << "No map defined";

	return 0;
}

/* Af Algorithm class */

Af::Af(Controller *controller)
	: AfAlgorithm(controller),
	  cfg_(),
	  range_(AF_RANGE_NORMAL),
	  speed_(AF_SPEED_NORMAL),
	  sensorSize_{ 4608, 2592 },
	  cafEnabled_(false),
	  useWeights_(false),
	  phaseWeights_{},
	  contrastWeights_{},
	  scanState_(SCAN_IDLE),
	  ftarget_(-INFINITY),
	  fsmooth_(-INFINITY),
	  prevContrast_(0.0),
	  skipCount_(DEFAULT_SKIP_FRAMES),
	  stepCount_(0),
	  dropCount_(0),
	  scanMaxContrast_(0.0),
	  scanMinContrast_(1.0e9),
	  scanData_(),
	  reportState_(AfStatus::STATE_UNKNOWN)
{
	scanData_.reserve(24);
}

Af::~Af()
{
}

char const *Af::name() const
{
	return NAME;
}

void Af::initialise()
{
	if (cfg_.map.empty()) {
		cfg_.map.append(DEFAULT_MAP_X0, DEFAULT_MAP_Y0);
		cfg_.map.append(DEFAULT_MAP_X1, DEFAULT_MAP_Y1);
	}
}

void Af::switchMode(CameraMode const &camera_mode, Metadata *metadata)
{
	(void)metadata;
	sensorSize_.width = camera_mode.sensorWidth;
	sensorSize_.height = camera_mode.sensorHeight;

	if (scanState_ >= SCAN_COARSE && scanState_ < SCAN_SETTLE) {
		/* If a scan was in progress, re-start it, as CDAF statistics
		 * may have changed. Though if the application is just about
		 * to take a still picture, this will not help...
		 */
		startProgrammedScan();
	}
	skipCount_ = cfg_.skip_frames;
}

bool Af::getPhase(PdafData const &data, double &phase, double &conf) const
{
	static const uint8_t DEFAULT_WEIGHTS[PDAF_DATA_ROWS][PDAF_DATA_COLS] = {
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 2, 4, 4, 4, 4, 4, 4, 2, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
	};
	int32_t sum_w = 0;
	int32_t sum_wc = 0;
	int32_t sum_wcp = 0;
	auto ww = useWeights_ ? phaseWeights_ : DEFAULT_WEIGHTS;

	for (unsigned i = 0; i < PDAF_DATA_ROWS; ++i) {
		for (unsigned j = 0; j < PDAF_DATA_COLS; ++j) {
			if (ww[i][j]) {
				uint32_t c = data.conf[i][j];
				if (c >= cfg_.conf_thresh) {
					if (c > cfg_.conf_clip)
						c = cfg_.conf_clip;
					sum_wc += ww[i][j] * c;
					c -= (cfg_.conf_thresh >> 1);
					sum_wcp += ww[i][j] * data.phase[i][j] * (int32_t)c;
				}
				sum_w += ww[i][j];
			}
		}
	}

	if (sum_wc > 0) {
		phase = (double)sum_wcp / (double)sum_wc;
		conf = (double)sum_wc / (double)sum_w;
		return true;
	} else {
		phase = 0.0;
		conf = 0.0;
		return false;
	}
}

void Af::doPDAF(double phase, double conf)
{
	/* Apply gain; scale down when confidence is low or error is small */
	phase *= cfg_.pdaf_gain[speed_];
	phase *= conf / (conf + cfg_.conf_epsilon);
	if (std::abs(phase) < cfg_.pdaf_squelch[speed_]) {
		if (scanState_ == SCAN_PDAF && stepCount_ + 3 < cfg_.pdaf_frames[speed_])
			scanState_ = SCAN_IDLE;
		else
			phase *= std::pow(std::abs(phase) / cfg_.pdaf_squelch[speed_], 2.0);
	}

	/* Apply slew rate limit. Report failure if out of bounds. */
	if (phase < -cfg_.max_slew[speed_]) {
		phase = -cfg_.max_slew[speed_];
		reportState_ = (ftarget_ <= cfg_.focus_min[range_]) ? AfStatus::STATE_FAILED : AfStatus::STATE_SCANNING;
	} else if (phase > cfg_.max_slew[speed_]) {
		phase = cfg_.max_slew[speed_];
		reportState_ = (ftarget_ >= cfg_.focus_max[range_]) ? AfStatus::STATE_FAILED : AfStatus::STATE_SCANNING;
	} else if (scanState_ == SCAN_IDLE) {
		reportState_ = AfStatus::STATE_FOCUSED;
	}

	ftarget_ = fsmooth_ + phase;
}

bool Af::earlyTerminationByPhase(double phase)
{
	if (scanData_.size() > 0 &&
	    scanData_[scanData_.size() - 1].conf >= cfg_.conf_epsilon) {
		double oldFocus = scanData_[scanData_.size() - 1].focus;
		double oldPhase = scanData_[scanData_.size() - 1].phase;

		/*
		 * Check that the gradient is finite and has the expected sign;
		 * Interpolate/extrapolate the lens position for zero phase.
		 * Check that the extrapolation is well-conditioned.
		 */
		if ((ftarget_ - oldFocus) * (phase - oldPhase) >= 0.01) {
			double param = phase / (phase - oldPhase);
			if (-3.0 <= param && param <= 3.5) {
				ftarget_ += param * (oldFocus - ftarget_);
				LOG(RPiAf, Debug) << "ETBP: param=" << param;
				return true;
			}
		}
	}

	return false;
}

double Af::findPeak(unsigned i) const
{
	double f = scanData_[i].focus;

	if (i > 0 && i + 1 < scanData_.size()) {
		double drop_lo = scanData_[i].contrast - scanData_[i - 1].contrast;
		double drop_hi = scanData_[i].contrast - scanData_[i + 1].contrast;
		if (drop_lo < drop_hi) {
			double param = 0.3125 * (1.0 - drop_lo / drop_hi) * (1.6 - drop_lo / drop_hi);
			f += param * (scanData_[i - 1].focus - f);
		} else if (drop_hi < drop_lo) {
			double param = 0.3125 * (1.0 - drop_hi / drop_lo) * (1.6 - drop_hi / drop_lo);
			f += param * (scanData_[i + 1].focus - f);
		}
	}

	LOG(RPiAf, Debug) << "FindPeak: " << f;
	return f;
}

void Af::doScan(double contrast, double phase, double conf)
{
	/* Record lens position, contrast and phase values for the current scan */
	if (scanData_.empty() || contrast > scanMaxContrast_) {
		scanMaxContrast_ = contrast;
		scanMaxIndex_ = scanData_.size();
	}
	if (contrast < scanMinContrast_)
		scanMinContrast_ = contrast;
	scanData_.push_back(ScanRecord{ ftarget_, contrast, phase, conf });

	if (scanState_ == SCAN_COARSE) {
		if (ftarget_ >= cfg_.focus_max[range_] ||
		    contrast < cfg_.contrast_ratio[speed_] * scanMaxContrast_) {
			/*
			 * Finished course scan, or termination based on contrast.
			 * Jump to just after max contrast and start fine scan.
			 */
			ftarget_ = std::min(ftarget_, findPeak(scanMaxIndex_) + 2.0 * cfg_.step_fine[speed_]);
			scanState_ = SCAN_FINE;
			scanData_.clear();
		} else {
			ftarget_ += cfg_.step_coarse[speed_];
		}
	} else { /* SCAN_FINE */
		if (ftarget_ <= cfg_.focus_min[range_] || scanData_.size() >= 5 ||
		    contrast < cfg_.contrast_ratio[speed_] * scanMaxContrast_) {
			/*
			 * Finished fine scan, or termination based on contrast.
			 * Use quadratic peak-finding to find best contrast position.
			 */
			ftarget_ = findPeak(scanMaxIndex_);
			scanState_ = SCAN_SETTLE;
		} else {
			ftarget_ -= cfg_.step_fine[speed_];
		}
	}

	stepCount_ = (ftarget_ == fsmooth_) ? 0 : cfg_.step_frames[speed_];
}

void Af::doAF(double contrast, double phase, double conf)
{
	/* Skip frames at startup and after mode change */
	if (skipCount_ > 0) {
		LOG(RPiAf, Debug) << "SKIP";
		skipCount_--;
		return;
	}

	if (scanState_ == SCAN_PDAF ||
	    (cafEnabled_ && scanState_ == SCAN_IDLE && cfg_.dropout_frames[speed_] > 0)) {
		/*
		 * Use PDAF closed-loop control whenever available, in both CAF
		 * mode and (for a limited number of iterations) when triggered.
		 * If PDAF fails (due to poor contrast, noise or large defocus),
		 * fall back to a CDAF-based scan. To avoid "nuisance" scans,
		 * scan only after a number of frames with low PDAF confidence.
		 */
		if (conf > (dropCount_ ? cfg_.conf_epsilon : 0.0)) {
			if (stepCount_ > 0)
				stepCount_--;
			else
				scanState_ = SCAN_IDLE;
			doPDAF(phase, conf);
			dropCount_ = 0;
		} else if (++dropCount_ == cfg_.dropout_frames[speed_]) {
			startProgrammedScan();
		}
	}

	else if (scanState_ >= SCAN_COARSE && fsmooth_ == ftarget_) {
		/*
		 * Scanning sequence. This means PDAF has become unavailable.
		 * Allow a delay between steps for CDAF FoM statistics to be
		 * updated, and a "settling time" at the end of the sequence.
		 * [A coarse or fine scan can be abandoned if two PDAF samples
		 * allow direct interpolation of the zero-phase lens position.]
		 */
		if (stepCount_ > 0) {
			stepCount_--;
		} else if (scanState_ == SCAN_SETTLE) {
			if (prevContrast_ >= cfg_.contrast_ratio[speed_] * scanMaxContrast_ &&
			    scanMinContrast_ <= cfg_.contrast_ratio[speed_] * scanMaxContrast_)
				reportState_ = AfStatus::STATE_FOCUSED;
			else
				reportState_ = AfStatus::STATE_FAILED;
			scanState_ = SCAN_IDLE;
			scanData_.clear();
		} else if (conf >= cfg_.conf_epsilon && earlyTerminationByPhase(phase)) {
			scanState_ = SCAN_SETTLE;
			stepCount_ = cafEnabled_ ? 0 : cfg_.step_frames[speed_];
		} else {
			doScan(contrast, phase, conf);
		}
	}
}

void Af::updateLensPosition()
{
	/* clamp ftarget_ to range limits, and fsmooth_ to slew rate limit */
	if (isfinite(ftarget_)) {
		ftarget_ = std::clamp(ftarget_,
				      cfg_.focus_min[range_],
				      cfg_.focus_max[range_]);
		if (isfinite(fsmooth_)) {
			fsmooth_ = std::clamp(ftarget_,
					      fsmooth_ - cfg_.max_slew[speed_],
					      fsmooth_ + cfg_.max_slew[speed_]);
		} else {
			fsmooth_ = ftarget_;
			skipCount_ = cfg_.skip_frames;
		}
	}
}

/*
 * PDAF phase data are available in prepare(), but CDAF statistics are not
 * available until process(). We are gambling on the availability of PDAF.
 * To expedite feedback control using PDAF, issue the V4L2 lens control from
 * prepare(). Conversely, during scans, we must allow an extra frame delay
 * between steps, to retrieve CDAF statistics from the previous process()
 * so we can terminate the scan early without having to change our minds.
 */

void Af::prepare(Metadata *image_metadata)
{
	/* Remember the previous lens position before we change it */
	AfStatus status;
	status.lensKnown = isfinite(fsmooth_);
	status.lensEstimate = isfinite(fsmooth_) ? fsmooth_ : 0.0;

	/* Get PDAF from the embedded metadata, and run AF algorithm core */
	PdafData data;
	double phase = 0.0, conf = 0.0;
	double oldFt = ftarget_;
	double oldFs = fsmooth_;
	ScanState oldSs = scanState_;
	uint32_t oldSt = stepCount_;
	if (image_metadata->get("pdaf.data", data) == 0)
		getPhase(data, phase, conf);
	doAF(prevContrast_, phase, conf);
	updateLensPosition();
	LOG(RPiAf, Debug) << std::fixed << std::setprecision(2)
			  << reportState_
			  << " sst" << oldSs << "->" << scanState_
			  << " stp" << oldSt << "->" << stepCount_
			  << " ft" << oldFt << "->" << ftarget_
			  << " fs" << oldFs << "->" << fsmooth_
			  << " cont=" << (int)prevContrast_ << " phase=" << (int)phase << " conf=" << (int)conf;

	/* Report status and produce new lens setting */
	status.state = reportState_;
	status.lensDriven = isfinite(fsmooth_);
	status.lensSetting = isfinite(fsmooth_) ? cfg_.map.eval(fsmooth_) : 0.0;
	image_metadata->set("af.status", status);
}

double Af::getContrast(struct bcm2835_isp_stats_focus const focus_stats[FOCUS_REGIONS]) const
{
	uint32_t tot_w = 0, tot_wc = 0;

	if (useWeights_) {
		for (unsigned i = 0; i < FOCUS_REGIONS; ++i) {
			unsigned w = contrastWeights_[i];
			tot_w += w;
			tot_wc += w * (focus_stats[i].contrast_val[1][1] >> 10);
		}
	}
	if (tot_w == 0) {
		tot_w = 2;
		tot_wc = (focus_stats[5].contrast_val[1][1] >> 10) +
			 (focus_stats[6].contrast_val[1][1] >> 10);
	}

	return (double)tot_wc / (double)tot_w;
}

void Af::process(StatisticsPtr &stats, Metadata *image_metadata)
{
	(void)image_metadata;
	prevContrast_ = getContrast(stats->focus_stats);
}

/* Controls */

void Af::setRange(AfRange r)
{
	LOG(RPiAf, Debug) << "setRange: " << (unsigned)r;
	if ((unsigned)r < AF_NUM_RANGES)
		range_ = r;
}

void Af::setSpeed(AfSpeed s)
{
	LOG(RPiAf, Debug) << "setSpeed: " << (unsigned)s;
	if ((unsigned)s < AF_NUM_SPEEDS) {
		if (scanState_ == SCAN_PDAF && cfg_.pdaf_frames[s] > cfg_.pdaf_frames[speed_])
			stepCount_ += cfg_.pdaf_frames[s] - cfg_.pdaf_frames[speed_];
		speed_ = s;
	}
}

void Af::setMetering(bool mode)
{
	useWeights_ = mode;
}

void Af::setWindows(libcamera::Span<libcamera::Rectangle const> const &wins)
{
	/*
	 * Here we just merge all of the given windows, weighted by area.
	 * If there are more than 15 overlapping windows, overflow can occur.
	 * TODO: A better approach might be to find the phase in each window
	 * and choose either the closest or the highest-confidence one?
	 *
	 * Using mostly "int" arithmetic, because Rectangle has signed x, y
	 */
	int gridY = (int)(sensorSize_.height / PDAF_DATA_ROWS);
	int gridX = (int)(sensorSize_.width / PDAF_DATA_COLS);
	int gridA = gridY * gridX;

	for (int i = 0; i < PDAF_DATA_ROWS; ++i)
		std::fill(phaseWeights_[i], phaseWeights_[i] + PDAF_DATA_COLS, 0);
	std::fill(contrastWeights_, contrastWeights_ + FOCUS_REGIONS, 0);

	for (auto &w : wins) {
		for (int i = 0; i < PDAF_DATA_ROWS; ++i) {
			int y0 = std::max(gridY * i, w.y);
			int y1 = std::min(gridY * (i + 1), w.y + (int)(w.height));
			if (y0 >= y1)
				continue;
			y1 -= y0;
			for (int j = 0; j < PDAF_DATA_COLS; ++j) {
				int x0 = std::max(gridX * j, w.x);
				int x1 = std::min(gridX * (j + 1), w.x + (int)(w.width));
				if (x0 >= x1)
					continue;
				int a = y1 * (x1 - x0);
				a = (16 * a + gridA - 1) / gridA;
				phaseWeights_[i][j] += a;
				contrastWeights_[4 * ((3 * i) / PDAF_DATA_ROWS) + ((4 * j) / PDAF_DATA_COLS)] += a;
			}
		}
	}
}

bool Af::setLensPosition(double dioptres, int *hwpos)
{
	bool changed = false;

	LOG(RPiAf, Debug) << "setLensPosition: " << dioptres;
	if (reportState_ != AfStatus::STATE_SCANNING) {
		changed = !(fsmooth_ == dioptres);
		ftarget_ = dioptres;
		updateLensPosition();
	}

	if (hwpos)
		*hwpos = cfg_.map.eval(fsmooth_);

	return changed;
}

void Af::enableCAF(bool enabled)
{
	LOG(RPiAf, Debug) << "enableCAF: " << enabled;
	if (enabled && !cafEnabled_) {
		if (cfg_.dropout_frames[speed_] > 0) {
			if (!isfinite(ftarget_)) {
				ftarget_ = cfg_.focus_default[range_];
				updateLensPosition();
			}
			scanState_ = SCAN_IDLE;
			scanData_.clear();
			dropCount_ = 0;
			reportState_ = AfStatus::STATE_SCANNING;
		} else {
			startProgrammedScan();
		}
	}
	cafEnabled_ = enabled;
}

void Af::cancelScan()
{
	LOG(RPiAf, Debug) << "cancelScan";
	scanState_ = SCAN_IDLE;
	scanData_.clear();
	reportState_ = AfStatus::STATE_UNKNOWN;
}

void Af::startProgrammedScan()
{
	ftarget_ = cfg_.focus_min[range_];
	updateLensPosition();
	scanState_ = SCAN_COARSE;
	scanMaxContrast_ = 0.0;
	scanMinContrast_ = 1.0e9;
	scanMaxIndex_ = 0;
	scanData_.clear();
	stepCount_ = cfg_.step_frames[speed_];
	reportState_ = AfStatus::STATE_SCANNING;
}

void Af::triggerScan()
{
	LOG(RPiAf, Debug) << "triggerScan";
	if (scanState_ == SCAN_IDLE) {
		if (cfg_.pdaf_frames[speed_] > 0 && cfg_.dropout_frames[speed_] > 0) {
			if (!isfinite(ftarget_)) {
				ftarget_ = cfg_.focus_default[range_];
				updateLensPosition();
			}
			stepCount_ = cfg_.pdaf_frames[speed_];
			scanState_ = SCAN_PDAF;
			dropCount_ = 0;
		} else {
			startProgrammedScan();
		}
		reportState_ = AfStatus::STATE_SCANNING;
	}
}

// Register algorithm with the system.
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Af(controller);
}
static RegisterAlgorithm reg(NAME, &create);
