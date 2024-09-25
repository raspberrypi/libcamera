/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ALSC (auto lens shading correction) control algorithm
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include "../awb_status.h"
#include "alsc.h"

/* Raspberry Pi ALSC (Auto Lens Shading Correction) algorithm. */

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiAlsc)

#define NAME "rpi.alsc"

static const double InsufficientData = -1.0;

Alsc::Alsc(Controller *controller)
	: Algorithm(controller)
{
	asyncAbort_ = asyncStart_ = asyncStarted_ = asyncFinished_ = false;
	asyncThread_ = std::thread(std::bind(&Alsc::asyncFunc, this));
}

Alsc::~Alsc()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		asyncAbort_ = true;
	}
	asyncSignal_.notify_one();
	asyncThread_.join();
}

char const *Alsc::name() const
{
	return NAME;
}

static int generateLut(Array2D<double> &lut, const libcamera::YamlObject &params)
{
	/* These must be signed ints for the co-ordinate calculations below. */
	int X = lut.dimensions().width, Y = lut.dimensions().height;
	double cstrength = params["corner_strength"].get<double>(2.0);
	if (cstrength <= 1.0) {
		LOG(RPiAlsc, Error) << "corner_strength must be > 1.0";
		return -EINVAL;
	}

	double asymmetry = params["asymmetry"].get<double>(1.0);
	if (asymmetry < 0) {
		LOG(RPiAlsc, Error) << "asymmetry must be >= 0";
		return -EINVAL;
	}

	double f1 = cstrength - 1, f2 = 1 + sqrt(cstrength);
	double R2 = X * Y / 4 * (1 + asymmetry * asymmetry);
	int num = 0;
	for (int y = 0; y < Y; y++) {
		for (int x = 0; x < X; x++) {
			double dy = y - Y / 2 + 0.5,
			       dx = (x - X / 2 + 0.5) * asymmetry;
			double r2 = (dx * dx + dy * dy) / R2;
			lut[num++] =
				(f1 * r2 + f2) * (f1 * r2 + f2) /
				(f2 * f2); /* this reproduces the cos^4 rule */
		}
	}
	return 0;
}

static int readLut(Array2D<double> &lut, const libcamera::YamlObject &params)
{
	if (params.size() != lut.size()) {
		LOG(RPiAlsc, Error) << "Invalid number of entries in LSC table";
		return -EINVAL;
	}

	int num = 0;
	for (const auto &p : params.asList()) {
		auto value = p.get<double>();
		if (!value)
			return -EINVAL;
		lut[num++] = *value;
	}

	return 0;
}

static int readCalibrations(std::vector<AlscCalibration> &calibrations,
			    const libcamera::YamlObject &params,
			    std::string const &name, const Size &size)
{
	if (params.contains(name)) {
		double lastCt = 0;
		for (const auto &p : params[name].asList()) {
			auto value = p["ct"].get<double>();
			if (!value)
				return -EINVAL;
			double ct = *value;
			if (ct <= lastCt) {
				LOG(RPiAlsc, Error)
					<< "Entries in " << name << " must be in increasing ct order";
				return -EINVAL;
			}
			AlscCalibration calibration;
			calibration.ct = lastCt = ct;

			const libcamera::YamlObject &table = p["table"];
			if (table.size() != size.width * size.height) {
				LOG(RPiAlsc, Error)
					<< "Incorrect number of values for ct "
					<< ct << " in " << name;
				return -EINVAL;
			}

			int num = 0;
			calibration.table.resize(size);
			for (const auto &elem : table.asList()) {
				value = elem.get<double>();
				if (!value)
					return -EINVAL;
				calibration.table[num++] = *value;
			}

			calibrations.push_back(std::move(calibration));
			LOG(RPiAlsc, Debug)
				<< "Read " << name << " calibration for ct " << ct;
		}
	}
	return 0;
}

int Alsc::read(const libcamera::YamlObject &params)
{
	config_.tableSize = getHardwareConfig().awbRegions;
	config_.framePeriod = params["frame_period"].get<uint16_t>(12);
	config_.startupFrames = params["startup_frames"].get<uint16_t>(10);
	config_.speed = params["speed"].get<double>(0.05);
	double sigma = params["sigma"].get<double>(0.01);
	config_.sigmaCr = params["sigma_Cr"].get<double>(sigma);
	config_.sigmaCb = params["sigma_Cb"].get<double>(sigma);
	config_.minCount = params["min_count"].get<double>(10.0);
	config_.minG = params["min_G"].get<uint16_t>(50);
	config_.omega = params["omega"].get<double>(1.3);
	config_.nIter = params["n_iter"].get<uint32_t>(config_.tableSize.width + config_.tableSize.height);
	config_.luminanceStrength =
		params["luminance_strength"].get<double>(1.0);

	config_.luminanceLut.resize(config_.tableSize, 1.0);
	int ret = 0;

	if (params.contains("corner_strength"))
		ret = generateLut(config_.luminanceLut, params);
	else if (params.contains("luminance_lut"))
		ret = readLut(config_.luminanceLut, params["luminance_lut"]);
	else
		LOG(RPiAlsc, Warning)
			<< "no luminance table - assume unity everywhere";
	if (ret)
		return ret;

	ret = readCalibrations(config_.calibrationsCr, params, "calibrations_Cr",
			       config_.tableSize);
	if (ret)
		return ret;
	ret = readCalibrations(config_.calibrationsCb, params, "calibrations_Cb",
			       config_.tableSize);
	if (ret)
		return ret;

	config_.defaultCt = params["default_ct"].get<double>(4500.0);
	config_.threshold = params["threshold"].get<double>(1e-3);
	config_.lambdaBound = params["lambda_bound"].get<double>(0.05);

	return 0;
}

static double getCt(Metadata *metadata, double defaultCt);
static void getCalTable(double ct, std::vector<AlscCalibration> const &calibrations,
			Array2D<double> &calTable);
static void resampleCalTable(const Array2D<double> &calTableIn, CameraMode const &cameraMode,
			     Array2D<double> &calTableOut);
static void compensateLambdasForCal(const Array2D<double> &calTable,
				    const Array2D<double> &oldLambdas,
				    Array2D<double> &newLambdas);
static void addLuminanceToTables(std::array<Array2D<double>, 3> &results,
				 const Array2D<double> &lambdaR, double lambdaG,
				 const Array2D<double> &lambdaB,
				 const Array2D<double> &luminanceLut,
				 double luminanceStrength);

void Alsc::initialise()
{
	frameCount2_ = frameCount_ = framePhase_ = 0;
	firstTime_ = true;
	ct_ = config_.defaultCt;

	const size_t XY = config_.tableSize.width * config_.tableSize.height;

	for (auto &r : syncResults_)
		r.resize(config_.tableSize);
	for (auto &r : prevSyncResults_)
		r.resize(config_.tableSize);
	for (auto &r : asyncResults_)
		r.resize(config_.tableSize);

	luminanceTable_.resize(config_.tableSize);
	asyncLambdaR_.resize(config_.tableSize);
	asyncLambdaB_.resize(config_.tableSize);
	/* The lambdas are initialised in the SwitchMode. */
	lambdaR_.resize(config_.tableSize);
	lambdaB_.resize(config_.tableSize);

	/* Temporaries for the computations, but sensible to allocate this up-front! */
	for (auto &c : tmpC_)
		c.resize(config_.tableSize);
	for (auto &m : tmpM_)
		m.resize(XY);
}

void Alsc::waitForAysncThread()
{
	if (asyncStarted_) {
		asyncStarted_ = false;
		std::unique_lock<std::mutex> lock(mutex_);
		syncSignal_.wait(lock, [&] {
			return asyncFinished_;
		});
		asyncFinished_ = false;
	}
}

static bool compareModes(CameraMode const &cm0, CameraMode const &cm1)
{
	/*
	 * Return true if the modes crop from the sensor significantly differently,
	 * or if the user transform has changed.
	 */
	if (cm0.transform != cm1.transform)
		return true;
	int leftDiff = std::abs(cm0.cropX - cm1.cropX);
	int topDiff = std::abs(cm0.cropY - cm1.cropY);
	int rightDiff = std::abs(cm0.cropX + cm0.scaleX * cm0.width -
				 cm1.cropX - cm1.scaleX * cm1.width);
	int bottomDiff = std::abs(cm0.cropY + cm0.scaleY * cm0.height -
				  cm1.cropY - cm1.scaleY * cm1.height);
	/*
	 * These thresholds are a rather arbitrary amount chosen to trigger
	 * when carrying on with the previously calculated tables might be
	 * worse than regenerating them (but without the adaptive algorithm).
	 */
	int thresholdX = cm0.sensorWidth >> 4;
	int thresholdY = cm0.sensorHeight >> 4;
	return leftDiff > thresholdX || rightDiff > thresholdX ||
	       topDiff > thresholdY || bottomDiff > thresholdY;
}

void Alsc::switchMode(CameraMode const &cameraMode,
		      [[maybe_unused]] Metadata *metadata)
{
	/*
	 * We're going to start over with the tables if there's any "significant"
	 * change.
	 */
	bool resetTables = firstTime_ || compareModes(cameraMode_, cameraMode);

	/* Believe the colour temperature from the AWB, if there is one. */
	ct_ = getCt(metadata, ct_);

	/* Ensure the other thread isn't running while we do this. */
	waitForAysncThread();

	cameraMode_ = cameraMode;

	/*
	 * We must resample the luminance table like we do the others, but it's
	 * fixed so we can simply do it up front here.
	 */
	resampleCalTable(config_.luminanceLut, cameraMode_, luminanceTable_);

	if (resetTables) {
		/*
		 * Upon every "table reset", arrange for something sensible to be
		 * generated. Construct the tables for the previous recorded colour
		 * temperature. In order to start over from scratch we initialise
		 * the lambdas, but the rest of this code then echoes the code in
		 * doAlsc, without the adaptive algorithm.
		 */
		std::fill(lambdaR_.begin(), lambdaR_.end(), 1.0);
		std::fill(lambdaB_.begin(), lambdaB_.end(), 1.0);
		Array2D<double> &calTableR = tmpC_[0], &calTableB = tmpC_[1], &calTableTmp = tmpC_[2];
		getCalTable(ct_, config_.calibrationsCr, calTableTmp);
		resampleCalTable(calTableTmp, cameraMode_, calTableR);
		getCalTable(ct_, config_.calibrationsCb, calTableTmp);
		resampleCalTable(calTableTmp, cameraMode_, calTableB);
		compensateLambdasForCal(calTableR, lambdaR_, asyncLambdaR_);
		compensateLambdasForCal(calTableB, lambdaB_, asyncLambdaB_);
		addLuminanceToTables(syncResults_, asyncLambdaR_, 1.0, asyncLambdaB_,
				     luminanceTable_, config_.luminanceStrength);
		prevSyncResults_ = syncResults_;
		framePhase_ = config_.framePeriod; /* run the algo again asap */
		firstTime_ = false;
	}
}

void Alsc::fetchAsyncResults()
{
	LOG(RPiAlsc, Debug) << "Fetch ALSC results";
	asyncFinished_ = false;
	asyncStarted_ = false;
	syncResults_ = asyncResults_;
}

double getCt(Metadata *metadata, double defaultCt)
{
	AwbStatus awbStatus;
	awbStatus.temperatureK = defaultCt; /* in case nothing found */
	if (metadata->get("awb.status", awbStatus) != 0)
		LOG(RPiAlsc, Debug) << "no AWB results found, using "
				    << awbStatus.temperatureK;
	else
		LOG(RPiAlsc, Debug) << "AWB results found, using "
				    << awbStatus.temperatureK;
	return awbStatus.temperatureK;
}

static void copyStats(RgbyRegions &regions, StatisticsPtr &stats,
		      std::array<Array2D<double>, 3> &prevSyncResults)
{
	if (!regions.numRegions())
		regions.init(stats->awbRegions.size());

	const std::vector<double> &rTable = prevSyncResults[0].data(); //status.r;
	const std::vector<double> &gTable = prevSyncResults[1].data(); //status.g;
	const std::vector<double> &bTable = prevSyncResults[2].data(); //status.b;
	for (unsigned int i = 0; i < stats->awbRegions.numRegions(); i++) {
		auto r = stats->awbRegions.get(i);
		if (stats->colourStatsPos == Statistics::ColourStatsPos::PostLsc) {
			r.val.rSum = static_cast<uint64_t>(r.val.rSum / rTable[i]);
			r.val.gSum = static_cast<uint64_t>(r.val.gSum / gTable[i]);
			r.val.bSum = static_cast<uint64_t>(r.val.bSum / bTable[i]);
		}
		regions.set(i, r);
	}
}

void Alsc::restartAsync(StatisticsPtr &stats, Metadata *imageMetadata)
{
	LOG(RPiAlsc, Debug) << "Starting ALSC calculation";
	/*
	 * Get the current colour temperature. It's all we need from the
	 * metadata. Default to the last CT value (which could be the default).
	 */
	ct_ = getCt(imageMetadata, ct_);
	/*
	 * We have to copy the statistics here, dividing out our best guess of
	 * the LSC table that the pipeline applied to them which we get from
	 * prevSyncResults_.
	 */
	copyStats(statistics_, stats, prevSyncResults_);
	framePhase_ = 0;
	asyncStarted_ = true;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		asyncStart_ = true;
	}
	asyncSignal_.notify_one();
}

void Alsc::prepare(Metadata *imageMetadata)
{
	/*
	 * Count frames since we started, and since we last poked the async
	 * thread.
	 */
	if (frameCount_ < (int)config_.startupFrames)
		frameCount_++;
	double speed = frameCount_ < (int)config_.startupFrames
			       ? 1.0
			       : config_.speed;
	LOG(RPiAlsc, Debug)
		<< "frame count " << frameCount_ << " speed " << speed;
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (asyncStarted_ && asyncFinished_)
			fetchAsyncResults();
	}
	/* Apply IIR filter to results and program into the pipeline. */
	for (unsigned int j = 0; j < syncResults_.size(); j++) {
		for (unsigned int i = 0; i < syncResults_[j].size(); i++)
			prevSyncResults_[j][i] = speed * syncResults_[j][i] + (1.0 - speed) * prevSyncResults_[j][i];
	}
	/* Put output values into status metadata. */
	AlscStatus status;
	status.r = prevSyncResults_[0].data();
	status.g = prevSyncResults_[1].data();
	status.b = prevSyncResults_[2].data();
	imageMetadata->set("alsc.status", status);
	/*
	 * Put the results in the global metadata as well. This will be used by
	 * AWB to factor in the colour shading correction.
	 */
	getGlobalMetadata().set("alsc.status", status);
}

void Alsc::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	/*
	 * Count frames since we started, and since we last poked the async
	 * thread.
	 */
	if (framePhase_ < (int)config_.framePeriod)
		framePhase_++;
	if (frameCount2_ < (int)config_.startupFrames)
		frameCount2_++;
	LOG(RPiAlsc, Debug) << "frame_phase " << framePhase_;
	if (framePhase_ >= (int)config_.framePeriod ||
	    frameCount2_ < (int)config_.startupFrames) {
		if (asyncStarted_ == false)
			restartAsync(stats, imageMetadata);
	}
}

void Alsc::asyncFunc()
{
	while (true) {
		{
			std::unique_lock<std::mutex> lock(mutex_);
			asyncSignal_.wait(lock, [&] {
				return asyncStart_ || asyncAbort_;
			});
			asyncStart_ = false;
			if (asyncAbort_)
				break;
		}
		doAlsc();
		{
			std::lock_guard<std::mutex> lock(mutex_);
			asyncFinished_ = true;
		}
		syncSignal_.notify_one();
	}
}

void getCalTable(double ct, std::vector<AlscCalibration> const &calibrations,
		 Array2D<double> &calTable)
{
	if (calibrations.empty()) {
		std::fill(calTable.begin(), calTable.end(), 1.0);
		LOG(RPiAlsc, Debug) << "no calibrations found";
	} else if (ct <= calibrations.front().ct) {
		calTable = calibrations.front().table;
		LOG(RPiAlsc, Debug) << "using calibration for "
				    << calibrations.front().ct;
	} else if (ct >= calibrations.back().ct) {
		calTable = calibrations.back().table;
		LOG(RPiAlsc, Debug) << "using calibration for "
				    << calibrations.back().ct;
	} else {
		int idx = 0;
		while (ct > calibrations[idx + 1].ct)
			idx++;
		double ct0 = calibrations[idx].ct, ct1 = calibrations[idx + 1].ct;
		LOG(RPiAlsc, Debug)
			<< "ct is " << ct << ", interpolating between "
			<< ct0 << " and " << ct1;
		for (unsigned int i = 0; i < calTable.size(); i++)
			calTable[i] =
				(calibrations[idx].table[i] * (ct1 - ct) +
				 calibrations[idx + 1].table[i] * (ct - ct0)) /
				(ct1 - ct0);
	}
}

void resampleCalTable(const Array2D<double> &calTableIn,
		      CameraMode const &cameraMode,
		      Array2D<double> &calTableOut)
{
	int X = calTableIn.dimensions().width;
	int Y = calTableIn.dimensions().height;

	/*
	 * Precalculate and cache the x sampling locations and phases to save
	 * recomputing them on every row.
	 */
	std::vector<int> xLo(X);
	std::vector<int> xHi(X);
	std::vector<double> xf(X);
	double scaleX = cameraMode.sensorWidth /
			(cameraMode.width * cameraMode.scaleX);
	double xOff = cameraMode.cropX / (double)cameraMode.sensorWidth;
	double x = .5 / scaleX + xOff * X - .5;
	double xInc = 1 / scaleX;
	for (int i = 0; i < X; i++, x += xInc) {
		xLo[i] = floor(x);
		xf[i] = x - xLo[i];
		xHi[i] = std::min(xLo[i] + 1, X - 1);
		xLo[i] = std::max(xLo[i], 0);
		if (!!(cameraMode.transform & libcamera::Transform::HFlip)) {
			xLo[i] = X - 1 - xLo[i];
			xHi[i] = X - 1 - xHi[i];
		}
	}
	/* Now march over the output table generating the new values. */
	double scaleY = cameraMode.sensorHeight /
			(cameraMode.height * cameraMode.scaleY);
	double yOff = cameraMode.cropY / (double)cameraMode.sensorHeight;
	double y = .5 / scaleY + yOff * Y - .5;
	double yInc = 1 / scaleY;
	for (int j = 0; j < Y; j++, y += yInc) {
		int yLo = floor(y);
		double yf = y - yLo;
		int yHi = std::min(yLo + 1, Y - 1);
		yLo = std::max(yLo, 0);
		if (!!(cameraMode.transform & libcamera::Transform::VFlip)) {
			yLo = Y - 1 - yLo;
			yHi = Y - 1 - yHi;
		}
		double const *rowAbove = calTableIn.ptr() + X * yLo;
		double const *rowBelow = calTableIn.ptr() + X * yHi;
		double *out = calTableOut.ptr() + X * j;
		for (int i = 0; i < X; i++) {
			double above = rowAbove[xLo[i]] * (1 - xf[i]) +
				       rowAbove[xHi[i]] * xf[i];
			double below = rowBelow[xLo[i]] * (1 - xf[i]) +
				       rowBelow[xHi[i]] * xf[i];
			*(out++) = above * (1 - yf) + below * yf;
		}
	}
}

/* Calculate chrominance statistics (R/G and B/G) for each region. */
static void calculateCrCb(const RgbyRegions &awbRegion, Array2D<double> &cr,
			  Array2D<double> &cb, uint32_t minCount, uint16_t minG)
{
	for (unsigned int i = 0; i < cr.size(); i++) {
		auto s = awbRegion.get(i);

		/* Do not return unreliable, or zero, colour ratio statistics. */
		if (s.counted <= minCount || s.val.gSum / s.counted <= minG ||
		    s.val.rSum / s.counted <= minG || s.val.bSum / s.counted <= minG) {
			cr[i] = cb[i] = InsufficientData;
			continue;
		}

		cr[i] = s.val.rSum / (double)s.val.gSum;
		cb[i] = s.val.bSum / (double)s.val.gSum;
	}
}

static void applyCalTable(const Array2D<double> &calTable, Array2D<double> &C)
{
	for (unsigned int i = 0; i < C.size(); i++)
		if (C[i] != InsufficientData)
			C[i] *= calTable[i];
}

void compensateLambdasForCal(const Array2D<double> &calTable,
			     const Array2D<double> &oldLambdas,
			     Array2D<double> &newLambdas)
{
	double minNewLambda = std::numeric_limits<double>::max();
	for (unsigned int i = 0; i < newLambdas.size(); i++) {
		newLambdas[i] = oldLambdas[i] * calTable[i];
		minNewLambda = std::min(minNewLambda, newLambdas[i]);
	}
	for (unsigned int i = 0; i < newLambdas.size(); i++)
		newLambdas[i] /= minNewLambda;
}

[[maybe_unused]] static void printCalTable(const Array2D<double> &C)
{
	const Size &size = C.dimensions();
	printf("table: [\n");
	for (unsigned int j = 0; j < size.height; j++) {
		for (unsigned int i = 0; i < size.width; i++) {
			printf("%5.3f", 1.0 / C[j * size.width + i]);
			if (i != size.width - 1 || j != size.height - 1)
				printf(",");
		}
		printf("\n");
	}
	printf("]\n");
}

/*
 * Compute weight out of 1.0 which reflects how similar we wish to make the
 * colours of these two regions.
 */
static double computeWeight(double Ci, double Cj, double sigma)
{
	if (Ci == InsufficientData || Cj == InsufficientData)
		return 0;
	double diff = (Ci - Cj) / sigma;
	return exp(-diff * diff / 2);
}

/* Compute all weights. */
static void computeW(const Array2D<double> &C, double sigma,
		     SparseArray<double> &W)
{
	size_t XY = C.size();
	size_t X = C.dimensions().width;

	for (unsigned int i = 0; i < XY; i++) {
		/* Start with neighbour above and go clockwise. */
		W[i][0] = i >= X ? computeWeight(C[i], C[i - X], sigma) : 0;
		W[i][1] = i % X < X - 1 ? computeWeight(C[i], C[i + 1], sigma) : 0;
		W[i][2] = i < XY - X ? computeWeight(C[i], C[i + X], sigma) : 0;
		W[i][3] = i % X ? computeWeight(C[i], C[i - 1], sigma) : 0;
	}
}

/* Compute M, the large but sparse matrix such that M * lambdas = 0. */
static void constructM(const Array2D<double> &C,
		       const SparseArray<double> &W,
		       SparseArray<double> &M)
{
	size_t XY = C.size();
	size_t X = C.dimensions().width;

	double epsilon = 0.001;
	for (unsigned int i = 0; i < XY; i++) {
		/*
		 * Note how, if C[i] == INSUFFICIENT_DATA, the weights will all
		 * be zero so the equation is still set up correctly.
		 */
		int m = !!(i >= X) + !!(i % X < X - 1) + !!(i < XY - X) +
			!!(i % X); /* total number of neighbours */
		/* we'll divide the diagonal out straight away */
		double diagonal = (epsilon + W[i][0] + W[i][1] + W[i][2] + W[i][3]) * C[i];
		M[i][0] = i >= X ? (W[i][0] * C[i - X] + epsilon / m * C[i]) / diagonal : 0;
		M[i][1] = i % X < X - 1 ? (W[i][1] * C[i + 1] + epsilon / m * C[i]) / diagonal : 0;
		M[i][2] = i < XY - X ? (W[i][2] * C[i + X] + epsilon / m * C[i]) / diagonal : 0;
		M[i][3] = i % X ? (W[i][3] * C[i - 1] + epsilon / m * C[i]) / diagonal : 0;
	}
}

/*
 * In the compute_lambda_ functions, note that the matrix coefficients for the
 * left/right neighbours are zero down the left/right edges, so we don't need
 * need to test the i value to exclude them.
 */
static double computeLambdaBottom(int i, const SparseArray<double> &M,
				  Array2D<double> &lambda)
{
	return M[i][1] * lambda[i + 1] + M[i][2] * lambda[i + lambda.dimensions().width] +
	       M[i][3] * lambda[i - 1];
}
static double computeLambdaBottomStart(int i, const SparseArray<double> &M,
				       Array2D<double> &lambda)
{
	return M[i][1] * lambda[i + 1] + M[i][2] * lambda[i + lambda.dimensions().width];
}
static double computeLambdaInterior(int i, const SparseArray<double> &M,
				    Array2D<double> &lambda)
{
	return M[i][0] * lambda[i - lambda.dimensions().width] + M[i][1] * lambda[i + 1] +
	       M[i][2] * lambda[i + lambda.dimensions().width] + M[i][3] * lambda[i - 1];
}
static double computeLambdaTop(int i, const SparseArray<double> &M,
			       Array2D<double> &lambda)
{
	return M[i][0] * lambda[i - lambda.dimensions().width] + M[i][1] * lambda[i + 1] +
	       M[i][3] * lambda[i - 1];
}
static double computeLambdaTopEnd(int i, const SparseArray<double> &M,
				  Array2D<double> &lambda)
{
	return M[i][0] * lambda[i - lambda.dimensions().width] + M[i][3] * lambda[i - 1];
}

/* Gauss-Seidel iteration with over-relaxation. */
static double gaussSeidel2Sor(const SparseArray<double> &M, double omega,
			      Array2D<double> &lambda, double lambdaBound)
{
	int XY = lambda.size();
	int X = lambda.dimensions().width;
	const double min = 1 - lambdaBound, max = 1 + lambdaBound;
	Array2D<double> oldLambda = lambda;
	int i;
	lambda[0] = computeLambdaBottomStart(0, M, lambda);
	lambda[0] = std::clamp(lambda[0], min, max);
	for (i = 1; i < X; i++) {
		lambda[i] = computeLambdaBottom(i, M, lambda);
		lambda[i] = std::clamp(lambda[i], min, max);
	}
	for (; i < XY - X; i++) {
		lambda[i] = computeLambdaInterior(i, M, lambda);
		lambda[i] = std::clamp(lambda[i], min, max);
	}
	for (; i < XY - 1; i++) {
		lambda[i] = computeLambdaTop(i, M, lambda);
		lambda[i] = std::clamp(lambda[i], min, max);
	}
	lambda[i] = computeLambdaTopEnd(i, M, lambda);
	lambda[i] = std::clamp(lambda[i], min, max);
	/*
	 * Also solve the system from bottom to top, to help spread the updates
	 * better.
	 */
	lambda[i] = computeLambdaTopEnd(i, M, lambda);
	lambda[i] = std::clamp(lambda[i], min, max);
	for (i = XY - 2; i >= XY - X; i--) {
		lambda[i] = computeLambdaTop(i, M, lambda);
		lambda[i] = std::clamp(lambda[i], min, max);
	}
	for (; i >= X; i--) {
		lambda[i] = computeLambdaInterior(i, M, lambda);
		lambda[i] = std::clamp(lambda[i], min, max);
	}
	for (; i >= 1; i--) {
		lambda[i] = computeLambdaBottom(i, M, lambda);
		lambda[i] = std::clamp(lambda[i], min, max);
	}
	lambda[0] = computeLambdaBottomStart(0, M, lambda);
	lambda[0] = std::clamp(lambda[0], min, max);
	double maxDiff = 0;
	for (i = 0; i < XY; i++) {
		lambda[i] = oldLambda[i] + (lambda[i] - oldLambda[i]) * omega;
		if (std::abs(lambda[i] - oldLambda[i]) > std::abs(maxDiff))
			maxDiff = lambda[i] - oldLambda[i];
	}
	return maxDiff;
}

/* Normalise the values so that the smallest value is 1. */
static void normalise(Array2D<double> &results)
{
	double minval = *std::min_element(results.begin(), results.end());
	std::for_each(results.begin(), results.end(),
		      [minval](double val) { return val / minval; });
}

/* Rescale the values so that the average value is 1. */
static void reaverage(Array2D<double> &data)
{
	double sum = std::accumulate(data.begin(), data.end(), 0.0);
	double ratio = 1 / (sum / data.size());
	std::for_each(data.begin(), data.end(),
		      [ratio](double val) { return val * ratio; });
}

static void runMatrixIterations(const Array2D<double> &C,
				Array2D<double> &lambda,
				const SparseArray<double> &W,
				SparseArray<double> &M, double omega,
				unsigned int nIter, double threshold, double lambdaBound)
{
	constructM(C, W, M);
	double lastMaxDiff = std::numeric_limits<double>::max();
	for (unsigned int i = 0; i < nIter; i++) {
		double maxDiff = std::abs(gaussSeidel2Sor(M, omega, lambda, lambdaBound));
		if (maxDiff < threshold) {
			LOG(RPiAlsc, Debug)
				<< "Stop after " << i + 1 << " iterations";
			break;
		}
		/*
		 * this happens very occasionally (so make a note), though
		 * doesn't seem to matter
		 */
		if (maxDiff > lastMaxDiff)
			LOG(RPiAlsc, Debug)
				<< "Iteration " << i << ": maxDiff gone up "
				<< lastMaxDiff << " to " << maxDiff;
		lastMaxDiff = maxDiff;
	}
	/* We're going to normalise the lambdas so the total average is 1. */
	reaverage(lambda);
}

static void addLuminanceRb(Array2D<double> &result, const Array2D<double> &lambda,
			   const Array2D<double> &luminanceLut,
			   double luminanceStrength)
{
	for (unsigned int i = 0; i < result.size(); i++)
		result[i] = lambda[i] * ((luminanceLut[i] - 1) * luminanceStrength + 1);
}

static void addLuminanceG(Array2D<double> &result, double lambda,
			  const Array2D<double> &luminanceLut,
			  double luminanceStrength)
{
	for (unsigned int i = 0; i < result.size(); i++)
		result[i] = lambda * ((luminanceLut[i] - 1) * luminanceStrength + 1);
}

void addLuminanceToTables(std::array<Array2D<double>, 3> &results,
			  const Array2D<double> &lambdaR,
			  double lambdaG, const Array2D<double> &lambdaB,
			  const Array2D<double> &luminanceLut,
			  double luminanceStrength)
{
	addLuminanceRb(results[0], lambdaR, luminanceLut, luminanceStrength);
	addLuminanceG(results[1], lambdaG, luminanceLut, luminanceStrength);
	addLuminanceRb(results[2], lambdaB, luminanceLut, luminanceStrength);
	for (auto &r : results)
		normalise(r);
}

void Alsc::doAlsc()
{
	Array2D<double> &cr = tmpC_[0], &cb = tmpC_[1], &calTableR = tmpC_[2],
			&calTableB = tmpC_[3], &calTableTmp = tmpC_[4];
	SparseArray<double> &wr = tmpM_[0], &wb = tmpM_[1], &M = tmpM_[2];

	/*
	 * Calculate our R/B ("Cr"/"Cb") colour statistics, and assess which are
	 * usable.
	 */
	calculateCrCb(statistics_, cr, cb, config_.minCount, config_.minG);
	/*
	 * Fetch the new calibrations (if any) for this CT. Resample them in
	 * case the camera mode is not full-frame.
	 */
	getCalTable(ct_, config_.calibrationsCr, calTableTmp);
	resampleCalTable(calTableTmp, cameraMode_, calTableR);
	getCalTable(ct_, config_.calibrationsCb, calTableTmp);
	resampleCalTable(calTableTmp, cameraMode_, calTableB);
	/*
	 * You could print out the cal tables for this image here, if you're
	 * tuning the algorithm...
	 * Apply any calibration to the statistics, so the adaptive algorithm
	 * makes only the extra adjustments.
	 */
	applyCalTable(calTableR, cr);
	applyCalTable(calTableB, cb);
	/* Compute weights between zones. */
	computeW(cr, config_.sigmaCr, wr);
	computeW(cb, config_.sigmaCb, wb);
	/* Run Gauss-Seidel iterations over the resulting matrix, for R and B. */
	runMatrixIterations(cr, lambdaR_, wr, M, config_.omega, config_.nIter,
			    config_.threshold, config_.lambdaBound);
	runMatrixIterations(cb, lambdaB_, wb, M, config_.omega, config_.nIter,
			    config_.threshold, config_.lambdaBound);
	/*
	 * Fold the calibrated gains into our final lambda values. (Note that on
	 * the next run, we re-start with the lambda values that don't have the
	 * calibration gains included.)
	 */
	compensateLambdasForCal(calTableR, lambdaR_, asyncLambdaR_);
	compensateLambdasForCal(calTableB, lambdaB_, asyncLambdaB_);
	/* Fold in the luminance table at the appropriate strength. */
	addLuminanceToTables(asyncResults_, asyncLambdaR_, 1.0,
			     asyncLambdaB_, luminanceTable_,
			     config_.luminanceStrength);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Alsc(controller);
}
static RegisterAlgorithm reg(NAME, &create);
