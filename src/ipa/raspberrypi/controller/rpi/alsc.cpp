/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * alsc.cpp - ALSC (auto lens shading correction) control algorithm
 */
#include <math.h>

#include "../awb_status.h"
#include "alsc.hpp"

// Raspberry Pi ALSC (Auto Lens Shading Correction) algorithm.

using namespace RPi;

#define NAME "rpi.alsc"

static const int X = ALSC_CELLS_X;
static const int Y = ALSC_CELLS_Y;
static const int XY = X * Y;
static const double INSUFFICIENT_DATA = -1.0;

Alsc::Alsc(Controller *controller)
	: Algorithm(controller)
{
	async_abort_ = async_start_ = async_started_ = async_finished_ = false;
	async_thread_ = std::thread(std::bind(&Alsc::asyncFunc, this));
}

Alsc::~Alsc()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		async_abort_ = true;
		async_signal_.notify_one();
	}
	async_thread_.join();
}

char const *Alsc::Name() const
{
	return NAME;
}

static void generate_lut(double *lut, boost::property_tree::ptree const &params)
{
	double cstrength = params.get<double>("corner_strength", 2.0);
	if (cstrength <= 1.0)
		throw std::runtime_error("Alsc: corner_strength must be > 1.0");
	double asymmetry = params.get<double>("asymmetry", 1.0);
	if (asymmetry < 0)
		throw std::runtime_error("Alsc: asymmetry must be >= 0");
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
				(f2 * f2); // this reproduces the cos^4 rule
		}
	}
}

static void read_lut(double *lut, boost::property_tree::ptree const &params)
{
	int num = 0;
	const int max_num = XY;
	for (auto &p : params) {
		if (num == max_num)
			throw std::runtime_error(
				"Alsc: too many entries in LSC table");
		lut[num++] = p.second.get_value<double>();
	}
	if (num < max_num)
		throw std::runtime_error("Alsc: too few entries in LSC table");
}

static void read_calibrations(std::vector<AlscCalibration> &calibrations,
			      boost::property_tree::ptree const &params,
			      std::string const &name)
{
	if (params.get_child_optional(name)) {
		double last_ct = 0;
		for (auto &p : params.get_child(name)) {
			double ct = p.second.get<double>("ct");
			if (ct <= last_ct)
				throw std::runtime_error(
					"Alsc: entries in " + name +
					" must be in increasing ct order");
			AlscCalibration calibration;
			calibration.ct = last_ct = ct;
			boost::property_tree::ptree const &table =
				p.second.get_child("table");
			int num = 0;
			for (auto it = table.begin(); it != table.end(); it++) {
				if (num == XY)
					throw std::runtime_error(
						"Alsc: too many values for ct " +
						std::to_string(ct) + " in " +
						name);
				calibration.table[num++] =
					it->second.get_value<double>();
			}
			if (num != XY)
				throw std::runtime_error(
					"Alsc: too few values for ct " +
					std::to_string(ct) + " in " + name);
			calibrations.push_back(calibration);
			RPI_LOG("Read " << name << " calibration for ct "
					<< ct);
		}
	}
}

void Alsc::Read(boost::property_tree::ptree const &params)
{
	RPI_LOG("Alsc");
	config_.frame_period = params.get<uint16_t>("frame_period", 12);
	config_.startup_frames = params.get<uint16_t>("startup_frames", 10);
	config_.speed = params.get<double>("speed", 0.05);
	double sigma = params.get<double>("sigma", 0.01);
	config_.sigma_Cr = params.get<double>("sigma_Cr", sigma);
	config_.sigma_Cb = params.get<double>("sigma_Cb", sigma);
	config_.min_count = params.get<double>("min_count", 10.0);
	config_.min_G = params.get<uint16_t>("min_G", 50);
	config_.omega = params.get<double>("omega", 1.3);
	config_.n_iter = params.get<uint32_t>("n_iter", X + Y);
	config_.luminance_strength =
		params.get<double>("luminance_strength", 1.0);
	for (int i = 0; i < XY; i++)
		config_.luminance_lut[i] = 1.0;
	if (params.get_child_optional("corner_strength"))
		generate_lut(config_.luminance_lut, params);
	else if (params.get_child_optional("luminance_lut"))
		read_lut(config_.luminance_lut,
			 params.get_child("luminance_lut"));
	else
		RPI_WARN("Alsc: no luminance table - assume unity everywhere");
	read_calibrations(config_.calibrations_Cr, params, "calibrations_Cr");
	read_calibrations(config_.calibrations_Cb, params, "calibrations_Cb");
	config_.default_ct = params.get<double>("default_ct", 4500.0);
	config_.threshold = params.get<double>("threshold", 1e-3);
}

static void get_cal_table(double ct,
			  std::vector<AlscCalibration> const &calibrations,
			  double cal_table[XY]);
static void resample_cal_table(double const cal_table_in[XY],
			       CameraMode const &camera_mode,
			       double cal_table_out[XY]);
static void compensate_lambdas_for_cal(double const cal_table[XY],
				       double const old_lambdas[XY],
				       double new_lambdas[XY]);
static void add_luminance_to_tables(double results[3][Y][X],
				    double const lambda_r[XY], double lambda_g,
				    double const lambda_b[XY],
				    double const luminance_lut[XY],
				    double luminance_strength);

void Alsc::Initialise()
{
	RPI_LOG("Alsc");
	frame_count2_ = frame_count_ = frame_phase_ = 0;
	first_time_ = true;
	// Initialise the lambdas. Each call to Process then restarts from the
	// previous results.  Also initialise the previous frame tables to the
	// same harmless values.
	for (int i = 0; i < XY; i++)
		lambda_r_[i] = lambda_b_[i] = 1.0;
}

void Alsc::SwitchMode(CameraMode const &camera_mode)
{
	// There's a bit of a question what we should do if the "crop" of the
	// camera mode has changed.  Any calculation currently in flight would
	// not be useful to the new mode, so arguably we should abort it, and
	// generate a new table (like the "first_time" code already here).  When
	// the crop doesn't change, we can presumably just leave things
	// alone. For now, I think we'll just wait and see. When the crop does
	// change, any effects should be transient, and if they're not transient
	// enough, we'll revisit the question then.
	camera_mode_ = camera_mode;
	if (first_time_) {
		// On the first time, arrange for something sensible in the
		// initial tables. Construct the tables for some default colour
		// temperature. This echoes the code in doAlsc, without the
		// adaptive algorithm.
		double cal_table_r[XY], cal_table_b[XY], cal_table_tmp[XY];
		get_cal_table(4000, config_.calibrations_Cr, cal_table_tmp);
		resample_cal_table(cal_table_tmp, camera_mode_, cal_table_r);
		get_cal_table(4000, config_.calibrations_Cb, cal_table_tmp);
		resample_cal_table(cal_table_tmp, camera_mode_, cal_table_b);
		compensate_lambdas_for_cal(cal_table_r, lambda_r_,
					   async_lambda_r_);
		compensate_lambdas_for_cal(cal_table_b, lambda_b_,
					   async_lambda_b_);
		add_luminance_to_tables(sync_results_, async_lambda_r_, 1.0,
					async_lambda_b_, config_.luminance_lut,
					config_.luminance_strength);
		memcpy(prev_sync_results_, sync_results_,
		       sizeof(prev_sync_results_));
		first_time_ = false;
	}
}

void Alsc::fetchAsyncResults()
{
	RPI_LOG("Fetch ALSC results");
	async_finished_ = false;
	async_started_ = false;
	memcpy(sync_results_, async_results_, sizeof(sync_results_));
}

static double get_ct(Metadata *metadata, double default_ct)
{
	AwbStatus awb_status;
	awb_status.temperature_K = default_ct; // in case nothing found
	if (metadata->Get("awb.status", awb_status) != 0)
		RPI_WARN("Alsc: no AWB results found, using "
			 << awb_status.temperature_K);
	else
		RPI_LOG("Alsc: AWB results found, using "
			<< awb_status.temperature_K);
	return awb_status.temperature_K;
}

static void copy_stats(bcm2835_isp_stats_region regions[XY], StatisticsPtr &stats,
		       AlscStatus const &status)
{
	bcm2835_isp_stats_region *input_regions = stats->awb_stats;
	double *r_table = (double *)status.r;
	double *g_table = (double *)status.g;
	double *b_table = (double *)status.b;
	for (int i = 0; i < XY; i++) {
		regions[i].r_sum = input_regions[i].r_sum / r_table[i];
		regions[i].g_sum = input_regions[i].g_sum / g_table[i];
		regions[i].b_sum = input_regions[i].b_sum / b_table[i];
		regions[i].counted = input_regions[i].counted;
		// (don't care about the uncounted value)
	}
}

void Alsc::restartAsync(StatisticsPtr &stats, Metadata *image_metadata)
{
	RPI_LOG("Starting ALSC thread");
	// Get the current colour temperature. It's all we need from the
	// metadata.
	ct_ = get_ct(image_metadata, config_.default_ct);
	// We have to copy the statistics here, dividing out our best guess of
	// the LSC table that the pipeline applied to them.
	AlscStatus alsc_status;
	if (image_metadata->Get("alsc.status", alsc_status) != 0) {
		RPI_WARN("No ALSC status found for applied gains!");
		for (int y = 0; y < Y; y++)
			for (int x = 0; x < X; x++) {
				alsc_status.r[y][x] = 1.0;
				alsc_status.g[y][x] = 1.0;
				alsc_status.b[y][x] = 1.0;
			}
	}
	copy_stats(statistics_, stats, alsc_status);
	frame_phase_ = 0;
	// copy the camera mode so it won't change during the calculations
	async_camera_mode_ = camera_mode_;
	async_start_ = true;
	async_started_ = true;
	async_signal_.notify_one();
}

void Alsc::Prepare(Metadata *image_metadata)
{
	// Count frames since we started, and since we last poked the async
	// thread.
	if (frame_count_ < (int)config_.startup_frames)
		frame_count_++;
	double speed = frame_count_ < (int)config_.startup_frames
			       ? 1.0
			       : config_.speed;
	RPI_LOG("Alsc: frame_count " << frame_count_ << " speed " << speed);
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (async_started_ && async_finished_) {
			RPI_LOG("ALSC thread finished");
			fetchAsyncResults();
		}
	}
	// Apply IIR filter to results and program into the pipeline.
	double *ptr = (double *)sync_results_,
	       *pptr = (double *)prev_sync_results_;
	for (unsigned int i = 0;
	     i < sizeof(sync_results_) / sizeof(double); i++)
		pptr[i] = speed * ptr[i] + (1.0 - speed) * pptr[i];
	// Put output values into status metadata.
	AlscStatus status;
	memcpy(status.r, prev_sync_results_[0], sizeof(status.r));
	memcpy(status.g, prev_sync_results_[1], sizeof(status.g));
	memcpy(status.b, prev_sync_results_[2], sizeof(status.b));
	image_metadata->Set("alsc.status", status);
}

void Alsc::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	// Count frames since we started, and since we last poked the async
	// thread.
	if (frame_phase_ < (int)config_.frame_period)
		frame_phase_++;
	if (frame_count2_ < (int)config_.startup_frames)
		frame_count2_++;
	RPI_LOG("Alsc: frame_phase " << frame_phase_);
	if (frame_phase_ >= (int)config_.frame_period ||
	    frame_count2_ < (int)config_.startup_frames) {
		std::unique_lock<std::mutex> lock(mutex_);
		if (async_started_ == false) {
			RPI_LOG("ALSC thread starting");
			restartAsync(stats, image_metadata);
		}
	}
}

void Alsc::asyncFunc()
{
	while (true) {
		{
			std::unique_lock<std::mutex> lock(mutex_);
			async_signal_.wait(lock, [&] {
				return async_start_ || async_abort_;
			});
			async_start_ = false;
			if (async_abort_)
				break;
		}
		doAlsc();
		{
			std::lock_guard<std::mutex> lock(mutex_);
			async_finished_ = true;
			sync_signal_.notify_one();
		}
	}
}

void get_cal_table(double ct, std::vector<AlscCalibration> const &calibrations,
		   double cal_table[XY])
{
	if (calibrations.empty()) {
		for (int i = 0; i < XY; i++)
			cal_table[i] = 1.0;
		RPI_LOG("Alsc: no calibrations found");
	} else if (ct <= calibrations.front().ct) {
		memcpy(cal_table, calibrations.front().table,
		       XY * sizeof(double));
		RPI_LOG("Alsc: using calibration for "
			<< calibrations.front().ct);
	} else if (ct >= calibrations.back().ct) {
		memcpy(cal_table, calibrations.back().table,
		       XY * sizeof(double));
		RPI_LOG("Alsc: using calibration for "
			<< calibrations.front().ct);
	} else {
		int idx = 0;
		while (ct > calibrations[idx + 1].ct)
			idx++;
		double ct0 = calibrations[idx].ct,
		       ct1 = calibrations[idx + 1].ct;
		RPI_LOG("Alsc: ct is " << ct << ", interpolating between "
				       << ct0 << " and " << ct1);
		for (int i = 0; i < XY; i++)
			cal_table[i] =
				(calibrations[idx].table[i] * (ct1 - ct) +
				 calibrations[idx + 1].table[i] * (ct - ct0)) /
				(ct1 - ct0);
	}
}

void resample_cal_table(double const cal_table_in[XY],
			CameraMode const &camera_mode, double cal_table_out[XY])
{
	// Precalculate and cache the x sampling locations and phases to save
	// recomputing them on every row.
	int x_lo[X], x_hi[X];
	double xf[X];
	double scale_x = camera_mode.sensor_width /
			 (camera_mode.width * camera_mode.scale_x);
	double x_off = camera_mode.crop_x / (double)camera_mode.sensor_width;
	double x = .5 / scale_x + x_off * X - .5;
	double x_inc = 1 / scale_x;
	for (int i = 0; i < X; i++, x += x_inc) {
		x_lo[i] = floor(x);
		xf[i] = x - x_lo[i];
		x_hi[i] = std::min(x_lo[i] + 1, X - 1);
		x_lo[i] = std::max(x_lo[i], 0);
	}
	// Now march over the output table generating the new values.
	double scale_y = camera_mode.sensor_height /
			 (camera_mode.height * camera_mode.scale_y);
	double y_off = camera_mode.crop_y / (double)camera_mode.sensor_height;
	double y = .5 / scale_y + y_off * Y - .5;
	double y_inc = 1 / scale_y;
	for (int j = 0; j < Y; j++, y += y_inc) {
		int y_lo = floor(y);
		double yf = y - y_lo;
		int y_hi = std::min(y_lo + 1, Y - 1);
		y_lo = std::max(y_lo, 0);
		double const *row_above = cal_table_in + X * y_lo;
		double const *row_below = cal_table_in + X * y_hi;
		for (int i = 0; i < X; i++) {
			double above = row_above[x_lo[i]] * (1 - xf[i]) +
				       row_above[x_hi[i]] * xf[i];
			double below = row_below[x_lo[i]] * (1 - xf[i]) +
				       row_below[x_hi[i]] * xf[i];
			*(cal_table_out++) = above * (1 - yf) + below * yf;
		}
	}
}

// Calculate chrominance statistics (R/G and B/G) for each region.
static_assert(XY == AWB_REGIONS, "ALSC/AWB statistics region mismatch");
static void calculate_Cr_Cb(bcm2835_isp_stats_region *awb_region, double Cr[XY],
			    double Cb[XY], uint32_t min_count, uint16_t min_G)
{
	for (int i = 0; i < XY; i++) {
		bcm2835_isp_stats_region &zone = awb_region[i];
		if (zone.counted <= min_count ||
		    zone.g_sum / zone.counted <= min_G) {
			Cr[i] = Cb[i] = INSUFFICIENT_DATA;
			continue;
		}
		Cr[i] = zone.r_sum / (double)zone.g_sum;
		Cb[i] = zone.b_sum / (double)zone.g_sum;
	}
}

static void apply_cal_table(double const cal_table[XY], double C[XY])
{
	for (int i = 0; i < XY; i++)
		if (C[i] != INSUFFICIENT_DATA)
			C[i] *= cal_table[i];
}

void compensate_lambdas_for_cal(double const cal_table[XY],
				double const old_lambdas[XY],
				double new_lambdas[XY])
{
	double min_new_lambda = std::numeric_limits<double>::max();
	for (int i = 0; i < XY; i++) {
		new_lambdas[i] = old_lambdas[i] * cal_table[i];
		min_new_lambda = std::min(min_new_lambda, new_lambdas[i]);
	}
	for (int i = 0; i < XY; i++)
		new_lambdas[i] /= min_new_lambda;
}

static void print_cal_table(double const C[XY])
{
	printf("table: [\n");
	for (int j = 0; j < Y; j++) {
		for (int i = 0; i < X; i++) {
			printf("%5.3f", 1.0 / C[j * X + i]);
			if (i != X - 1 || j != Y - 1)
				printf(",");
		}
		printf("\n");
	}
	printf("]\n");
}

// Compute weight out of 1.0 which reflects how similar we wish to make the
// colours of these two regions.
static double compute_weight(double C_i, double C_j, double sigma)
{
	if (C_i == INSUFFICIENT_DATA || C_j == INSUFFICIENT_DATA)
		return 0;
	double diff = (C_i - C_j) / sigma;
	return exp(-diff * diff / 2);
}

// Compute all weights.
static void compute_W(double const C[XY], double sigma, double W[XY][4])
{
	for (int i = 0; i < XY; i++) {
		// Start with neighbour above and go clockwise.
		W[i][0] = i >= X ? compute_weight(C[i], C[i - X], sigma) : 0;
		W[i][1] = i % X < X - 1 ? compute_weight(C[i], C[i + 1], sigma)
					: 0;
		W[i][2] =
			i < XY - X ? compute_weight(C[i], C[i + X], sigma) : 0;
		W[i][3] = i % X ? compute_weight(C[i], C[i - 1], sigma) : 0;
	}
}

// Compute M, the large but sparse matrix such that M * lambdas = 0.
static void construct_M(double const C[XY], double const W[XY][4],
			double M[XY][4])
{
	double epsilon = 0.001;
	for (int i = 0; i < XY; i++) {
		// Note how, if C[i] == INSUFFICIENT_DATA, the weights will all
		// be zero so the equation is still set up correctly.
		int m = !!(i >= X) + !!(i % X < X - 1) + !!(i < XY - X) +
			!!(i % X); // total number of neighbours
		// we'll divide the diagonal out straight away
		double diagonal =
			(epsilon + W[i][0] + W[i][1] + W[i][2] + W[i][3]) *
			C[i];
		M[i][0] = i >= X ? (W[i][0] * C[i - X] + epsilon / m * C[i]) /
					   diagonal
				 : 0;
		M[i][1] = i % X < X - 1
				  ? (W[i][1] * C[i + 1] + epsilon / m * C[i]) /
					    diagonal
				  : 0;
		M[i][2] = i < XY - X
				  ? (W[i][2] * C[i + X] + epsilon / m * C[i]) /
					    diagonal
				  : 0;
		M[i][3] = i % X ? (W[i][3] * C[i - 1] + epsilon / m * C[i]) /
					  diagonal
				: 0;
	}
}

// In the compute_lambda_ functions, note that the matrix coefficients for the
// left/right neighbours are zero down the left/right edges, so we don't need
// need to test the i value to exclude them.
static double compute_lambda_bottom(int i, double const M[XY][4],
				    double lambda[XY])
{
	return M[i][1] * lambda[i + 1] + M[i][2] * lambda[i + X] +
	       M[i][3] * lambda[i - 1];
}
static double compute_lambda_bottom_start(int i, double const M[XY][4],
					  double lambda[XY])
{
	return M[i][1] * lambda[i + 1] + M[i][2] * lambda[i + X];
}
static double compute_lambda_interior(int i, double const M[XY][4],
				      double lambda[XY])
{
	return M[i][0] * lambda[i - X] + M[i][1] * lambda[i + 1] +
	       M[i][2] * lambda[i + X] + M[i][3] * lambda[i - 1];
}
static double compute_lambda_top(int i, double const M[XY][4],
				 double lambda[XY])
{
	return M[i][0] * lambda[i - X] + M[i][1] * lambda[i + 1] +
	       M[i][3] * lambda[i - 1];
}
static double compute_lambda_top_end(int i, double const M[XY][4],
				     double lambda[XY])
{
	return M[i][0] * lambda[i - X] + M[i][3] * lambda[i - 1];
}

// Gauss-Seidel iteration with over-relaxation.
static double gauss_seidel2_SOR(double const M[XY][4], double omega,
				double lambda[XY])
{
	double old_lambda[XY];
	for (int i = 0; i < XY; i++)
		old_lambda[i] = lambda[i];
	int i;
	lambda[0] = compute_lambda_bottom_start(0, M, lambda);
	for (i = 1; i < X; i++)
		lambda[i] = compute_lambda_bottom(i, M, lambda);
	for (; i < XY - X; i++)
		lambda[i] = compute_lambda_interior(i, M, lambda);
	for (; i < XY - 1; i++)
		lambda[i] = compute_lambda_top(i, M, lambda);
	lambda[i] = compute_lambda_top_end(i, M, lambda);
	// Also solve the system from bottom to top, to help spread the updates
	// better.
	lambda[i] = compute_lambda_top_end(i, M, lambda);
	for (i = XY - 2; i >= XY - X; i--)
		lambda[i] = compute_lambda_top(i, M, lambda);
	for (; i >= X; i--)
		lambda[i] = compute_lambda_interior(i, M, lambda);
	for (; i >= 1; i--)
		lambda[i] = compute_lambda_bottom(i, M, lambda);
	lambda[0] = compute_lambda_bottom_start(0, M, lambda);
	double max_diff = 0;
	for (int i = 0; i < XY; i++) {
		lambda[i] = old_lambda[i] + (lambda[i] - old_lambda[i]) * omega;
		if (fabs(lambda[i] - old_lambda[i]) > fabs(max_diff))
			max_diff = lambda[i] - old_lambda[i];
	}
	return max_diff;
}

// Normalise the values so that the smallest value is 1.
static void normalise(double *ptr, size_t n)
{
	double minval = ptr[0];
	for (size_t i = 1; i < n; i++)
		minval = std::min(minval, ptr[i]);
	for (size_t i = 0; i < n; i++)
		ptr[i] /= minval;
}

static void run_matrix_iterations(double const C[XY], double lambda[XY],
				  double const W[XY][4], double omega,
				  int n_iter, double threshold)
{
	double M[XY][4];
	construct_M(C, W, M);
	double last_max_diff = std::numeric_limits<double>::max();
	for (int i = 0; i < n_iter; i++) {
		double max_diff = fabs(gauss_seidel2_SOR(M, omega, lambda));
		if (max_diff < threshold) {
			RPI_LOG("Stop after " << i + 1 << " iterations");
			break;
		}
		// this happens very occasionally (so make a note), though
		// doesn't seem to matter
		if (max_diff > last_max_diff)
			RPI_LOG("Iteration " << i << ": max_diff gone up "
					     << last_max_diff << " to "
					     << max_diff);
		last_max_diff = max_diff;
	}
	// We're going to normalise the lambdas so the smallest is 1. Not sure
	// this is really necessary as they get renormalised later, but I
	// suppose it does stop these quantities from wandering off...
	normalise(lambda, XY);
}

static void add_luminance_rb(double result[XY], double const lambda[XY],
			     double const luminance_lut[XY],
			     double luminance_strength)
{
	for (int i = 0; i < XY; i++)
		result[i] = lambda[i] *
			    ((luminance_lut[i] - 1) * luminance_strength + 1);
}

static void add_luminance_g(double result[XY], double lambda,
			    double const luminance_lut[XY],
			    double luminance_strength)
{
	for (int i = 0; i < XY; i++)
		result[i] = lambda *
			    ((luminance_lut[i] - 1) * luminance_strength + 1);
}

void add_luminance_to_tables(double results[3][Y][X], double const lambda_r[XY],
			     double lambda_g, double const lambda_b[XY],
			     double const luminance_lut[XY],
			     double luminance_strength)
{
	add_luminance_rb((double *)results[0], lambda_r, luminance_lut,
			 luminance_strength);
	add_luminance_g((double *)results[1], lambda_g, luminance_lut,
			luminance_strength);
	add_luminance_rb((double *)results[2], lambda_b, luminance_lut,
			 luminance_strength);
	normalise((double *)results, 3 * XY);
}

void Alsc::doAlsc()
{
	double Cr[XY], Cb[XY], Wr[XY][4], Wb[XY][4], cal_table_r[XY],
		cal_table_b[XY], cal_table_tmp[XY];
	// Calculate our R/B ("Cr"/"Cb") colour statistics, and assess which are
	// usable.
	calculate_Cr_Cb(statistics_, Cr, Cb, config_.min_count, config_.min_G);
	// Fetch the new calibrations (if any) for this CT. Resample them in
	// case the camera mode is not full-frame.
	get_cal_table(ct_, config_.calibrations_Cr, cal_table_tmp);
	resample_cal_table(cal_table_tmp, async_camera_mode_, cal_table_r);
	get_cal_table(ct_, config_.calibrations_Cb, cal_table_tmp);
	resample_cal_table(cal_table_tmp, async_camera_mode_, cal_table_b);
	// You could print out the cal tables for this image here, if you're
	// tuning the algorithm...
	(void)print_cal_table;
	// Apply any calibration to the statistics, so the adaptive algorithm
	// makes only the extra adjustments.
	apply_cal_table(cal_table_r, Cr);
	apply_cal_table(cal_table_b, Cb);
	// Compute weights between zones.
	compute_W(Cr, config_.sigma_Cr, Wr);
	compute_W(Cb, config_.sigma_Cb, Wb);
	// Run Gauss-Seidel iterations over the resulting matrix, for R and B.
	run_matrix_iterations(Cr, lambda_r_, Wr, config_.omega, config_.n_iter,
			      config_.threshold);
	run_matrix_iterations(Cb, lambda_b_, Wb, config_.omega, config_.n_iter,
			      config_.threshold);
	// Fold the calibrated gains into our final lambda values. (Note that on
	// the next run, we re-start with the lambda values that don't have the
	// calibration gains included.)
	compensate_lambdas_for_cal(cal_table_r, lambda_r_, async_lambda_r_);
	compensate_lambdas_for_cal(cal_table_b, lambda_b_, async_lambda_b_);
	// Fold in the luminance table at the appropriate strength.
	add_luminance_to_tables(async_results_, async_lambda_r_, 1.0,
				async_lambda_b_, config_.luminance_lut,
				config_.luminance_strength);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Alsc(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
