/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * AWB control algorithm using neural network
 */

#include <condition_variable>
#include <thread>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>

#include "../awb_algorithm.h"
#include "../awb_status.h"
#include "../lux_status.h"
#include "libipa/pwl.h"

#include "alsc_status.h"
#include "awb.h"

using namespace libcamera;

LOG_DECLARE_CATEGORY(RPiAwb)

constexpr double kDefaultCT = 4500.0;

#define NAME "rpi.nn.awb"

namespace RPiController {

struct AwbNNConfig {
	AwbNNConfig() {}
	int read(const libcamera::YamlObject &params, AwbConfig &config);

	/* An empty model will check default locations for model.tflite */
	std::string model;
	int numOutputs;
	float minTemp;
	float maxTemp;

	bool enable_nn;

	/* CCM matrix for 5000K temperature */
	double ccm[9];
};

class AwbNN : public Awb
{
public:
	AwbNN(Controller *controller = NULL);
	~AwbNN();
	char const *name() const override;
	void initialise() override;
	int read(const libcamera::YamlObject &params) override;

protected:
	void doAwb() override;

private:
	bool isAutoEnabled() const;
	AwbNNConfig nnConfig_;
	void transverseSearch(double t, double &r, double &b);
	RGB processZone(RGB zone, float lux, float red_gain, float blue_gain);
	void awbNN();
	void loadModel();

	std::unique_ptr<tflite::FlatBufferModel> model_;
	std::unique_ptr<tflite::Interpreter> interpreter_;
};

int AwbNNConfig::read(const libcamera::YamlObject &params, AwbConfig &config)
{
	model = params["model"].get<std::string>("");
	numOutputs = params["num_outputs"].get<int>(10);
	minTemp = params["min_temp"].get<float>(2000.0);
	maxTemp = params["max_temp"].get<float>(8000.0);

	for (int i = 0; i < 9; i++)
		ccm[i] = params["ccm"][i].get<double>(0.0);

	enable_nn = params["enable_nn"].get<int>(1);

	if (enable_nn) {
		if (!config.hasCtCurve()) {
			LOG(RPiAwb, Error) << "CT curve not specified";
			enable_nn = false;
		}

		if (!model.empty() && model.find(".tflite") == std::string::npos) {
			LOG(RPiAwb, Error) << "Model must be a .tflite file";
			enable_nn = false;
		}

		bool valid_ccm = true;
		for (int i = 0; i < 9; i++)
			if (ccm[i] == 0.0)
				valid_ccm = false;

		if (!valid_ccm) {
			LOG(RPiAwb, Error) << "CCM not specified or invalid";
			enable_nn = false;
		}

		if (!enable_nn) {
			LOG(RPiAwb, Warning) << "Neural Network AWB mis-configured - switch to Grey method";
		}
	}

	if (!enable_nn) {
		config.sensitivityR = config.sensitivityB = 1.0;
		config.greyWorld = false;
	}

	return 0;
}

AwbNN::AwbNN(Controller *controller)
	: Awb(controller)
{
}

AwbNN::~AwbNN()
{
}

char const *AwbNN::name() const
{
	return NAME;
}

int AwbNN::read(const libcamera::YamlObject &params)
{
	int ret;

	ret = config_.read(params);
	if (ret)
		return ret;

	ret = nnConfig_.read(params, config_);
	if (ret)
		return ret;

	return 0;
}

void AwbNN::loadModel()
{
	LOG(RPiAwb, Debug) << "Loading AWB model " << nnConfig_.model;
	std::string modelPath = "/ipa/rpi/pisp/awb_model.tflite";
	if (nnConfig_.model.empty()) {
		std::string root = utils::libcameraSourcePath();
		if (!root.empty()) {
			modelPath = root + modelPath;
		} else {
			modelPath = LIBCAMERA_DATA_DIR + modelPath;
		}

		if (!File::exists(modelPath)) {
			LOG(RPiAwb, Error) << "No model file found in standard locations";
			nnConfig_.enable_nn = false;
			return;
		}
	} else {
		modelPath = nnConfig_.model;
	}

	model_ = tflite::FlatBufferModel::BuildFromFile(modelPath.c_str());

	if (!model_) {
		LOG(RPiAwb, Error) << "Failed to load model from " << modelPath;
		nnConfig_.enable_nn = false;
		return;
	}

	tflite::MutableOpResolver resolver;
	tflite::ops::builtin::BuiltinOpResolver builtin_resolver;
	resolver.AddAll(builtin_resolver);
	tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
	if (!interpreter_) {
		LOG(RPiAwb, Error) << "Failed to build interpreter for model " << nnConfig_.model;
		nnConfig_.enable_nn = false;
		return;
	}

	interpreter_->AllocateTensors();
	TfLiteTensor *input_tensor = interpreter_->input_tensor(0);
	TfLiteTensor *output_tensor = interpreter_->output_tensor(0);
	if (!input_tensor || !output_tensor) {
		LOG(RPiAwb, Error) << "Model missing input or output tensor";
		nnConfig_.enable_nn = false;
		return;
	}

	const int expected_input_size = 32 * 32 * 3;
	int actual_input_size = 1;
	for (int i = 0; i < input_tensor->dims->size; i++) {
		actual_input_size *= input_tensor->dims->data[i];
	}

	if (actual_input_size != expected_input_size) {
		LOG(RPiAwb, Error) << "Model input tensor size mismatch. Expected: "
				   << expected_input_size << ", Got: " << actual_input_size;
		nnConfig_.enable_nn = false;
		return;
	}

	int actual_output_size = 1;
	for (int i = 0; i < output_tensor->dims->size; i++) {
		actual_output_size *= output_tensor->dims->data[i];
	}

	if (actual_output_size != nnConfig_.numOutputs) {
		LOG(RPiAwb, Error) << "Model output tensor size mismatch. Expected: "
				   << nnConfig_.numOutputs << ", Got: " << actual_output_size;
		nnConfig_.enable_nn = false;
		return;
	}

	if (input_tensor->type != kTfLiteFloat32 || output_tensor->type != kTfLiteFloat32) {
		LOG(RPiAwb, Error) << "Model input and output tensor must be float32";
		nnConfig_.enable_nn = false;
		return;
	}

	LOG(RPiAwb, Info) << "Model loaded successfully from " << modelPath;
	LOG(RPiAwb, Debug) << "Model validation successful - Input: " << actual_input_size
			   << " floats, Output: " << actual_output_size << " floats";
}

void AwbNN::initialise()
{
	Awb::initialise();

	if (nnConfig_.enable_nn) {
		loadModel();
		if (!nnConfig_.enable_nn) {
			LOG(RPiAwb, Warning) << "Neural Network AWB failed to load - switch to Grey method";
			config_.greyWorld = false;
			config_.sensitivityR = config_.sensitivityB = 1.0;
		}
	}
}

void AwbNN::transverseSearch(double t, double &r, double &b)
{
	int spanR = -1, spanB = -1;
	config_.ctR.eval(t, &spanR);
	config_.ctB.eval(t, &spanB);

	const int diff = 10;
	double rDiff = config_.ctR.eval(t + diff, &spanR) -
		       config_.ctR.eval(t - diff, &spanR);
	double bDiff = config_.ctB.eval(t + diff, &spanB) -
		       config_.ctB.eval(t - diff, &spanB);

	ipa::Pwl::Point transverse({ bDiff, -rDiff });
	if (transverse.length2() < 1e-6)
		return;

	transverse = transverse / transverse.length();
	double transverseRange = config_.transverseNeg + config_.transversePos;
	const int maxNumDeltas = 12;
	int numDeltas = floor(transverseRange * 100 + 0.5) + 1;
	numDeltas = numDeltas < 3 ? 3 : (numDeltas > maxNumDeltas ? maxNumDeltas : numDeltas);

	ipa::Pwl::Point points[maxNumDeltas];
	int bestPoint = 0;

	for (int i = 0; i < numDeltas; i++) {
		points[i][0] = -config_.transverseNeg +
			       (transverseRange * i) / (numDeltas - 1);
		ipa::Pwl::Point rbTest = ipa::Pwl::Point({ r, b }) +
					 transverse * points[i].x();
		double rTest = rbTest.x(), bTest = rbTest.y();
		double gainR = 1 / rTest, gainB = 1 / bTest;
		double delta2Sum = computeDelta2Sum(gainR, gainB, 0.0, 0.0);
		points[i][1] = delta2Sum;
		if (points[i].y() < points[bestPoint].y())
			bestPoint = i;
	}

	bestPoint = std::clamp(bestPoint, 1, numDeltas - 2);
	ipa::Pwl::Point rbBest = ipa::Pwl::Point({ r, b }) +
				 transverse * interpolateQuadatric(points[bestPoint - 1],
								   points[bestPoint],
								   points[bestPoint + 1]);
	double rBest = rbBest.x(), bBest = rbBest.y();

	r = rBest, b = bBest;
}

static double getElementPadded(const std::vector<double> &array, int i, int j)
{
	int i_padded = std::clamp(i, 0, 31);
	int j_padded = std::clamp(j, 0, 31);
	return array[i_padded * 32 + j_padded];
}

static std::vector<AwbNN::RGB> filter(const std::vector<AwbNN::RGB> &rgb)
{
	/*
	 * Decreases the difference between the colour channels when it is higher
	 * than nearby pixels.
	 */
	const int total_pixels = 32 * 32;

	std::vector<AwbNN::RGB> new_rgb(rgb.size());

	std::vector<double> r_diff(total_pixels);
	std::vector<double> b_diff(total_pixels);

	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 32; j++) {
			int idx = i * 32 + j;
			const AwbNN::RGB &pixel = rgb[idx];
			r_diff[idx] = pixel.R - pixel.G;
			b_diff[idx] = pixel.B - pixel.G;
		}
	}

	std::vector<double> min_r_diff(total_pixels);
	std::vector<double> min_b_diff(total_pixels);

	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 32; j++) {
			int idx = i * 32 + j;

			double min_abs_r = std::numeric_limits<double>::max();
			double min_abs_b = std::numeric_limits<double>::max();

			double selected_r = 0.0;
			double selected_b = 0.0;

			for (int di = -1; di <= 1; di++) {
				for (int dj = -1; dj <= 1; dj++) {
					double r = getElementPadded(r_diff, i + di, j + dj);
					double b = getElementPadded(b_diff, i + di, j + dj);

					if (std::abs(r) < min_abs_r) {
						min_abs_r = std::abs(r);
						selected_r = r;
					}

					if (std::abs(b) < min_abs_b) {
						min_abs_b = std::abs(b);
						selected_b = b;
					}
				}
			}
			min_r_diff[idx] = selected_r;
			min_b_diff[idx] = selected_b;
		}
	}

	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 32; j++) {
			int idx = i * 32 + j;

			const AwbNN::RGB &original_pixel = rgb[idx];
			double g = original_pixel.G;
			AwbNN::RGB new_pixel(g + min_r_diff[idx], g, g + min_b_diff[idx]);
			new_rgb[idx] = new_pixel;
		}
	}

	return new_rgb;
}

AwbNN::RGB AwbNN::processZone(AwbNN::RGB zone, float lux, float red_gain, float blue_gain)
{
	/*
	 * Renders the pixel at 5000K temperature then takes the log of the
	 * r/g and b/g ratios. The log is then scaled to 0-1 and clamped.
	 * These are then combined with the lux to get the input for the.
	 * neural network.
	 */
	RGB zone_gains = zone;

	zone_gains.R *= red_gain;
	zone_gains.G *= 1.0;
	zone_gains.B *= blue_gain;

	RGB zone_ccm = zone_gains;

	zone_ccm.R = nnConfig_.ccm[0] * zone_gains.R + nnConfig_.ccm[1] * zone_gains.G + nnConfig_.ccm[2] * zone_gains.B;
	zone_ccm.G = nnConfig_.ccm[3] * zone_gains.R + nnConfig_.ccm[4] * zone_gains.G + nnConfig_.ccm[5] * zone_gains.B;
	zone_ccm.B = nnConfig_.ccm[6] * zone_gains.R + nnConfig_.ccm[7] * zone_gains.G + nnConfig_.ccm[8] * zone_gains.B;

	RGB zone_log;

	double green = std::clamp(zone_ccm.G, 0.1, 1.0);

	double r_ratio = zone_ccm.R / green;
	double b_ratio = zone_ccm.B / green;

	r_ratio = std::clamp(r_ratio, 0.01, 1.0);
	b_ratio = std::clamp(b_ratio, 0.01, 1.0);

	zone_log.R = std::log(r_ratio);
	zone_log.G = std::log(lux) / 10;
	zone_log.B = std::log(b_ratio);

	zone_log.R += 1.0;
	zone_log.B += 1.0;

	zone_log.R /= 2.0;
	zone_log.B /= 2.0;

	zone_log.R = std::clamp(zone_log.R, 0.0, 1.0);
	zone_log.G = std::clamp(zone_log.G, 0.0, 1.0);
	zone_log.B = std::clamp(zone_log.B, 0.0, 1.0);

	return zone_log;
}

static float interpolateOutput(float *output_data, int num_outputs, float min_temp, float max_temp)
{
	float total_value = 0.0;
	float total_weight = 0.0;

	for (int i = 0; i < num_outputs; i++) {
		float temp = min_temp + i * (max_temp - min_temp) / (num_outputs - 1);
		float weight = output_data[i];
		weight *= weight;
		total_value += temp * weight;
		total_weight += weight;
	}

	return total_value / total_weight;
}

void AwbNN::awbNN()
{
	float *input_data = interpreter_->typed_input_tensor<float>(0);
	float *output_data = interpreter_->typed_output_tensor<float>(0);

	float red_gain = 1.0 / config_.ctR.eval(5000);
	float blue_gain = 1.0 / config_.ctB.eval(5000);

	std::vector<RGB> zones = filter(zones_);

	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 32; j++) {
			int zone_idx = i * 32 + j;

			RGB processedZone = processZone(zones[zone_idx] * (1.0 / 65535), lux_, red_gain, blue_gain);

			int base_idx = (i * 32 + j) * 3;

			input_data[base_idx + 0] = static_cast<float>(processedZone.R);
			input_data[base_idx + 1] = static_cast<float>(processedZone.G);
			input_data[base_idx + 2] = static_cast<float>(processedZone.B);
		}
	}

	interpreter_->Invoke();

	LOG(RPiAwb, Debug) << "Output data: ";
	for (int i = 0; i < nnConfig_.numOutputs; i++) {
		LOG(RPiAwb, Debug) << "Output " << i << ": " << output_data[i];
	}

	double t = interpolateOutput(output_data, nnConfig_.numOutputs, nnConfig_.minTemp, nnConfig_.maxTemp);

	LOG(RPiAwb, Debug) << "Interpolated temperature: " << t;

	t = std::clamp(t, mode_->ctLo, mode_->ctHi);

	double r = config_.ctR.eval(t);
	double b = config_.ctB.eval(t);

	transverseSearch(t, r, b);

	LOG(RPiAwb, Debug) << "After transverse search: Temperature: " << t << " Red gain: " << 1.0 / r << " Blue gain: " << 1.0 / b;

	asyncResults_.temperatureK = t;
	asyncResults_.gainR = 1.0 / r * config_.sensitivityR;
	asyncResults_.gainG = 1.0;
	asyncResults_.gainB = 1.0 / b * config_.sensitivityB;
}

void AwbNN::doAwb()
{
	prepareStats();
	if (zones_.size() == 1024 && nnConfig_.enable_nn) {
		awbNN();
	} else {
		awbGrey();
	}
	statistics_.reset();
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new AwbNN(controller);
}
static RegisterAlgorithm reg(NAME, &create);

} /* namespace RPiController */
