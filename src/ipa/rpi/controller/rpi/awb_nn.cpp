/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025, Raspberry Pi Ltd
 *
 * AWB control algorithm using neural network
 */

#include <chrono>
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
	float minTemp;
	float maxTemp;

	bool enableNn;

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
	void prepareStats() override;

private:
	bool isAutoEnabled() const;
	AwbNNConfig nnConfig_;
	void transverseSearch(double t, double &r, double &b);
	RGB processZone(RGB zone, float red_gain, float blue_gain);
	void awbNN();
	void loadModel();

	libcamera::Size zoneSize_;
	std::unique_ptr<tflite::FlatBufferModel> model_;
	std::unique_ptr<tflite::Interpreter> interpreter_;
};

int AwbNNConfig::read(const libcamera::YamlObject &params, AwbConfig &config)
{
	model = params["model"].get<std::string>("");
	minTemp = params["min_temp"].get<float>(2800.0);
	maxTemp = params["max_temp"].get<float>(7600.0);

	for (int i = 0; i < 9; i++)
		ccm[i] = params["ccm"][i].get<double>(0.0);

	enableNn = params["enable_nn"].get<int>(1);

	if (enableNn) {
		if (!config.hasCtCurve()) {
			LOG(RPiAwb, Error) << "CT curve not specified";
			enableNn = false;
		}

		if (!model.empty() && model.find(".tflite") == std::string::npos) {
			LOG(RPiAwb, Error) << "Model must be a .tflite file";
			enableNn = false;
		}

		bool validCcm = true;
		for (int i = 0; i < 9; i++)
			if (ccm[i] == 0.0)
				validCcm = false;

		if (!validCcm) {
			LOG(RPiAwb, Error) << "CCM not specified or invalid";
			enableNn = false;
		}

		if (!enableNn) {
			LOG(RPiAwb, Warning) << "Neural Network AWB mis-configured - switch to Grey method";
		}
	}

	if (!enableNn) {
		config.sensitivityR = config.sensitivityB = 1.0;
		config.greyWorld = true;
	}

	return 0;
}

AwbNN::AwbNN(Controller *controller)
	: Awb(controller)
{
	zoneSize_ = getHardwareConfig().awbRegions;
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

static bool checkTensorShape(TfLiteTensor *tensor, const int *expectedDims, const int expectedDimsSize)
{
	if (tensor->dims->size != expectedDimsSize) {
		return false;
	}

	for (int i = 0; i < tensor->dims->size; i++) {
		if (tensor->dims->data[i] != expectedDims[i]) {
			return false;
		}
	}
	return true;
}

static std::string buildDimString(const int *dims, const int dimsSize)
{
	std::string s = "[";
	for (int i = 0; i < dimsSize; i++) {
		s += std::to_string(dims[i]);
		if (i < dimsSize - 1)
			s += ",";
		else
			s += "]";
	}
	return s;
}

void AwbNN::loadModel()
{
	std::string modelPath;
	if (getTarget() == "bcm2835") {
		modelPath = "/ipa/rpi/vc4/awb_model.tflite";
	} else {
		modelPath = "/ipa/rpi/pisp/awb_model.tflite";
	}

	if (nnConfig_.model.empty()) {
		std::string root = utils::libcameraSourcePath();
		if (!root.empty()) {
			modelPath = root + modelPath;
		} else {
			modelPath = LIBCAMERA_DATA_DIR + modelPath;
		}

		if (!File::exists(modelPath)) {
			LOG(RPiAwb, Error) << "No model file found in standard locations";
			nnConfig_.enableNn = false;
			return;
		}
	} else {
		modelPath = nnConfig_.model;
	}

	LOG(RPiAwb, Debug) << "Attempting to load model from: " << modelPath;

	model_ = tflite::FlatBufferModel::BuildFromFile(modelPath.c_str());

	if (!model_) {
		LOG(RPiAwb, Error) << "Failed to load model from " << modelPath;
		nnConfig_.enableNn = false;
		return;
	}

	tflite::MutableOpResolver resolver;
	tflite::ops::builtin::BuiltinOpResolver builtin_resolver;
	resolver.AddAll(builtin_resolver);
	tflite::InterpreterBuilder(*model_, resolver)(&interpreter_);
	if (!interpreter_) {
		LOG(RPiAwb, Error) << "Failed to build interpreter for model " << nnConfig_.model;
		nnConfig_.enableNn = false;
		return;
	}

	interpreter_->AllocateTensors();
	TfLiteTensor *inputTensor = interpreter_->input_tensor(0);
	TfLiteTensor *inputLuxTensor = interpreter_->input_tensor(1);
	TfLiteTensor *outputTensor = interpreter_->output_tensor(0);
	if (!inputTensor || !inputLuxTensor || !outputTensor) {
		LOG(RPiAwb, Error) << "Model missing input or output tensor";
		nnConfig_.enableNn = false;
		return;
	}

	const int expectedInputDims[] = { 1, (int)zoneSize_.height, (int)zoneSize_.width, 3 };
	const int expectedInputLuxDims[] = { 1 };
	const int expectedOutputDims[] = { 1 };

	if (!checkTensorShape(inputTensor, expectedInputDims, 4)) {
		LOG(RPiAwb, Error) << "Model input tensor dimension mismatch. Expected: " << buildDimString(expectedInputDims, 4)
				   << ", Got: " << buildDimString(inputTensor->dims->data, inputTensor->dims->size);
		nnConfig_.enableNn = false;
		return;
	}

	if (!checkTensorShape(inputLuxTensor, expectedInputLuxDims, 1)) {
		LOG(RPiAwb, Error) << "Model input lux tensor dimension mismatch. Expected: " << buildDimString(expectedInputLuxDims, 1)
				   << ", Got: " << buildDimString(inputLuxTensor->dims->data, inputLuxTensor->dims->size);
		nnConfig_.enableNn = false;
		return;
	}

	if (!checkTensorShape(outputTensor, expectedOutputDims, 1)) {
		LOG(RPiAwb, Error) << "Model output tensor dimension mismatch. Expected: " << buildDimString(expectedOutputDims, 1)
				   << ", Got: " << buildDimString(outputTensor->dims->data, outputTensor->dims->size);
		nnConfig_.enableNn = false;
		return;
	}

	if (inputTensor->type != kTfLiteFloat32 || inputLuxTensor->type != kTfLiteFloat32 || outputTensor->type != kTfLiteFloat32) {
		LOG(RPiAwb, Error) << "Model input and output tensors must be float32";
		nnConfig_.enableNn = false;
		return;
	}

	LOG(RPiAwb, Info) << "Model loaded successfully from " << modelPath;
	LOG(RPiAwb, Debug) << "Model validation successful - Input Image: "
			   << buildDimString(expectedInputDims, 4)
			   << ", Input Lux: " << buildDimString(expectedInputLuxDims, 1)
			   << ", Output: " << buildDimString(expectedOutputDims, 1) << " floats";
}

void AwbNN::initialise()
{
	Awb::initialise();

	if (nnConfig_.enableNn) {
		loadModel();
		if (!nnConfig_.enableNn) {
			LOG(RPiAwb, Warning) << "Neural Network AWB failed to load - switch to Grey method";
			config_.greyWorld = true;
			config_.sensitivityR = config_.sensitivityB = 1.0;
		}
	}
}

void AwbNN::prepareStats()
{
	zones_.clear();
	/*
	 * LSC has already been applied to the stats in this pipeline, so stop
	 * any LSC compensation.  We also ignore config_.fast in this version.
	 */
	generateStats(zones_, statistics_, 0.0, 0.0, getGlobalMetadata(), 0.0, 0.0, 0.0);
	/*
	 * apply sensitivities, so values appear to come from our "canonical"
	 * sensor.
	 */
	for (auto &zone : zones_) {
		zone.R *= config_.sensitivityR;
		zone.B *= config_.sensitivityB;
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

AwbNN::RGB AwbNN::processZone(AwbNN::RGB zone, float redGain, float blueGain)
{
	/*
	 * Renders the pixel at 5000K temperature
	 */
	RGB zoneGains = zone;

	zoneGains.R *= redGain;
	zoneGains.G *= 1.0;
	zoneGains.B *= blueGain;

	RGB zoneCcm;

	zoneCcm.R = nnConfig_.ccm[0] * zoneGains.R + nnConfig_.ccm[1] * zoneGains.G + nnConfig_.ccm[2] * zoneGains.B;
	zoneCcm.G = nnConfig_.ccm[3] * zoneGains.R + nnConfig_.ccm[4] * zoneGains.G + nnConfig_.ccm[5] * zoneGains.B;
	zoneCcm.B = nnConfig_.ccm[6] * zoneGains.R + nnConfig_.ccm[7] * zoneGains.G + nnConfig_.ccm[8] * zoneGains.B;

	return zoneCcm;
}

void AwbNN::awbNN()
{
	float *inputData = interpreter_->typed_input_tensor<float>(0);
	float *inputLux = interpreter_->typed_input_tensor<float>(1);

	float redGain = 1.0 / config_.ctR.eval(5000);
	float blueGain = 1.0 / config_.ctB.eval(5000);

	for (uint i = 0; i < zoneSize_.height; i++) {
		for (uint j = 0; j < zoneSize_.width; j++) {
			uint zoneIdx = i * zoneSize_.width + j;

			RGB processedZone = processZone(zones_[zoneIdx] * (1.0 / 65535), redGain, blueGain);
			uint baseIdx = zoneIdx * 3;

			inputData[baseIdx + 0] = static_cast<float>(processedZone.R);
			inputData[baseIdx + 1] = static_cast<float>(processedZone.G);
			inputData[baseIdx + 2] = static_cast<float>(processedZone.B);
		}
	}

	inputLux[0] = static_cast<float>(lux_);

	TfLiteStatus status = interpreter_->Invoke();
	if (status != kTfLiteOk) {
		LOG(RPiAwb, Error) << "Model inference failed with status: " << status;
		return;
	}

	float *outputData = interpreter_->typed_output_tensor<float>(0);

	double t = outputData[0];

	LOG(RPiAwb, Debug) << "Model output temperature: " << t;

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
	if (zones_.size() == (zoneSize_.width * zoneSize_.height) && nnConfig_.enableNn) {
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
