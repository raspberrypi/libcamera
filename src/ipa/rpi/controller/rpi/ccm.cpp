/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ccm.cpp - CCM (colour correction matrix) control algorithm
 */

#include <libcamera/base/log.h>

#include "../awb_status.h"
#include "../ccm_status.h"
#include "../lux_status.h"
#include "../metadata.h"

#include "ccm.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiCcm)

/*
 * This algorithm selects a CCM (Colour Correction Matrix) according to the
 * colour temperature estimated by AWB (interpolating between known matricies as
 * necessary). Additionally the amount of colour saturation can be controlled
 * both according to the current estimated lux level and according to a
 * saturation setting that is exposed to applications.
 */

#define NAME "rpi.ccm"

Matrix::Matrix()
{
	memset(m, 0, sizeof(m));
}
Matrix::Matrix(double m0, double m1, double m2, double m3, double m4, double m5,
	       double m6, double m7, double m8)
{
	m[0][0] = m0, m[0][1] = m1, m[0][2] = m2, m[1][0] = m3, m[1][1] = m4,
	m[1][2] = m5, m[2][0] = m6, m[2][1] = m7, m[2][2] = m8;
}
int Matrix::read(const libcamera::YamlObject &params)
{
	double *ptr = (double *)m;

	if (params.size() != 9) {
		LOG(RPiCcm, Error) << "Wrong number of values in CCM";
		return -EINVAL;
	}

	for (const auto &param : params.asList()) {
		auto value = param.get<double>();
		if (!value)
			return -EINVAL;
		*ptr++ = *value;
	}

	return 0;
}

Ccm::Ccm(Controller *controller)
	: CcmAlgorithm(controller), saturation_(1.0) {}

char const *Ccm::name() const
{
	return NAME;
}

int Ccm::read(const libcamera::YamlObject &params)
{
	int ret;

	if (params.contains("saturation")) {
		ret = config_.saturation.read(params["saturation"]);
		if (ret)
			return ret;
	}

	for (auto &p : params["ccms"].asList()) {
		auto value = p["ct"].get<double>();
		if (!value)
			return -EINVAL;

		CtCcm ctCcm;
		ctCcm.ct = *value;
		ret = ctCcm.ccm.read(p["ccm"]);
		if (ret)
			return ret;

		if (!config_.ccms.empty() && ctCcm.ct <= config_.ccms.back().ct) {
			LOG(RPiCcm, Error)
				<< "CCM not in increasing colour temperature order";
			return -EINVAL;
		}

		config_.ccms.push_back(std::move(ctCcm));
	}

	if (config_.ccms.empty()) {
		LOG(RPiCcm, Error) << "No CCMs specified";
		return -EINVAL;
	}

	return 0;
}

void Ccm::setSaturation(double saturation)
{
	saturation_ = saturation;
}

void Ccm::initialise()
{
}

template<typename T>
static bool getLocked(Metadata *metadata, std::string const &tag, T &value)
{
	T *ptr = metadata->getLocked<T>(tag);
	if (ptr == nullptr)
		return false;
	value = *ptr;
	return true;
}

Matrix calculateCcm(std::vector<CtCcm> const &ccms, double ct)
{
	if (ct <= ccms.front().ct)
		return ccms.front().ccm;
	else if (ct >= ccms.back().ct)
		return ccms.back().ccm;
	else {
		int i = 0;
		for (; ct > ccms[i].ct; i++)
			;
		double lambda =
			(ct - ccms[i - 1].ct) / (ccms[i].ct - ccms[i - 1].ct);
		return lambda * ccms[i].ccm + (1.0 - lambda) * ccms[i - 1].ccm;
	}
}

Matrix applySaturation(Matrix const &ccm, double saturation)
{
	Matrix RGB2Y(0.299, 0.587, 0.114, -0.169, -0.331, 0.500, 0.500, -0.419,
		     -0.081);
	Matrix Y2RGB(1.000, 0.000, 1.402, 1.000, -0.345, -0.714, 1.000, 1.771,
		     0.000);
	Matrix S(1, 0, 0, 0, saturation, 0, 0, 0, saturation);
	return Y2RGB * S * RGB2Y * ccm;
}

void Ccm::prepare(Metadata *imageMetadata)
{
	bool awbOk = false, luxOk = false;
	struct AwbStatus awb = {};
	awb.temperatureK = 4000; /* in case no metadata */
	struct LuxStatus lux = {};
	lux.lux = 400; /* in case no metadata */
	{
		/* grab mutex just once to get everything */
		std::lock_guard<Metadata> lock(*imageMetadata);
		awbOk = getLocked(imageMetadata, "awb.status", awb);
		luxOk = getLocked(imageMetadata, "lux.status", lux);
	}
	if (!awbOk)
		LOG(RPiCcm, Warning) << "no colour temperature found";
	if (!luxOk)
		LOG(RPiCcm, Warning) << "no lux value found";
	Matrix ccm = calculateCcm(config_.ccms, awb.temperatureK);
	double saturation = saturation_;
	struct CcmStatus ccmStatus;
	ccmStatus.saturation = saturation;
	if (!config_.saturation.empty())
		saturation *= config_.saturation.eval(
			config_.saturation.domain().clip(lux.lux));
	ccm = applySaturation(ccm, saturation);
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			ccmStatus.matrix[j * 3 + i] =
				std::max(-8.0, std::min(7.9999, ccm.m[j][i]));
	LOG(RPiCcm, Debug)
		<< "colour temperature " << awb.temperatureK << "K";
	LOG(RPiCcm, Debug)
		<< "CCM: " << ccmStatus.matrix[0] << " " << ccmStatus.matrix[1]
		<< " " << ccmStatus.matrix[2] << "     "
		<< ccmStatus.matrix[3] << " " << ccmStatus.matrix[4]
		<< " " << ccmStatus.matrix[5] << "     "
		<< ccmStatus.matrix[6] << " " << ccmStatus.matrix[7]
		<< " " << ccmStatus.matrix[8];
	imageMetadata->set("ccm.status", ccmStatus);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Ccm(controller);
	;
}
static RegisterAlgorithm reg(NAME, &create);
