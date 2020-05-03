/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ccm.cpp - CCM (colour correction matrix) control algorithm
 */

#include "../awb_status.h"
#include "../ccm_status.h"
#include "../logging.hpp"
#include "../lux_status.h"
#include "../metadata.hpp"

#include "ccm.hpp"

using namespace RPi;

// This algorithm selects a CCM (Colour Correction Matrix) according to the
// colour temperature estimated by AWB (interpolating between known matricies as
// necessary). Additionally the amount of colour saturation can be controlled
// both according to the current estimated lux level and according to a
// saturation setting that is exposed to applications.

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
void Matrix::Read(boost::property_tree::ptree const &params)
{
	double *ptr = (double *)m;
	int n = 0;
	for (auto it = params.begin(); it != params.end(); it++) {
		if (n++ == 9)
			throw std::runtime_error("Ccm: too many values in CCM");
		*ptr++ = it->second.get_value<double>();
	}
	if (n < 9)
		throw std::runtime_error("Ccm: too few values in CCM");
}

Ccm::Ccm(Controller *controller)
	: CcmAlgorithm(controller), saturation_(1.0) {}

char const *Ccm::Name() const
{
	return NAME;
}

void Ccm::Read(boost::property_tree::ptree const &params)
{
	if (params.get_child_optional("saturation"))
		config_.saturation.Read(params.get_child("saturation"));
	for (auto &p : params.get_child("ccms")) {
		CtCcm ct_ccm;
		ct_ccm.ct = p.second.get<double>("ct");
		ct_ccm.ccm.Read(p.second.get_child("ccm"));
		if (!config_.ccms.empty() &&
		    ct_ccm.ct <= config_.ccms.back().ct)
			throw std::runtime_error(
				"Ccm: CCM not in increasing colour temperature order");
		config_.ccms.push_back(std::move(ct_ccm));
	}
	if (config_.ccms.empty())
		throw std::runtime_error("Ccm: no CCMs specified");
}

void Ccm::SetSaturation(double saturation)
{
	saturation_ = saturation;
}

void Ccm::Initialise() {}

template<typename T>
static bool get_locked(Metadata *metadata, std::string const &tag, T &value)
{
	T *ptr = metadata->GetLocked<T>(tag);
	if (ptr == nullptr)
		return false;
	value = *ptr;
	return true;
}

Matrix calculate_ccm(std::vector<CtCcm> const &ccms, double ct)
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

Matrix apply_saturation(Matrix const &ccm, double saturation)
{
	Matrix RGB2Y(0.299, 0.587, 0.114, -0.169, -0.331, 0.500, 0.500, -0.419,
		     -0.081);
	Matrix Y2RGB(1.000, 0.000, 1.402, 1.000, -0.345, -0.714, 1.000, 1.771,
		     0.000);
	Matrix S(1, 0, 0, 0, saturation, 0, 0, 0, saturation);
	return Y2RGB * S * RGB2Y * ccm;
}

void Ccm::Prepare(Metadata *image_metadata)
{
	bool awb_ok = false, lux_ok = false;
	struct AwbStatus awb = {};
	awb.temperature_K = 4000; // in case no metadata
	struct LuxStatus lux = {};
	lux.lux = 400; // in case no metadata
	{
		// grab mutex just once to get everything
		std::lock_guard<Metadata> lock(*image_metadata);
		awb_ok = get_locked(image_metadata, "awb.status", awb);
		lux_ok = get_locked(image_metadata, "lux.status", lux);
	}
	if (!awb_ok)
		RPI_WARN("Ccm: no colour temperature found");
	if (!lux_ok)
		RPI_WARN("Ccm: no lux value found");
	Matrix ccm = calculate_ccm(config_.ccms, awb.temperature_K);
	double saturation = saturation_;
	struct CcmStatus ccm_status;
	ccm_status.saturation = saturation;
	if (!config_.saturation.Empty())
		saturation *= config_.saturation.Eval(
			config_.saturation.Domain().Clip(lux.lux));
	ccm = apply_saturation(ccm, saturation);
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			ccm_status.matrix[j * 3 + i] =
				std::max(-8.0, std::min(7.9999, ccm.m[j][i]));
	RPI_LOG("CCM: colour temperature " << awb.temperature_K << "K");
	RPI_LOG("CCM: " << ccm_status.matrix[0] << " " << ccm_status.matrix[1]
			<< " " << ccm_status.matrix[2] << "     "
			<< ccm_status.matrix[3] << " " << ccm_status.matrix[4]
			<< " " << ccm_status.matrix[5] << "     "
			<< ccm_status.matrix[6] << " " << ccm_status.matrix[7]
			<< " " << ccm_status.matrix[8]);
	image_metadata->Set("ccm.status", ccm_status);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Ccm(controller);
	;
}
static RegisterAlgorithm reg(NAME, &Create);
