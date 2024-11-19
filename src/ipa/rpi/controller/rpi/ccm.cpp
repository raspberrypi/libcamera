/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * CCM (colour correction matrix) control algorithm
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

using Matrix3x3 = Matrix<double, 3, 3>;

Ccm::Ccm(Controller *controller)
	: CcmAlgorithm(controller), saturation_(1.0) {}

char const *Ccm::name() const
{
	return NAME;
}

int Ccm::read(const libcamera::YamlObject &params)
{
	if (params.contains("saturation")) {
		config_.saturation = params["saturation"].get<ipa::Pwl>(ipa::Pwl{});
		if (config_.saturation.empty())
			return -EINVAL;
	}

	for (auto &p : params["ccms"].asList()) {
		auto value = p["ct"].get<double>();
		if (!value)
			return -EINVAL;

		CtCcm ctCcm;
		ctCcm.ct = *value;

		auto ccm = p["ccm"].get<Matrix3x3>();
		if (!ccm)
			return -EINVAL;

		ctCcm.ccm = *ccm;

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

namespace {

template<typename T>
bool getLocked(Metadata *metadata, std::string const &tag, T &value)
{
	T *ptr = metadata->getLocked<T>(tag);
	if (ptr == nullptr)
		return false;
	value = *ptr;
	return true;
}

Matrix3x3 calculateCcm(std::vector<CtCcm> const &ccms, double ct)
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

Matrix3x3 applySaturation(Matrix3x3 const &ccm, double saturation)
{
	static const Matrix3x3 RGB2Y({ 0.299, 0.587, 0.114,
				       -0.169, -0.331, 0.500,
				       0.500, -0.419, -0.081 });

	static const Matrix3x3 Y2RGB({ 1.000, 0.000, 1.402,
				       1.000, -0.345, -0.714,
				       1.000, 1.771, 0.000 });

	Matrix3x3 S({ 1, 0, 0,
		      0, saturation, 0,
		      0, 0, saturation });

	return Y2RGB * S * RGB2Y * ccm;
}

} /* namespace */

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
	Matrix3x3 ccm = calculateCcm(config_.ccms, awb.temperatureK);
	double saturation = saturation_;
	struct CcmStatus ccmStatus;
	ccmStatus.saturation = saturation;
	if (!config_.saturation.empty())
		saturation *= config_.saturation.eval(
			config_.saturation.domain().clamp(lux.lux));
	ccm = applySaturation(ccm, saturation);
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			ccmStatus.matrix[j * 3 + i] =
				std::max(-8.0, std::min(7.9999, ccm[j][i]));
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
