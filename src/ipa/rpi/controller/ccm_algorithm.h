/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * CCM (colour correction matrix) control algorithm interface
 */
#pragma once

#include "libcamera/internal/matrix.h"

#include "algorithm.h"

namespace RPiController {

using Matrix3x3 = libcamera::Matrix<double, 3, 3>;

class CcmAlgorithm : public Algorithm
{
public:
	CcmAlgorithm(Controller *controller) : Algorithm(controller) {}
	/* A CCM algorithm must provide the following: */
	virtual void enableAuto() = 0;
	virtual void setSaturation(double saturation) = 0;
	virtual void setCcm(Matrix3x3 const &matrix) = 0;
};

} /* namespace RPiController */
